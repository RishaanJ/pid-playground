// Numerically verifies every level in the playground is winnable, and that the
// intended gain is the one that wins it. Mirrors the sim core exactly.
const DEG = 180 / Math.PI;
const SUB = 0.002;
const BASE = { GRAV: 3.0, DAMP: 2.4, INERTIA: 1.0, VMAX: 12.0 };
const T0 = Math.PI / 3;

function mulberry(seed) {
  return function () {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function simulate(env, g, mm, seconds, seed = 7) {
  const rnd = mulberry(seed);
  let theta = 0, omega = 0, integral = 0, prevErr = 0, prevOut = 0;
  let profPos = 0, profVel = 0, browned = false;
  let sumChatter = 0, nCh = 0;
  const rec = [];
  let initialised = false;

  const steps = Math.round(seconds / SUB);
  for (let n = 0; n < steps; n++) {
    const t = n * SUB;

    // ---- environment ----
    let target = T0;
    if (env.sequence) {
      for (const s of env.sequence) if (t >= s.t) target = s.ang / DEG;
    }
    let gravMul = 1;
    if (env.massAt && t >= env.massAt.t) gravMul = env.massAt.mul;

    let vmax = BASE.VMAX;
    if (env.brownout) {
      // sustained hard pushing sags the battery
      if (Math.abs(prevOut) > env.brownout.thresh) {
        vmax = env.brownout.sagged;
        if (Math.abs(prevOut) > env.brownout.thresh + 1) browned = true;
      }
    }

    // ---- sensor ----
    let meas = theta;
    if (env.noise) {
      if (env.noise.tick) meas = Math.round(meas / env.noise.tick) * env.noise.tick;
      if (env.noise.sigma) meas += (rnd() - 0.5) * 2 * env.noise.sigma;
    }

    // ---- profile ----
    let cmd;
    if (mm) {
      if (!initialised) { profPos = theta; initialised = true; }
      const CRUISE = 1.0, ACCEL = 1.6;
      const remain = target - profPos, dir = remain > 0 ? 1 : -1;
      const vStop = Math.sqrt(Math.max(0, 2 * ACCEL * Math.abs(remain)));
      const vWant = dir * Math.min(CRUISE, vStop), maxDv = ACCEL * SUB;
      profVel += Math.max(-maxDv, Math.min(maxDv, vWant - profVel));
      profPos += profVel * SUB;
      if (Math.abs(target - profPos) < 0.002 && Math.abs(profVel) < 0.02) { profPos = target; profVel = 0; }
      cmd = profPos;
    } else cmd = target;

    const err = cmd - meas;
    if (n === 0) prevErr = err;
    integral = Math.max(-8, Math.min(8, integral + err * SUB));
    const deriv = (err - prevErr) / SUB;
    prevErr = err;

    let out = g.P * err + g.I * integral + g.D * deriv + g.G * Math.cos(meas);
    if (g.S) out += g.S * (err > 0.004 ? 1 : err < -0.004 ? -1 : 0);
    out = Math.max(-vmax, Math.min(vmax, out));

    sumChatter += Math.abs(out - prevOut); nCh++;
    prevOut = out;

    // ---- plant ----
    let torque = out - BASE.GRAV * gravMul * Math.cos(theta) - BASE.DAMP * omega;

    if (env.disturb && t > 1.0) {
      const k = Math.floor(t / env.disturb.every);
      const phase = t - k * env.disturb.every;
      if (phase < 0.08) torque += (k % 2 ? -1 : 1) * env.disturb.mag;
    }

    let accel = torque / BASE.INERTIA;

    if (env.stiction) {
      // must overcome static friction before anything moves
      const net = out - BASE.GRAV * gravMul * Math.cos(theta);
      if (Math.abs(omega) < 0.02 && Math.abs(net) < env.stiction) { accel = 0; omega = 0; }
    }

    omega += accel * SUB;
    theta += omega * SUB;
    if (theta < -0.35) { theta = -0.35; if (omega < 0) omega = 0; }
    if (theta > 2.9) { theta = 2.9; if (omega > 0) omega = 0; }

    if (n % 10 === 0) rec.push({ t, a: theta, cmd, target, out });
  }

  // ---- metrics ----
  const tgtDeg = rec[rec.length - 1].target * DEG;
  let peak = -1e9, minErr = 1e9;
  for (const r of rec) {
    if (Math.abs(r.target * DEG - tgtDeg) < 0.01) {
      peak = Math.max(peak, r.a * DEG);
      minErr = Math.min(minErr, Math.abs(tgtDeg - r.a * DEG));
    }
  }
  const tail = rec.slice(-40);
  const settleErr = tail.reduce((s, r) => s + Math.abs(r.target * DEG - r.a * DEG), 0) / tail.length;

  // worst error across the last 3 s (disturbance / mass-change recovery)
  const late = rec.filter(r => r.t > rec[rec.length - 1].t - 3);
  const worstLate = late.reduce((m, r) => Math.max(m, Math.abs(r.target * DEG - r.a * DEG)), 0);

  // Waypoint accuracy: how far off it was at the END of each commanded segment.
  // Average tracking error would punish a smooth profile for taking its time,
  // which is exactly the behaviour we want to reward.
  let waypointErr = 0;
  if (env.sequence) {
    const bounds = env.sequence.map((s, i) =>
      i + 1 < env.sequence.length ? env.sequence[i + 1].t : seconds);
    for (let i = 0; i < bounds.length; i++) {
      const at = bounds[i] - 0.1;
      let best = null;
      for (const r of rec) if (r.t <= at && (!best || r.t > best.t)) best = r;
      if (best) waypointErr = Math.max(waypointErr, Math.abs(best.target * DEG - best.a * DEG));
    }
  }

  return {
    overshoot: Math.max(0, peak - tgtDeg),
    settleErr, minErr, worstLate, waypointErr,
    chatter: sumChatter / nCh,
    browned,
    finite: rec.every(r => Number.isFinite(r.a))
  };
}

// ---------------- level definitions (must match the HTML) ----------------
const LEVELS = [
  { n: "1 Lift the arm",      env: {}, dur: 6, pass: m => m.minErr < 10 },
  { n: "2 Stop the bounce",   env: {}, dur: 6, pass: m => m.minErr < 10 && m.overshoot < 5 },
  { n: "3 Kill the sag",      env: {}, dur: 6, pass: m => m.settleErr < 1 },
  { n: "4 Smooth operator",   env: {}, dur: 7, mmReq: true, pass: m => m.settleErr < 1 && m.overshoot < 4 },
  { n: "5 Noisy encoder",     env: { noise: { tick: 0.006, sigma: 0.0025 } }, dur: 7, pass: m => m.settleErr < 2 && m.chatter < 1.0 },
  { n: "6 Sticky gearbox",    env: { stiction: 1.1 }, dur: 7, pass: m => m.settleErr < 1.5 },
  { n: "7 Brownout battery",  env: { brownout: { thresh: 8, sagged: 7 } }, dur: 7, pass: m => m.settleErr < 1.5 && !m.browned },
  { n: "8 Game piece",        env: { massAt: { t: 3.0, mul: 2.2 } }, dur: 10, pass: m => m.worstLate < 3 },
  { n: "9 Playing defense",   env: { disturb: { every: 1.5, mag: 7 } }, dur: 8, pass: m => m.worstLate < 6 },
  { n: "10 Match sequence",   env: { sequence: [{ t: 0, ang: 60 }, { t: 2.5, ang: 12 }, { t: 5, ang: 85 }] }, dur: 8, pass: m => m.waypointErr < 3 },
];

// candidate tunings a student might land on
const TUNINGS = [
  ["zeros",            { P: 0, I: 0, D: 0, G: 0, S: 0 }, false],
  ["P only lo",        { P: 6, I: 0, D: 0, G: 0, S: 0 }, false],
  ["P only hi",        { P: 16, I: 0, D: 0, G: 0, S: 0 }, false],
  ["P+D",              { P: 16, I: 0, D: 2.5, G: 0, S: 0 }, false],
  ["P+D+G",            { P: 16, I: 0, D: 2.5, G: 3, S: 0 }, false],
  ["P+D+G +MM",        { P: 16, I: 0, D: 2.5, G: 3, S: 0 }, true],
  ["low-D +G +MM",     { P: 10, I: 0, D: 0.4, G: 3, S: 0 }, true],
  ["P+D+G+I",          { P: 16, I: 1.0, D: 2.5, G: 3, S: 0 }, false],
  ["P+D+G+S",          { P: 16, I: 0, D: 2.5, G: 3, S: 1.2 }, false],
  ["P+D+G+S +MM",      { P: 12, I: 0, D: 1.5, G: 3, S: 1.2 }, true],
  ["gentle +G +MM",    { P: 7, I: 0.3, D: 1.0, G: 3, S: 0 }, true],
  ["stiff +D +G",      { P: 20, I: 0.5, D: 3.5, G: 3, S: 0 }, false],
];

let allOk = true;
for (const L of LEVELS) {
  const winners = [];
  for (const [name, g, mm] of TUNINGS) {
    if (L.mmReq && !mm) continue;
    const m = simulate(L.env, g, mm, L.dur);
    if (!m.finite) { console.log("  NON-FINITE in", L.n, name); allOk = false; }
    if (L.pass(m)) winners.push(name);
  }
  const ok = winners.length > 0;
  if (!ok) allOk = false;
  console.log(
    (ok ? "PASS " : "FAIL ") + L.n.padEnd(22) +
    (winners.length ? "winners: " + winners.join(", ") : "*** NO TUNING WINS ***")
  );
}
console.log(allOk ? "\nAll levels are winnable." : "\nSome levels are unwinnable — fix before shipping.");
