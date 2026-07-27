// Flywheel plant: velocity control, not position.
// First-order lag:  rpm' = (K*V - rpm) / tau
// K   = free speed per volt (RPM/V)
// tau = spin-up time constant (s)
const K = 415, TAU = 0.35, VMAX = 12, SUB = 0.002;
const TARGET = 4000;

// Gains are scaled per-1000-RPM so the sliders land in friendly ranges.
//   P = kP * err/1000
//   I = kI * integral/1000
//   D = kD * deriv/100000
//   V = kV * target/1000        <- feedforward, the dominant term
function run(g, env, seconds) {
  let rpm = 0, integral = 0, prevErr = TARGET, prevOut = 0;
  const rec = [];
  const shots = [];
  const steps = Math.round(seconds / SUB);

  for (let n = 0; n < steps; n++) {
    const t = n * SUB;

    // a game piece passing through robs energy instantly
    if (env.shotEvery && t > env.shotFirst) {
      const k = Math.floor((t - env.shotFirst) / env.shotEvery);
      const phase = (t - env.shotFirst) - k * env.shotEvery;
      if (phase < SUB) { rpm -= env.shotLoss; shots.push(t); }
    }

    const err = TARGET - rpm;
    integral = Math.max(-40000, Math.min(40000, integral + err * SUB));
    const deriv = (err - prevErr) / SUB;
    prevErr = err;

    let out = g.P * err / 1000 + g.I * integral / 1000 + g.D * deriv / 100000 + g.V * TARGET / 1000;
    // a flywheel motor can push but you never actively brake it
    out = Math.max(0, Math.min(VMAX, out));
    prevOut = out;

    rpm += ((K * out - rpm) / TAU) * SUB;
    if (rpm < 0) rpm = 0;

    if (n % 10 === 0) rec.push({ t, rpm, out });
  }

  // ---- metrics ----
  const tail = rec.slice(-40);
  const settleErr = tail.reduce((s, r) => s + Math.abs(TARGET - r.rpm), 0) / tail.length;

  let spinUpT = null;
  for (const r of rec) { if (spinUpT === null && Math.abs(TARGET - r.rpm) < 100) spinUpT = r.t; }

  // worst dip + how long it takes to climb back within 75 RPM after each shot
  let maxDip = 0, worstRecover = 0;
  for (const st of shots) {
    let lowest = TARGET;
    for (const r of rec) if (r.t >= st && r.t < st + 2.0) lowest = Math.min(lowest, r.rpm);
    maxDip = Math.max(maxDip, TARGET - lowest);
    let back = null;
    for (const r of rec) {
      if (r.t > st + 0.05 && back === null && Math.abs(TARGET - r.rpm) < 75) back = r.t - st;
    }
    worstRecover = Math.max(worstRecover, back === null ? 99 : back);
  }

  const late = rec.filter(r => r.t > seconds - 4);
  const meanErr = late.reduce((s, r) => s + Math.abs(TARGET - r.rpm), 0) / (late.length || 1);

  return {
    settleErr, spinUpT, maxDip, worstRecover, meanErr,
    finite: rec.every(r => Number.isFinite(r.rpm)),
    endRpm: rec[rec.length - 1].rpm
  };
}

console.log("ideal feedforward kV = " + (TARGET / K / (TARGET / 1000)).toFixed(3) +
            "   (holding " + TARGET + " RPM needs " + (TARGET / K).toFixed(2) + " V)\n");

const tunings = [
  ["nothing",            { P: 0,   I: 0,   D: 0, V: 0 }],
  ["kP only 1.0",        { P: 1.0, I: 0,   D: 0, V: 0 }],
  ["kP only 3.0",        { P: 3.0, I: 0,   D: 0, V: 0 }],
  ["kP only 8.0",        { P: 8.0, I: 0,   D: 0, V: 0 }],
  ["kV only 2.4",        { P: 0,   I: 0,   D: 0, V: 2.4 }],
  ["kV 2.4 + kP 2",      { P: 2,   I: 0,   D: 0, V: 2.4 }],
  ["kV 2.4 +P2 +I1",     { P: 2,   I: 1,   D: 0, V: 2.4 }],
  ["kV 2.0 +P3 +I2",     { P: 3,   I: 2,   D: 0, V: 2.0 }],
  ["kV 2.4 +P5 +I2",     { P: 5,   I: 2,   D: 0, V: 2.4 }],
  ["kV 2.4 +P5 +I2 +D1", { P: 5,   I: 2,   D: 1, V: 2.4 }],
];

console.log("== L11 spin up (no shots, 5 s) ==");
console.log("tuning".padEnd(22), "settleErr".padStart(10), "spinUp".padStart(8), "endRPM".padStart(9));
for (const [n, g] of tunings) {
  const r = run(g, {}, 5);
  console.log(n.padEnd(22),
    (r.settleErr.toFixed(0) + " rpm").padStart(10),
    (r.spinUpT === null ? "never" : r.spinUpT.toFixed(2) + "s").padStart(8),
    r.endRpm.toFixed(0).padStart(9),
    r.finite ? "" : " NON-FINITE");
}

console.log("\n== L12 rapid fire (shot every 2 s, -350 RPM, 10 s) ==");
const env = { shotEvery: 2.0, shotFirst: 3.0, shotLoss: 350 };
console.log("tuning".padEnd(22), "maxDip".padStart(8), "recover".padStart(9), "meanErr".padStart(9));
for (const [n, g] of tunings) {
  const r = run(g, env, 10);
  console.log(n.padEnd(22),
    r.maxDip.toFixed(0).padStart(8),
    (r.worstRecover > 90 ? "never" : r.worstRecover.toFixed(2) + "s").padStart(9),
    r.meanErr.toFixed(0).padStart(9));
}
