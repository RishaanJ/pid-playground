// Sandbox-mode physics checks.
//
// Sandbox mode models a real mechanism in SI units so that each student can be
// given a different arm/elevator and has to work out their own gains. That is
// only worth anything if the physics is honest, so this file asserts the
// properties that make it honest - above all: zero gains means zero motor
// output and the mechanism just falls.

var MOTORS = {
  "Kraken X60": { stall: 7.09, freeRpm: 6000 },
  "Falcon 500": { stall: 4.69, freeRpm: 6380 },
  "NEO":        { stall: 2.60, freeRpm: 5676 },
  "NEO 550":    { stall: 0.97, freeRpm: 11000 },
  "CIM":        { stall: 2.41, freeRpm: 5330 }
};

var G_ACC = 9.81, VBUS = 12, SUB = 0.001;

function freeRad(m) { return MOTORS[m].freeRpm * 2 * Math.PI / 60; }

// Torque delivered at the mechanism, given applied volts and mechanism speed.
// Standard brushed-DC line: torque falls linearly from stall to free speed.
function mechTorque(cfg, volts, omegaMech) {
  var M = MOTORS[cfg.motor];
  var omegaMotor = omegaMech * cfg.ratio;
  var tau = M.stall * (volts / VBUS) - M.stall * omegaMotor / freeRad(cfg.motor);
  return tau * cfg.motors * cfg.ratio * cfg.eff;
}

// ---- ideal feedforward, derived analytically (this is the "answer key") ----
function idealKG(cfg) {
  var M = MOTORS[cfg.motor];
  var load = cfg.kind === "arm"
    ? cfg.mass * G_ACC * cfg.len          // holding torque at horizontal
    : cfg.mass * G_ACC * cfg.drum;        // torque to hold the carriage
  return VBUS * load / (M.stall * cfg.motors * cfg.ratio * cfg.eff);
}

function idealKV(cfg) {
  var freeMechRad = freeRad(cfg.motor) / cfg.ratio;
  if (cfg.kind === "arm") return VBUS / freeMechRad;        // V per rad/s
  return VBUS / (freeMechRad * cfg.drum);                   // V per m/s
}

// ---- plant ----
function simulate(cfg, gains, opts) {
  opts = opts || {};
  var target = opts.target !== undefined ? opts.target : (cfg.kind === "arm" ? 0 : 0.5);
  var seconds = opts.seconds || 6;

  var pos = opts.start !== undefined ? opts.start : (cfg.kind === "arm" ? -Math.PI / 2 : 0);
  var vel = 0, integral = 0, prevErr = target - pos;

  var maxVolts = 0, maxPos = -Infinity, minPos = Infinity;
  var steps = Math.round(seconds / SUB);

  for (var n = 0; n < steps; n++) {
    var err = target - pos;
    integral += err * SUB;
    var deriv = (err - prevErr) / SUB;
    prevErr = err;

    var grav = cfg.kind === "arm" ? Math.cos(pos) : 1;   // arm: gravity load varies with angle
    var volts = gains.P * err + gains.I * integral + gains.D * deriv +
                gains.G * grav + gains.V * vel +
                (gains.S ? gains.S * (vel > 1e-6 ? 1 : vel < -1e-6 ? -1 : 0) : 0);

    volts = Math.max(-VBUS, Math.min(VBUS, volts));
    if (Math.abs(volts) > maxVolts) maxVolts = Math.abs(volts);

    var tau = mechTorque(cfg, volts, vel);
    var acc;
    if (cfg.kind === "arm") {
      var J = cfg.mass * cfg.len * cfg.len;               // point mass at the end
      acc = (tau - cfg.mass * G_ACC * cfg.len * Math.cos(pos) - cfg.damp * vel) / J;
    } else {
      var F = tau / cfg.drum;
      acc = (F - cfg.mass * G_ACC - cfg.damp * vel) / cfg.mass;
    }

    vel += acc * SUB;
    pos += vel * SUB;

    // hard stops
    if (cfg.kind === "arm") {
      if (pos < -Math.PI / 2) { pos = -Math.PI / 2; if (vel < 0) vel = 0; }
      if (pos > Math.PI / 2) { pos = Math.PI / 2; if (vel > 0) vel = 0; }
    } else {
      if (pos < 0) { pos = 0; if (vel < 0) vel = 0; }
      if (pos > cfg.travel) { pos = cfg.travel; if (vel > 0) vel = 0; }
    }

    if (pos > maxPos) maxPos = pos;
    if (pos < minPos) minPos = pos;
  }

  return { pos: pos, vel: vel, maxVolts: maxVolts, maxPos: maxPos, minPos: minPos,
           finite: isFinite(pos) && isFinite(vel) };
}

// ---- deterministic per-student assignment ----
function hash(str) {
  var h = 2166136261;
  for (var i = 0; i < str.length; i++) { h ^= str.charCodeAt(i); h = (h * 16777619) >>> 0; }
  return h >>> 0;
}

function assignment(seedStr) {
  var h = hash(String(seedStr));
  function pick(arr, shift) { return arr[(h >>> shift) % arr.length]; }
  function span(lo, hi, shift) { return lo + (((h >>> shift) % 1000) / 1000) * (hi - lo); }

  var kind = pick(["arm", "elevator"], 3);
  var motor = pick(["Kraken X60", "Falcon 500", "NEO", "NEO 550", "CIM"], 7);
  var cfg = {
    kind: kind,
    motor: motor,
    motors: pick([1, 1, 2, 2, 4], 11),
    ratio: Math.round(span(20, 160, 13)),
    eff: 0.85,
    damp: kind === "arm" ? span(0.4, 2.5, 17) : span(2, 14, 17),
    mass: kind === "arm" ? span(2, 9, 19) : span(4, 16, 19)
  };
  if (kind === "arm") { cfg.len = span(0.3, 1.1, 23); }
  else { cfg.drum = span(0.018, 0.045, 23); cfg.travel = span(0.8, 1.8, 29); }
  return cfg;
}

// ============================ tests ============================
var fails = 0;
function check(name, cond, detail) {
  if (!cond) { fails++; console.log("FAIL  " + name + (detail ? "  -> " + detail : "")); }
  else console.log("ok    " + name + (detail ? "  (" + detail + ")" : ""));
}

console.log("=== 1. zero gains must mean zero motor output (the YamGen bug) ===");
for (var s = 1; s <= 40; s++) {
  var cfg = assignment("student-" + s);
  var start = cfg.kind === "arm" ? 0 : cfg.travel * 0.6;   // start it up high
  var r = simulate(cfg, { P: 0, I: 0, D: 0, G: 0, S: 0, V: 0 }, { start: start, seconds: 5 });
  if (r.maxVolts > 1e-9) { fails++; console.log("FAIL seed " + s + " applied " + r.maxVolts + " V with all gains zero"); }
  // it must never end up higher than it started - gravity only pulls down
  if (r.maxPos > start + 1e-6) { fails++; console.log("FAIL seed " + s + " rose above its start with no gains (" + r.maxPos + " > " + start + ")"); }
}
check("40 random mechanisms: 0 V applied, never rises", fails === 0);

console.log("\n=== 2. it actually falls (not frozen) ===");
// Gravity torque on an arm goes as cos(theta), so it approaches hanging
// asymptotically - it is still descending at 4 s. Give it time to arrive.
var af = { kind: "arm", motor: "NEO", motors: 2, ratio: 60, eff: 0.85,
           damp: 1.0, mass: 5, len: 0.7 };
var zero = { P: 0, I: 0, D: 0, G: 0, S: 0, V: 0 };
var fellMid = simulate(af, zero, { start: 0, seconds: 2 });
var fellEnd = simulate(af, zero, { start: 0, seconds: 10 });
check("arm is descending at 2 s", fellMid.pos < -0.5 && fellMid.vel < 0,
      (fellMid.pos * 180 / Math.PI).toFixed(1) + " deg");
check("arm settles hanging", Math.abs(fellEnd.pos + Math.PI / 2) < 0.02,
      (fellEnd.pos * 180 / Math.PI).toFixed(2) + " deg");

var ef = { kind: "elevator", motor: "NEO", motors: 2, ratio: 40, eff: 0.85, damp: 6,
           mass: 8, drum: 0.03, travel: 1.5 };
var eMid = simulate(ef, zero, { start: 1.0, seconds: 2 });
var eEnd = simulate(ef, zero, { start: 1.0, seconds: 10 });
check("elevator is descending at 2 s", eMid.pos < 0.95 && eMid.vel < 0,
      eMid.pos.toFixed(3) + " m at " + eMid.vel.toFixed(3) + " m/s");
check("elevator reaches the bottom", eEnd.pos < 0.01, eEnd.pos.toFixed(4) + " m");

console.log("\n=== 3. the analytic kG really does hold the load ===");
var held = 0, total = 0;
for (var s2 = 1; s2 <= 40; s2++) {
  var c = assignment("student-" + s2);
  var kg = idealKG(c);
  if (kg > VBUS) continue;                       // geared too weakly to hold; skip
  total++;
  var startPos = c.kind === "arm" ? 0 : c.travel * 0.5;
  var h = simulate(c, { P: 0, I: 0, D: 0, G: kg, S: 0, V: 0 },
                   { start: startPos, seconds: 3, target: startPos });
  var drift = Math.abs(h.pos - startPos);
  var tol = c.kind === "arm" ? 0.09 : 0.05;      // rad / m
  if (drift < tol) held++;
  else console.log("  seed " + s2 + " (" + c.kind + ") drifted " + drift.toFixed(3) + " with kG=" + kg.toFixed(2));
}
check("ideal kG holds position", held === total, held + "/" + total + " mechanisms hold");

console.log("\n=== 4. kG is genuinely different per student (homework is unique) ===");
var seen = {}, dupes = 0;
for (var s3 = 1; s3 <= 30; s3++) {
  var cc = assignment("student-" + s3);
  var key = idealKG(cc).toFixed(2) + "/" + idealKV(cc).toFixed(3);
  if (seen[key]) dupes++;
  seen[key] = 1;
}
check("30 students, no duplicate answer keys", dupes === 0, dupes + " duplicates");

console.log("\n=== 5. every assignment is tunable to its target ===");
var solved = 0, tried = 0;
for (var s4 = 1; s4 <= 40; s4++) {
  var c4 = assignment("student-" + s4);
  var kg4 = idealKG(c4);
  if (kg4 > VBUS * 0.8) continue;                // under-geared: not a fair exercise
  tried++;
  var tgt = c4.kind === "arm" ? 0.6 : Math.min(0.8, c4.travel * 0.6);
  var tol = c4.kind === "arm" ? 0.12 : 0.06;
  // Mechanisms differ by orders of magnitude, so one fixed tuning cannot serve
  // them all - that is the point of the exercise. Search a modest grid and
  // require only that SOME sensible tuning solves it.
  var kps = c4.kind === "arm" ? [10, 20, 30, 50, 80, 120] : [30, 60, 100, 200, 400, 800];
  var kds = c4.kind === "arm" ? [2, 5, 10, 20, 40] : [5, 10, 20, 40, 80];
  var best = Infinity, bestAt = "";
  for (var a = 0; a < kps.length; a++) {
    for (var b = 0; b < kds.length; b++) {
      var r4 = simulate(c4, { P: kps[a], I: 0, D: kds[b], G: kg4, S: 0, V: 0 },
                        { start: c4.kind === "arm" ? -Math.PI / 2 : 0, target: tgt, seconds: 8 });
      if (!r4.finite) continue;
      var e4 = Math.abs(tgt - r4.pos);
      if (e4 < best) { best = e4; bestAt = "kP=" + kps[a] + " kD=" + kds[b]; }
    }
  }
  if (best < tol) solved++;
  else console.log("  seed " + s4 + " (" + c4.kind + ") best error " + best.toFixed(3) + " at " + bestAt);
}
check("assignments reachable with kG + kP + kD", solved === tried, solved + "/" + tried);

console.log("\n=== 6. no NaN / instability across the whole parameter space ===");
var bad = 0;
for (var s5 = 1; s5 <= 200; s5++) {
  var c5 = assignment("x" + s5);
  var r5 = simulate(c5, { P: 80, I: 5, D: 20, G: idealKG(c5), S: 0, V: 0 }, { seconds: 4 });
  if (!r5.finite) { bad++; console.log("  seed x" + s5 + " went non-finite"); }
}
check("200 mechanisms stay finite under aggressive gains", bad === 0);


// ============================================================
// 7. the advanced/realism layer must genuinely change the difficulty
// ============================================================
// Mirrors the in-page sbStep, including control-loop rate, sensor delay,
// noise, gearbox backlash and stiction.
function simulateReal(cfg, gains, o) {
  o = o || {};
  var loopHz = o.loopHz || 500, delay = o.delay || 0, noise = o.noise || 0,
      backlash = o.backlash || 0, stiction = o.stiction || 0;
  var target = o.target !== undefined ? o.target : (cfg.kind === "arm" ? Math.PI / 3 : 0.5);
  var seconds = o.seconds || 8;

  var pos = cfg.kind === "arm" ? -Math.PI / 2 : 0, load = pos, vel = 0;
  var integral = 0, prevErr = target - pos, volts = 0, ctrlAcc = 1e9;
  var hist = [], sd = 1;
  function rnd() { sd = (sd * 1103515245 + 12345) & 0x7fffffff; return sd / 0x7fffffff * 2 - 1; }

  var peak = -Infinity, flips = 0, lastSign = 0, sum = 0, cnt = 0;
  var steps = Math.round(seconds / SUB);

  for (var n = 0; n < steps; n++) {
    var t = n * SUB;
    var meas = load + (noise ? rnd() * noise : 0);
    hist.push(meas);
    var sensed = hist[Math.max(0, hist.length - 1 - Math.round(delay / SUB))];

    var period = 1 / loopHz;
    ctrlAcc += SUB;
    if (ctrlAcc >= period - 1e-9) {
      ctrlAcc = 0;
      var err = target - sensed;
      integral = Math.max(-50, Math.min(50, integral + err * period));
      var der = (err - prevErr) / period;
      prevErr = err;
      var grav = cfg.kind === "arm" ? Math.cos(sensed) : 1;
      volts = gains.P * err + gains.I * integral + gains.D * der + gains.G * grav;
      volts = Math.max(-VBUS_, Math.min(VBUS_, volts));
    }

    var tau = mechTorque(cfg, volts, vel), net, inertia;
    if (cfg.kind === "arm") {
      inertia = cfg.mass * cfg.len * cfg.len;
      net = tau - cfg.mass * G_ACC * cfg.len * Math.cos(load) - cfg.damp * vel;
    } else {
      inertia = cfg.mass;
      net = tau / cfg.drum - cfg.mass * G_ACC - cfg.damp * vel;
    }
    if (stiction) {
      if (Math.abs(vel) < 1e-3 && Math.abs(net) < stiction) net = 0;
      else if (Math.abs(vel) >= 1e-3) net -= stiction * (vel > 0 ? 1 : -1);
    }
    vel += (net / inertia) * SUB;
    pos += vel * SUB;

    if (backlash) {
      var half = backlash / 2;
      if (pos - load > half) load = pos - half;
      else if (load - pos > half) load = pos + half;
    } else load = pos;

    if (cfg.kind === "arm") {
      if (load < -Math.PI / 2) { load = pos = -Math.PI / 2; if (vel < 0) vel = 0; }
      if (load > Math.PI / 2) { load = pos = Math.PI / 2; if (vel > 0) vel = 0; }
    } else {
      if (load < 0) { load = pos = 0; if (vel < 0) vel = 0; }
      if (load > cfg.travel) { load = pos = cfg.travel; if (vel > 0) vel = 0; }
    }

    if (t > 0.3) {
      if (load > peak) peak = load;
      var sg = (target - load) > 0 ? 1 : -1;
      if (lastSign && sg !== lastSign) flips++;
      lastSign = sg;
    }
    if (t > seconds - 2) { sum += Math.abs(target - load); cnt++; }
  }
  var settle = cnt ? sum / cnt : 99;
  var toDeg = cfg.kind === "arm" ? 180 / Math.PI : 100;
  return {
    settle: settle * toDeg, overshoot: (peak - target) * toDeg, flips: flips,
    ok: isFinite(settle) && settle * toDeg < 2.5 && (peak - target) * toDeg < 6 && flips < 8
  };
}
var VBUS_ = 12;

console.log("\n=== 7. the realism layer actually bites ===");
var rc = { kind: "arm", motor: "NEO", motors: 2, ratio: 60, eff: 0.85,
           damp: 1.0, mass: 5, len: 0.7 };
var rg = { P: 40, I: 0, D: 8, G: idealKG(rc) };
var REAL = { loopHz: 50, delay: 0.020, noise: 0.3 * Math.PI / 180,
             backlash: 0.5 * Math.PI / 180, stiction: 3 };

var ideal = simulateReal(rc, rg, {});
var real = simulateReal(rc, rg, REAL);
check("a good tuning passes the ideal plant", ideal.ok,
      "settle " + ideal.settle.toFixed(2) + " deg, " + ideal.flips + " wobbles");
check("the same tuning FAILS the realistic plant", !real.ok,
      "settle " + real.settle.toFixed(2) + " deg, " + real.flips + " wobbles");

// defaults must leave behaviour untouched
var defaultsSame = simulateReal(rc, rg, { loopHz: 500, delay: 0, noise: 0, backlash: 0, stiction: 0 });
check("realism at defaults == ideal", Math.abs(defaultsSame.settle - ideal.settle) < 1e-9);

// and it must still be winnable with the right gains
var win = null;
for (var p7 = 5; p7 <= 60 && !win; p7 += 5) {
  for (var d7 = 0; d7 <= 6; d7 += 0.5) {
    var r7 = simulateReal(rc, { P: p7, I: 0, D: d7, G: idealKG(rc) }, REAL);
    if (r7.ok) { win = "kP=" + p7 + " kD=" + d7; break; }
  }
}
check("realistic plant is still winnable", !!win, win || "no tuning found");

console.log(fails === 0 ? "\nAll sandbox physics checks PASSED." : "\n" + fails + " CHECK(S) FAILED.");

process.exit(fails === 0 ? 0 : 1);
