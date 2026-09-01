import { chromium } from "playwright";
import { spawn, execFileSync } from "node:child_process";
import { createHash } from "node:crypto";
import { mkdir, readFile, readdir, writeFile } from "node:fs/promises";
import { join, resolve } from "node:path";
import http from "node:http";

const PORT = 4173;
const BASE = `http://127.0.0.1:${PORT}`;
const ARTIFACTS = resolve("artifacts");
await mkdir(ARTIFACTS, { recursive: true });
await mkdir(join(ARTIFACTS, "video"), { recursive: true });

const logs = [];
const startedAt = new Date().toISOString();
const server = spawn(
  "npm",
  ["run", "preview", "--", "--host", "127.0.0.1", "--port", String(PORT), "--strictPort"],
  { stdio: ["ignore", "pipe", "pipe"] },
);
server.stdout.on("data", (b) => logs.push(`[server] ${b.toString()}`));
server.stderr.on("data", (b) => logs.push(`[server:err] ${b.toString()}`));

function waitForHttp(url, timeoutMs = 60_000) {
  const deadline = Date.now() + timeoutMs;
  return new Promise((resolvePromise, reject) => {
    const attempt = () => {
      const req = http.get(url, (res) => {
        res.resume();
        if (res.statusCode && res.statusCode < 500) return resolvePromise();
        setTimeout(attempt, 250);
      });
      req.on("error", () => {
        if (Date.now() > deadline) reject(new Error(`server not ready: ${url}`));
        else setTimeout(attempt, 250);
      });
    };
    attempt();
  });
}

const round = (n, d = 5) => Number(Number(n).toFixed(d));
const rad2deg = (r) => (r * 180) / Math.PI;
function wrapRad(v) {
  while (v > Math.PI) v -= 2 * Math.PI;
  while (v < -Math.PI) v += 2 * Math.PI;
  return v;
}
function distance2(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}
function maxOf(arr, fn) {
  return arr.reduce((m, x) => Math.max(m, fn(x)), -Infinity);
}

async function sha256File(path) {
  const data = await readFile(path);
  return createHash("sha256").update(data).digest("hex");
}

async function listEvidenceHashes() {
  const out = {};
  for (const dir of ["public/policies", "public/robot/mjlab"]) {
    let names = [];
    try { names = await readdir(dir); } catch { continue; }
    for (const name of names) {
      if (!/\.(onnx|xml|glb|json)$/i.test(name)) continue;
      const path = join(dir, name);
      out[path] = await sha256File(path);
    }
  }
  return out;
}

function hookPage(page, label) {
  page.on("console", (msg) => logs.push(`[${label}:console:${msg.type()}] ${msg.text()}`));
  page.on("pageerror", (err) => logs.push(`[${label}:pageerror] ${err.stack || err.message}`));
  page.on("requestfailed", (req) => logs.push(`[${label}:requestfailed] ${req.url()} ${req.failure()?.errorText || ""}`));
}

async function waitForReady(page) {
  await page.waitForFunction(
    () => Boolean(window.rl?.data && window.rl?.model && window.rl?.mujoco && window.rl?.ort),
    null,
    { timeout: 180_000 },
  );
  await page.waitForFunction(() => window.rl && !window.rl.inputLocked, null, { timeout: 90_000 });
  await page.evaluate(() => { window.rl.chaseCam = false; });
}

async function state(page) {
  return page.evaluate(() => {
    const rl = window.rl;
    const q = rl.data.qpos;
    const v = rl.data.qvel;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    let gravityZ = null;
    try { gravityZ = Number(rl.buildObs()[5]); } catch {}
    return {
      wallMs: performance.now(),
      mode: rl.mode,
      loco: rl.loco,
      inputLocked: rl.inputLocked,
      recovery: rl.recovery,
      root: Array.from(q.slice(0, 7), Number),
      rootVelocity: Array.from(v.slice(0, 6), Number),
      yaw,
      gravityZ,
      ballActive: rl.ballActive,
      ball: adr == null ? null : Array.from(q.slice(adr, adr + 7), Number),
    };
  });
}

async function positionBall(page, forward = 0.18, lateral = 0) {
  return page.evaluate(({ forward, lateral }) => {
    const rl = window.rl;
    if (!rl.ballActive) rl.spawnBall();
    const q = rl.data.qpos;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const fx = Math.cos(yaw), fy = Math.sin(yaw);
    const lx = -fy, ly = fx;
    q[adr] = q[0] + fx * forward + lx * lateral;
    q[adr + 1] = q[1] + fy * forward + ly * lateral;
    q[adr + 2] = 0.052;
    q[adr + 3] = 1; q[adr + 4] = 0; q[adr + 5] = 0; q[adr + 6] = 0;
    rl.data.qvel.fill(0);
    rl.mujoco.mj_forward(rl.model, rl.data);
    rl.render();
    return { ball: [q[adr], q[adr + 1], q[adr + 2]], yaw };
  }, { forward, lateral });
}

async function screenshot(page, name) {
  await page.evaluate(() => window.rl?.render?.());
  await page.screenshot({ path: join(ARTIFACTS, name), fullPage: true });
}

async function runWatchableDemo(browser) {
  const context = await browser.newContext({
    viewport: { width: 1280, height: 720 },
    recordVideo: { dir: join(ARTIFACTS, "video"), size: { width: 1280, height: 720 } },
  });
  const page = await context.newPage();
  hookPage(page, "demo");
  await page.goto(`${BASE}/?boot=1`, { waitUntil: "domcontentloaded", timeout: 120_000 });
  await waitForReady(page);
  await screenshot(page, "01-ready.png");

  await page.keyboard.down("w");
  await page.waitForTimeout(2600);
  await page.keyboard.up("w");
  await page.waitForTimeout(700);
  await screenshot(page, "02-after-forward.png");

  await page.keyboard.down("a");
  await page.waitForTimeout(1000);
  await page.keyboard.up("a");
  await page.waitForTimeout(650);
  await screenshot(page, "03-after-turn.png");

  await page.evaluate(() => window.rl.resetSim());
  await page.waitForTimeout(1100);
  await positionBall(page, 0.18, 0.025);
  await page.waitForTimeout(500);
  await page.evaluate(() => window.rl.triggerKick("left", "probe-video"));
  await page.waitForTimeout(1600);
  await screenshot(page, "04-left-kick.png");

  await page.evaluate(() => window.rl.resetSim());
  await page.waitForTimeout(1100);
  await positionBall(page, 0.18, -0.025);
  await page.waitForTimeout(500);
  await page.evaluate(() => window.rl.triggerKick("right", "probe-video"));
  await page.waitForTimeout(1600);
  await screenshot(page, "05-right-kick.png");

  await page.evaluate(() => {
    window.rl.resetSim();
    window.rl.debugPush(0, 0, 0, 0, 7.5, 0);
  });
  await page.waitForTimeout(7000);
  await screenshot(page, "06-after-recovery-window.png");

  const demoFinal = await state(page);
  await page.close();
  await context.close();
  return demoFinal;
}

async function deterministicKickTrial(page, foot, forward, lateral) {
  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); window.rl.render(); });
  await positionBall(page, forward, lateral);
  const before = await state(page);
  const result = await page.evaluate(async ({ foot }) => {
    const rl = window.rl;
    const adr = rl.ballQposAdr;
    const start = [Number(rl.data.qpos[adr]), Number(rl.data.qpos[adr + 1])];
    const samples = [];
    const launched = rl.triggerKick(foot, "probe");
    const steps = Number(rl.kickSteps) + 65;
    for (let i = 0; i < steps; i++) {
      await rl.step(1);
      if (i % 2 === 0) {
        samples.push({
          step: i + 1,
          ball: [Number(rl.data.qpos[adr]), Number(rl.data.qpos[adr + 1]), Number(rl.data.qpos[adr + 2])],
          mode: rl.mode,
          gravityZ: Number(rl.buildObs()[5]),
        });
      }
    }
    rl.render();
    return { launched, start, samples, finalMode: rl.mode };
  }, { foot });
  const end = await state(page);
  const maxDisplacement = maxOf(result.samples, (s) => distance2(s.ball, result.start));
  return {
    foot,
    forward,
    lateral,
    launched: result.launched,
    finalMode: result.finalMode,
    maxBallDisplacementM: round(maxDisplacement),
    finalBallDisplacementM: round(distance2(end.ball, before.ball)),
    minGravityZ: round(Math.min(...result.samples.map((s) => s.gravityZ))),
    maxBallHeightM: round(maxOf(result.samples, (s) => s.ball[2])),
    samples: result.samples.map((s) => ({
      step: s.step,
      ball: s.ball.map((n) => round(n)),
      mode: s.mode,
      gravityZ: round(s.gravityZ),
    })),
  };
}

async function runDeterministicMeasurements(browser) {
  const context = await browser.newContext({ viewport: { width: 1280, height: 720 } });
  const page = await context.newPage();
  hookPage(page, "measure");
  await page.goto(`${BASE}/?boot=1&manual=1`, { waitUntil: "domcontentloaded", timeout: 120_000 });
  await waitForReady(page);

  const boot = await state(page);
  const timingStart = performance.now();
  await page.evaluate(async () => { await window.rl.step(50); });
  const fiftyStepWallSeconds = (performance.now() - timingStart) / 1000;

  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); });
  const forwardStart = await state(page);
  await page.keyboard.down("w");
  await page.evaluate(async () => { await window.rl.step(250); });
  await page.keyboard.up("w");
  await page.evaluate(async () => { await window.rl.step(25); window.rl.render(); });
  const forwardEnd = await state(page);
  await screenshot(page, "07-forward-measurement.png");

  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); });
  const turnStart = await state(page);
  await page.keyboard.down("a");
  await page.evaluate(async () => { await window.rl.step(75); });
  await page.keyboard.up("a");
  await page.evaluate(async () => { await window.rl.step(25); window.rl.render(); });
  const turnEnd = await state(page);
  await screenshot(page, "08-turn-measurement.png");

  const placements = [
    { forward: 0.14, lateral: -0.04 }, { forward: 0.14, lateral: 0 }, { forward: 0.14, lateral: 0.04 },
    { forward: 0.18, lateral: -0.04 }, { forward: 0.18, lateral: 0 }, { forward: 0.18, lateral: 0.04 },
    { forward: 0.22, lateral: -0.04 }, { forward: 0.22, lateral: 0 }, { forward: 0.22, lateral: 0.04 },
  ];
  const kickTrials = [];
  for (const foot of ["left", "right"]) {
    for (const p of placements) kickTrials.push(await deterministicKickTrial(page, foot, p.forward, p.lateral));
  }
  const bestLeft = kickTrials.filter((t) => t.foot === "left").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];
  const bestRight = kickTrials.filter((t) => t.foot === "right").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];

  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); });
  const fallStart = await state(page);
  const recoveryTrace = await page.evaluate(async () => {
    const rl = window.rl;
    rl.debugPush(0, 0, 0, 0, 8.0, 0);
    const trace = [];
    let entered = false;
    let returned = false;
    for (let i = 0; i < 450; i++) {
      await rl.step(1);
      if (i % 2 === 0) {
        const q = rl.data.qpos;
        let gz = null;
        try { gz = Number(rl.buildObs()[5]); } catch {}
        trace.push({ step: i + 1, z: Number(q[2]), gravityZ: gz, recovery: rl.recovery, mode: rl.mode });
      }
      if (rl.recovery) entered = true;
      if (entered && !rl.recovery && rl.mode === "walk") { returned = true; break; }
    }
    rl.render();
    return { entered, returned, trace };
  });
  const fallEnd = await state(page);
  await screenshot(page, "09-fall-recovery-measurement.png");

  await page.close();
  await context.close();

  const forwardDistance = distance2(forwardStart.root, forwardEnd.root);
  const turnDelta = wrapRad(turnEnd.yaw - turnStart.yaw);
  const recoveryEnteredAt = recoveryTrace.trace.find((s) => s.recovery)?.step ?? null;
  const recoveryReturnedAt = recoveryTrace.returned ? recoveryTrace.trace.at(-1)?.step ?? null : null;
  return {
    boot,
    timing: {
      simulatedSteps: 50,
      simulatedSeconds: 1,
      wallSeconds: round(fiftyStepWallSeconds),
      realtimeFactor: round(1 / fiftyStepWallSeconds, 3),
    },
    walking: {
      commandedSimSeconds: 5,
      startXY: forwardStart.root.slice(0, 2).map((n) => round(n)),
      endXY: forwardEnd.root.slice(0, 2).map((n) => round(n)),
      displacementM: round(forwardDistance),
      meanGroundSpeedMps: round(forwardDistance / 5),
      finalGravityZ: round(forwardEnd.gravityZ),
    },
    turning: {
      commandedSimSeconds: 1.5,
      startYawDeg: round(rad2deg(turnStart.yaw), 2),
      endYawDeg: round(rad2deg(turnEnd.yaw), 2),
      yawChangeDeg: round(rad2deg(turnDelta), 2),
      finalGravityZ: round(turnEnd.gravityZ),
    },
    kicking: {
      trials: kickTrials,
      bestLeft,
      bestRight,
    },
    recovery: {
      pushAngularVelocityRadS: 8,
      enteredRecovery: recoveryTrace.entered,
      returnedToWalk: recoveryTrace.returned,
      enteredAtStep: recoveryEnteredAt,
      returnedAtStep: recoveryReturnedAt,
      simulatedSecondsUntilReturn: recoveryReturnedAt ? round(recoveryReturnedAt * 0.02) : null,
      finalMode: fallEnd.mode,
      finalRecovery: fallEnd.recovery,
      finalGravityZ: round(fallEnd.gravityZ),
      minimumTrunkHeightM: round(Math.min(...recoveryTrace.trace.map((s) => s.z))),
      maximumGravityZ: round(Math.max(...recoveryTrace.trace.map((s) => s.gravityZ ?? -1))),
      trace: recoveryTrace.trace.map((s) => ({
        step: s.step,
        z: round(s.z),
        gravityZ: s.gravityZ == null ? null : round(s.gravityZ),
        recovery: s.recovery,
        mode: s.mode,
      })),
      start: fallStart,
      end: fallEnd,
    },
  };
}

let browser;
try {
  await waitForHttp(BASE);
  browser = await chromium.launch({
    headless: true,
    args: [
      "--no-sandbox",
      "--disable-dev-shm-usage",
      "--use-gl=swiftshader",
      "--enable-webgl",
      "--ignore-gpu-blocklist",
    ],
  });

  const demoFinal = await runWatchableDemo(browser);
  const measurements = await runDeterministicMeasurements(browser);
  const hashes = await listEvidenceHashes();
  const upstreamCommit = execFileSync("git", ["rev-parse", "HEAD"], { cwd: "..", encoding: "utf8" }).trim();
  const result = {
    schemaVersion: 1,
    startedAt,
    finishedAt: new Date().toISOString(),
    source: {
      repository: "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
      commit: upstreamCommit,
      physics: "MuJoCo WASM",
      policyRuntime: "onnxruntime-web",
      controlHz: 50,
      deterministicPatch: "Only disables the concurrent wall-clock loop when ?manual=1; physics, model, policies, and controlStep are unchanged.",
      evidenceSha256: hashes,
    },
    browser: {
      version: browser.version(),
      headless: true,
      webglRequested: "SwiftShader",
    },
    demoFinal,
    measurements,
    verdictInputs: {
      canWalk: measurements.walking.displacementM > 0.1 && measurements.walking.finalGravityZ < -0.5,
      canTurn: Math.abs(measurements.turning.yawChangeDeg) > 20 && measurements.turning.finalGravityZ < -0.5,
      canKickLeft: measurements.kicking.bestLeft.maxBallDisplacementM > 0.05,
      canKickRight: measurements.kicking.bestRight.maxBallDisplacementM > 0.05,
      canRecover: measurements.recovery.enteredRecovery && measurements.recovery.returnedToWalk,
    },
  };
  await writeFile(join(ARTIFACTS, "results.json"), `${JSON.stringify(result, null, 2)}\n`);
  await writeFile(join(ARTIFACTS, "console.log"), logs.join("\n"));
  console.log("MICRODUCK_PROBE_RESULT", JSON.stringify(result.verdictInputs));
} catch (error) {
  logs.push(`[fatal] ${error.stack || error.message}`);
  await writeFile(join(ARTIFACTS, "console.log"), logs.join("\n"));
  await writeFile(join(ARTIFACTS, "failure.json"), `${JSON.stringify({ error: error.stack || error.message, startedAt }, null, 2)}\n`);
  console.error(error);
  process.exitCode = 1;
} finally {
  if (browser) await browser.close().catch(() => {});
  server.kill("SIGTERM");
}
