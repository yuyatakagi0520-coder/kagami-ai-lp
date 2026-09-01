import { chromium } from "playwright";
import { spawn, execFileSync } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import http from "node:http";
import { resolve, join } from "node:path";

const artifacts = resolve("artifacts");
await mkdir(artifacts, { recursive: true });
const logs = [];
const server = spawn("npm", ["run", "preview", "--", "--host", "127.0.0.1", "--port", "4173", "--strictPort"], { stdio: ["ignore", "pipe", "pipe"] });
server.stdout.on("data", (b) => logs.push(`[server] ${b}`));
server.stderr.on("data", (b) => logs.push(`[server-error] ${b}`));

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const round = (n, d = 5) => Number(Number(n).toFixed(d));
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const wrap = (v) => Math.atan2(Math.sin(v), Math.cos(v));

async function waitHttp() {
  const until = Date.now() + 60_000;
  for (;;) {
    try {
      await new Promise((ok, bad) => {
        const req = http.get("http://127.0.0.1:4173", (res) => { res.resume(); ok(); });
        req.on("error", bad);
      });
      return;
    } catch (e) {
      if (Date.now() > until) throw e;
      await sleep(250);
    }
  }
}

function pageState() {
  const rl = window.rl;
  const q = rl.data.qpos;
  const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const adr = rl.ballQposAdr;
  let gravityZ = null;
  try { gravityZ = Number(rl.buildObs()[5]); } catch {}
  return {
    mode: rl.mode,
    loco: rl.loco,
    inputLocked: rl.inputLocked,
    recovery: rl.recovery,
    root: Array.from(q.slice(0, 7), Number),
    yaw,
    gravityZ,
    ball: adr == null ? null : Array.from(q.slice(adr, adr + 3), Number),
  };
}

async function state(page) { return page.evaluate(pageState); }
async function shot(page, foot, forward, lateral) {
  await page.evaluate(async () => { window.rl.resetSim(); await window.rl.step(35); });
  const before = await page.evaluate(({ forward, lateral }) => {
    const rl = window.rl;
    const q = rl.data.qpos;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const fx = Math.cos(yaw), fy = Math.sin(yaw), lx = -fy, ly = fx;
    q[adr] = q[0] + fx * forward + lx * lateral;
    q[adr + 1] = q[1] + fy * forward + ly * lateral;
    q[adr + 2] = 0.052;
    q[adr + 3] = 1; q[adr + 4] = 0; q[adr + 5] = 0; q[adr + 6] = 0;
    rl.data.qvel.fill(0);
    rl.mujoco.mj_forward(rl.model, rl.data);
    return [Number(q[adr]), Number(q[adr + 1])];
  }, { forward, lateral });
  const run = await page.evaluate(async ({ foot, before }) => {
    const rl = window.rl;
    const adr = rl.ballQposAdr;
    const launched = rl.triggerKick(foot, "fast-probe");
    let max = 0;
    let maxZ = 0;
    let minG = 1;
    for (let i = 0; i < Number(rl.kickSteps) + 65; i++) {
      await rl.step(1);
      const x = Number(rl.data.qpos[adr]), y = Number(rl.data.qpos[adr + 1]);
      max = Math.max(max, Math.hypot(x - before[0], y - before[1]));
      maxZ = Math.max(maxZ, Number(rl.data.qpos[adr + 2]));
      try { minG = Math.min(minG, Number(rl.buildObs()[5])); } catch {}
    }
    rl.render();
    return { launched, max, maxZ, minG, mode: rl.mode, final: [Number(rl.data.qpos[adr]), Number(rl.data.qpos[adr + 1])] };
  }, { foot, before });
  return {
    foot, forward, lateral,
    launched: run.launched,
    maxBallDisplacementM: round(run.max),
    finalBallDisplacementM: round(dist(run.final, before)),
    maxBallHeightM: round(run.maxZ),
    minGravityZ: round(run.minG),
    finalMode: run.mode,
  };
}

let browser;
try {
  await waitHttp();
  browser = await chromium.launch({ headless: true, args: ["--no-sandbox", "--disable-dev-shm-usage", "--use-gl=swiftshader", "--enable-webgl", "--ignore-gpu-blocklist"] });
  const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
  page.on("console", (m) => logs.push(`[console:${m.type()}] ${m.text()}`));
  page.on("pageerror", (e) => logs.push(`[pageerror] ${e.stack || e.message}`));
  page.on("requestfailed", (r) => logs.push(`[requestfailed] ${r.url()} ${r.failure()?.errorText || ""}`));
  await page.goto("http://127.0.0.1:4173/?boot=1&manual=1", { waitUntil: "domcontentloaded", timeout: 120_000 });
  await page.waitForFunction(() => Boolean(window.rl?.model && window.rl?.data && window.rl?.ort), null, { timeout: 180_000 });
  try {
    await page.waitForFunction(() => !window.rl.inputLocked, null, { timeout: 75_000 });
  } catch {
    await page.evaluate(() => window.rl.entrance.start());
    await page.waitForFunction(() => !window.rl.inputLocked, null, { timeout: 30_000 });
  }
  await page.evaluate(async () => { window.rl.chaseCam = false; await window.rl.step(40); window.rl.render(); });
  await page.screenshot({ path: join(artifacts, "01-ready.png") });
  const boot = await state(page);

  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); });
  const walkStart = await state(page);
  await page.keyboard.down("w");
  await page.evaluate(async () => { await window.rl.step(150); });
  await page.keyboard.up("w");
  await page.evaluate(async () => { await window.rl.step(25); window.rl.render(); });
  const walkEnd = await state(page);
  await page.screenshot({ path: join(artifacts, "02-after-3s-forward.png") });

  await page.evaluate(() => window.rl.resetSim());
  await page.evaluate(async () => { await window.rl.step(35); });
  const turnStart = await state(page);
  await page.keyboard.down("a");
  await page.evaluate(async () => { await window.rl.step(50); });
  await page.keyboard.up("a");
  await page.evaluate(async () => { await window.rl.step(25); window.rl.render(); });
  const turnEnd = await state(page);
  await page.screenshot({ path: join(artifacts, "03-after-1s-turn.png") });

  const trials = [];
  for (const foot of ["left", "right"]) {
    for (const forward of [0.13, 0.17, 0.21]) {
      for (const lateral of [-0.045, 0, 0.045]) {
        console.log(`KICK_TRIAL ${foot} ${forward} ${lateral}`);
        trials.push(await shot(page, foot, forward, lateral));
      }
    }
  }
  const bestLeft = trials.filter((x) => x.foot === "left").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];
  const bestRight = trials.filter((x) => x.foot === "right").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];
  await page.screenshot({ path: join(artifacts, "04-after-kick-grid.png") });

  let recovery = null;
  for (const push of [8, 12, 16]) {
    const attempt = await page.evaluate(async ({ push }) => {
      const rl = window.rl;
      rl.resetSim();
      await rl.step(35);
      rl.debugPush(0, 0, 0, 0, push, 0);
      let entered = false;
      let returned = false;
      let maxGravityZ = -1;
      let minZ = 99;
      let steps = 0;
      for (let i = 0; i < 450; i++) {
        await rl.step(1);
        steps = i + 1;
        entered ||= Boolean(rl.recovery);
        minZ = Math.min(minZ, Number(rl.data.qpos[2]));
        try { maxGravityZ = Math.max(maxGravityZ, Number(rl.buildObs()[5])); } catch {}
        if (entered && !rl.recovery && rl.mode === "walk") { returned = true; break; }
      }
      rl.render();
      return { push, entered, returned, steps, maxGravityZ, minZ, mode: rl.mode, recovery: rl.recovery };
    }, { push });
    recovery = attempt;
    if (attempt.entered) break;
  }
  await page.screenshot({ path: join(artifacts, "05-after-recovery.png") });

  const walkDistance = dist(walkStart.root, walkEnd.root);
  const yawChange = wrap(turnEnd.yaw - turnStart.yaw) * 180 / Math.PI;
  const result = {
    schemaVersion: 1,
    finishedAt: new Date().toISOString(),
    source: {
      repository: "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
      commit: execFileSync("git", ["rev-parse", "HEAD"], { cwd: "..", encoding: "utf8" }).trim(),
      engine: "MuJoCo WASM",
      policyRuntime: "onnxruntime-web",
      controlHz: 50,
      deterministicPatch: "Only disables the concurrent timer with ?manual=1. window.rl.step executes the original controlStep unchanged.",
    },
    browser: { version: browser.version(), rendererRequested: "SwiftShader WebGL" },
    boot,
    walking: {
      commandSeconds: 3,
      displacementM: round(walkDistance),
      meanSpeedMps: round(walkDistance / 3),
      startXY: walkStart.root.slice(0, 2).map((n) => round(n)),
      endXY: walkEnd.root.slice(0, 2).map((n) => round(n)),
      finalGravityZ: round(walkEnd.gravityZ),
    },
    turning: {
      commandSeconds: 1,
      yawChangeDeg: round(yawChange, 2),
      startYawDeg: round(turnStart.yaw * 180 / Math.PI, 2),
      endYawDeg: round(turnEnd.yaw * 180 / Math.PI, 2),
      finalGravityZ: round(turnEnd.gravityZ),
    },
    kicking: { trials, bestLeft, bestRight },
    recovery: {
      pushAngularVelocityRadS: recovery.push,
      enteredRecovery: recovery.entered,
      returnedToWalk: recovery.returned,
      simulatedSeconds: round(recovery.steps * 0.02),
      maximumGravityZ: round(recovery.maxGravityZ),
      minimumTrunkHeightM: round(recovery.minZ),
      finalMode: recovery.mode,
      finalRecovery: recovery.recovery,
    },
  };
  result.verdict = {
    canWalk: result.walking.displacementM > 0.08 && result.walking.finalGravityZ < -0.5,
    canTurn: Math.abs(result.turning.yawChangeDeg) > 15 && result.turning.finalGravityZ < -0.5,
    canKickLeft: result.kicking.bestLeft.maxBallDisplacementM > 0.05,
    canKickRight: result.kicking.bestRight.maxBallDisplacementM > 0.05,
    canRecover: result.recovery.enteredRecovery && result.recovery.returnedToWalk,
  };
  await writeFile(join(artifacts, "results.json"), `${JSON.stringify(result, null, 2)}\n`);
  await writeFile(join(artifacts, "console.log"), logs.join("\n"));
  console.log("FAST_RESULT", JSON.stringify(result.verdict));
  await page.close();
} catch (e) {
  logs.push(`[fatal] ${e.stack || e.message}`);
  await writeFile(join(artifacts, "failure.json"), `${JSON.stringify({ error: e.stack || e.message }, null, 2)}\n`);
  await writeFile(join(artifacts, "console.log"), logs.join("\n"));
  console.error(e);
  process.exitCode = 1;
} finally {
  await browser?.close().catch(() => {});
  server.kill("SIGTERM");
}
