import { chromium } from "playwright";
import { spawn, execFileSync } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import http from "node:http";
import { resolve, join } from "node:path";

const OUT = resolve("artifacts");
await mkdir(OUT, { recursive: true });
await mkdir(join(OUT, "video"), { recursive: true });
const log = [];
const server = spawn(
  "npm",
  ["run", "preview", "--", "--host", "127.0.0.1", "--port", "4173", "--strictPort"],
  { stdio: ["ignore", "pipe", "pipe"] },
);
server.stdout.on("data", (b) => log.push(`[server] ${b.toString()}`));
server.stderr.on("data", (b) => log.push(`[server-error] ${b.toString()}`));

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const round = (n, d = 5) => Number(Number(n).toFixed(d));
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));

async function waitHttp() {
  const deadline = Date.now() + 60_000;
  while (Date.now() < deadline) {
    try {
      await new Promise((ok, fail) => {
        const req = http.get("http://127.0.0.1:4173", (res) => {
          res.resume();
          ok();
        });
        req.on("error", fail);
      });
      return;
    } catch {
      await sleep(250);
    }
  }
  throw new Error("Vite preview did not start");
}

function snapshot() {
  const rl = window.rl;
  const q = rl.data.qpos;
  const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  let gravityZ = null;
  try { gravityZ = Number(rl.buildObs()[5]); } catch {}
  const adr = rl.ballQposAdr;
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

async function getState(page) {
  return page.evaluate(snapshot);
}

async function settle(page, steps = 20) {
  await page.evaluate(async ({ steps }) => {
    window.rl.resetSim();
    await window.rl.step(steps);
    window.rl.render();
  }, { steps });
}

async function kickTrial(page, foot, lateral) {
  await settle(page, 12);
  const start = await page.evaluate(({ lateral }) => {
    const rl = window.rl;
    const q = rl.data.qpos;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const fx = Math.cos(yaw), fy = Math.sin(yaw), lx = -fy, ly = fx;
    const forward = 0.17;
    q[adr] = q[0] + fx * forward + lx * lateral;
    q[adr + 1] = q[1] + fy * forward + ly * lateral;
    q[adr + 2] = 0.052;
    q[adr + 3] = 1; q[adr + 4] = 0; q[adr + 5] = 0; q[adr + 6] = 0;
    rl.data.qvel.fill(0);
    rl.mujoco.mj_forward(rl.model, rl.data);
    rl.render();
    return [Number(q[adr]), Number(q[adr + 1])];
  }, { lateral });

  const measured = await page.evaluate(async ({ foot, start }) => {
    const rl = window.rl;
    const adr = rl.ballQposAdr;
    const launched = rl.triggerKick(foot, "ultra-probe");
    let maxDistance = 0;
    let maxHeight = 0;
    let minGravityZ = 1;
    for (let i = 0; i < Number(rl.kickSteps) + 32; i++) {
      await rl.step(1);
      const bx = Number(rl.data.qpos[adr]);
      const by = Number(rl.data.qpos[adr + 1]);
      maxDistance = Math.max(maxDistance, Math.hypot(bx - start[0], by - start[1]));
      maxHeight = Math.max(maxHeight, Number(rl.data.qpos[adr + 2]));
      try { minGravityZ = Math.min(minGravityZ, Number(rl.buildObs()[5])); } catch {}
    }
    rl.render();
    return {
      launched,
      maxDistance,
      maxHeight,
      minGravityZ,
      final: [Number(rl.data.qpos[adr]), Number(rl.data.qpos[adr + 1])],
      finalMode: rl.mode,
    };
  }, { foot, start });

  return {
    foot,
    forwardM: 0.17,
    lateralM: lateral,
    launched: measured.launched,
    maxBallDisplacementM: round(measured.maxDistance),
    finalBallDisplacementM: round(distance(measured.final, start)),
    maxBallHeightM: round(measured.maxHeight),
    minGravityZ: round(measured.minGravityZ),
    finalMode: measured.finalMode,
  };
}

async function recoveryTrial(page, push) {
  await settle(page, 20);
  return page.evaluate(async ({ push }) => {
    const rl = window.rl;
    rl.debugPush(0, 0, 0, 0, push, 0);
    let entered = false;
    let returned = false;
    let maxGravityZ = -1;
    let minHeight = 99;
    let steps = 0;
    const transitions = [];
    let last = `${rl.recovery}|${rl.mode}`;
    for (let i = 0; i < 400; i++) {
      await rl.step(1);
      steps = i + 1;
      entered ||= Boolean(rl.recovery);
      minHeight = Math.min(minHeight, Number(rl.data.qpos[2]));
      try { maxGravityZ = Math.max(maxGravityZ, Number(rl.buildObs()[5])); } catch {}
      const current = `${rl.recovery}|${rl.mode}`;
      if (current !== last) {
        transitions.push({ step: i + 1, recovery: rl.recovery, mode: rl.mode });
        last = current;
      }
      if (entered && !rl.recovery && rl.mode === "walk") {
        returned = true;
        break;
      }
    }
    rl.render();
    return {
      push,
      entered,
      returned,
      steps,
      simulatedSeconds: steps * 0.02,
      maxGravityZ,
      minHeight,
      finalMode: rl.mode,
      finalRecovery: rl.recovery,
      transitions,
    };
  }, { push });
}

async function makeVisibleDemo(page) {
  await settle(page, 12);
  await page.keyboard.down("w");
  await page.evaluate(async () => {
    for (let i = 0; i < 65; i++) {
      await window.rl.step(1);
      if (i % 2 === 0) window.rl.render();
      await new Promise((r) => setTimeout(r, 18));
    }
  });
  await page.keyboard.up("w");

  await page.keyboard.down("a");
  await page.evaluate(async () => {
    for (let i = 0; i < 35; i++) {
      await window.rl.step(1);
      window.rl.render();
      await new Promise((r) => setTimeout(r, 18));
    }
  });
  await page.keyboard.up("a");

  await settle(page, 12);
  await page.evaluate(() => {
    const rl = window.rl;
    const q = rl.data.qpos;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    q[adr] = q[0] + Math.cos(yaw) * 0.17;
    q[adr + 1] = q[1] + Math.sin(yaw) * 0.17;
    q[adr + 2] = 0.052;
    q[adr + 3] = 1; q[adr + 4] = 0; q[adr + 5] = 0; q[adr + 6] = 0;
    rl.data.qvel.fill(0);
    rl.mujoco.mj_forward(rl.model, rl.data);
    rl.triggerKick("left", "ultra-video");
    rl.render();
  });
  await page.evaluate(async () => {
    for (let i = 0; i < 50; i++) {
      await window.rl.step(1);
      window.rl.render();
      await new Promise((r) => setTimeout(r, 20));
    }
  });
}

let browser;
let context;
try {
  await waitHttp();
  const launchOptions = {
    headless: true,
    args: ["--no-sandbox", "--disable-dev-shm-usage", "--use-gl=swiftshader", "--enable-webgl", "--ignore-gpu-blocklist"],
  };
  if (process.env.CHROME_BIN) launchOptions.executablePath = process.env.CHROME_BIN;
  browser = await chromium.launch(launchOptions);
  context = await browser.newContext({
    viewport: { width: 1280, height: 720 },
    recordVideo: { dir: join(OUT, "video"), size: { width: 1280, height: 720 } },
  });
  const page = await context.newPage();
  page.on("console", (m) => log.push(`[console:${m.type()}] ${m.text()}`));
  page.on("pageerror", (e) => log.push(`[pageerror] ${e.stack || e.message}`));
  page.on("requestfailed", (r) => log.push(`[requestfailed] ${r.url()} ${r.failure()?.errorText || ""}`));

  await page.goto("http://127.0.0.1:4173/?boot=1&manual=1", { waitUntil: "domcontentloaded", timeout: 120_000 });
  await page.waitForFunction(() => Boolean(window.rl?.model && window.rl?.data && window.rl?.ort), null, { timeout: 180_000 });
  try {
    await page.waitForFunction(() => !window.rl.inputLocked, null, { timeout: 75_000 });
  } catch {
    await page.evaluate(() => window.rl.entrance.start());
    await page.waitForFunction(() => !window.rl.inputLocked, null, { timeout: 30_000 });
  }
  await page.evaluate(() => { window.rl.chaseCam = false; window.rl.render(); });
  await page.screenshot({ path: join(OUT, "01-ready.png") });
  const boot = await getState(page);

  await settle(page, 20);
  const walkStart = await getState(page);
  await page.keyboard.down("w");
  await page.evaluate(async () => { await window.rl.step(100); });
  await page.keyboard.up("w");
  await page.evaluate(async () => { await window.rl.step(15); window.rl.render(); });
  const walkEnd = await getState(page);
  await page.screenshot({ path: join(OUT, "02-walk.png") });

  await settle(page, 20);
  const turnCtart = await getState(page);
  await page.keyboard.down("a");
  await page.evaluate(async () => { await window.rl.step(50); });
  await page.keyboard.up("a");
  await page.evaluate(async () => { await window.rl.step(15); window.rl.render(); });
  const turnEnd = await getState(page);
  await page.screenshot({ path: join(OUT, "03-turn.png") });

  const kickTrials = [];
  for (const foot of ["left", "right"]) {
    for (const lateral of [-0.045, 0, 0.045]) {
      console.log(`ULTRA_KICK ${foot} ${lateral}`);
      kickTrials.push(await kickTrial(page, foot, lateral));
    }
  }
  const bestLeft = kickTrials.filter((x) => x.foot === "left").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];
  const bestRight = kickTrials.filter((x) => x.foot === "right").sort((a, b) => b.maxBallDisplacementM - a.maxBallDisplacementM)[0];
  await page.screenshot({ path: join(OUT, "04-kick.png") });

  let recovery = await recoveryTrial(page, 12);
  if (!recovery.entered) recovery = await recoveryTrial(page, 18);
  await page.screenshot({ path: join(OUT, "05-recovery.png") });

  const walkingDistance = distance(walkStart.root, walkEnd.root);
  const turningDeg = wrap(turnEnd.yaw - turnStart.yaw) * 180 / Math.PI;
  const result = {
    schemaVersion: 1,
    finishedAt: new Date().toISOString(),
    source: {
      repository: "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
      commit: execFileSync("git", ["rev-parse", "HEAD"], { cwd: "..", encoding: "utf8" }).trim(),
      engine: "MuJoCo WASM",
      policyRuntime: "onnxruntime-web",
      controlHz: 50,
      deterministicPatch: "The concurrent timer is disabled only for ?manual=1; the original controlStep, MuJoCo model, and ONNX policies are unchanged.",
    },
    browser: { version: browser.version(), executablePath: process.env.CHROME_BIN || "Playwright Chromium" },
    boot,
    walking: {
      commandSeconds: 2,
      displacementM: round(walkingDistance),
      meanSpeedMps: round(walkingDistance / 2),
      startXY: walkStart.root.slice(0, 2).map((n) => round(n)),
      endXY: walkEnd.root.slice(0, 2).map((n) => round(n)),
      finalGravityZ: round(walkEnd.gravityZ),
    },
    turning: {
      commandSeconds: 1,
      yawChangeDeg: round(turningDeg, 2),
      startYawDeg: round(turnStart.yaw * 180 / Math.PI, 2),
      endYawDeg: round(turnEnd.yaw * 180 / Math.PI, 2),
      finalGravityZ: round(turnEnd.gravityZ),
    },
    kicking: { trials: kickTrials, bestLeft, bestRight },
    recovery: {
      pushAngularVelocityRadS: recovery.push,
      enteredRecovery: recovery.entered,
      returnedToWalk: recovery.returned,
      simulatedSeconds: round(recovery.simulatedSeconds),
      maximumGravityZ: round(recovery.maxGravityZ),
      minimumTrunkHeightM: round(recovery.minHeight),
      finalMode: recovery.finalMode,
      finalRecovery: recovery.finalRecovery,
      transitions: recovery.transitions,
    },
  };
  result.verdict = {
    canWalk: result.walking.displacementM > 0.05 && result.walking.finalGravityZ < -0.5,
    canTurn: Math.abs(result.turning.yawChangeDeg) > 10 && result.turning.finalGravityZ < -0.5,
    canKickLeft: result.kicking.bestLeft.maxBallDisplacementM > 0.04,
    canKickRight: result.kicking.bestRight.maxBallDisplacementM > 0.04,
    canRecover: result.recovery.enteredRecovery && result.recovery.returnedToWalk,
  };
  await writeFile(join(OUT, "results.json"), `${JSON.stringify(result, null, 2)}\n`);
  console.log("ULTRA_RESULT", JSON.stringify(result.verdict));

  await makeVisibleDemo(page);
  await page.screenshot({ path: join(OUT, "06-demo-end.png") });
  await page.close();
  await context.close();
  context = null;
  await writeFile(join(OUT, "console.log"), log.join("\n"));
} catch (error) {
  log.push(`[fatal] ${error.stack || error.message}`);
  await writeFile(join(OUT, "failure.json"), `${JSON.stringify({ error: error.stack || error.message }, null, 2)}\n`);
  await writeFile(join(OUT, "console.log"), log.join("\n"));
  console.error(error);
  process.exitCode = 1;
} finally {
  await context?.close().catch(() => {});
  await browser?.close().catch(() => {});
  server.kill("SIGTERM");
}
