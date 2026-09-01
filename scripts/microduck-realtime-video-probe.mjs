import { chromium } from "playwright";
import { spawn, execFileSync } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import http from "node:http";
import { resolve, join } from "node:path";

const OUT = resolve("artifacts");
await mkdir(join(OUT, "video"), { recursive: true });
const logs = [];
const server = spawn(
  "npm",
  ["run", "preview", "--", "--host", "127.0.0.1", "--port", "4173", "--strictPort"],
  { stdio: ["ignore", "pipe", "pipe"] },
);
server.stdout.on("data", (b) => logs.push(`[server] ${b.toString()}`));
server.stderr.on("data", (b) => logs.push(`[server-error] ${b.toString()}`));

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const round = (n, d = 5) => Number(Number(n).toFixed(d));
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));

async function waitHttp() {
  const deadline = Date.now() + 60_000;
  while (Date.now() < deadline) {
    try {
      await new Promise((ok, fail) => {
        const req = http.get("http://127.0.0.1:4173", (res) => { res.resume(); ok(); });
        req.on("error", fail);
      });
      return;
    } catch {
      await sleep(250);
    }
  }
  throw new Error("preview server did not start");
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
    atMs: performance.now(),
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

async function state(page) { return page.evaluate(snapshot); }

async function reset(page) {
  await page.evaluate(() => window.rl.resetSim());
  await page.waitForTimeout(1200);
  await page.evaluate(() => window.rl.render());
}

async function positionBall(page, lateral) {
  return page.evaluate(({ lateral }) => {
    const rl = window.rl;
    const q = rl.data.qpos;
    const adr = rl.ballQposAdr;
    const w = Number(q[3]), x = Number(q[4]), y = Number(q[5]), z = Number(q[6]);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const fx = Math.cos(yaw), fy = Math.sin(yaw), lx = -fy, ly = fx;
    q[adr] = q[0] + fx * 0.17 + lx * lateral;
    q[adr + 1] = q[1] + fy * 0.17 + ly * lateral;
    q[adr + 2] = 0.052;
    q[adr + 3] = 1; q[adr + 4] = 0; q[adr + 5] = 0; q[adr + 6] = 0;
    rl.data.qvel.fill(0);
    rl.mujoco.mj_forward(rl.model, rl.data);
    rl.render();
    return [Number(q[adr]), Number(q[adr + 1]), Number(q[adr + 2])];
  }, { lateral });
}

let browser;
let context;
try {
  await waitHttp();
  const launch = {
    headless: true,
    args: ["--no-sandbox", "--disable-dev-shm-usage", "--use-gl=swiftshader", "--enable-webgl", "--ignore-gpu-blocklist"],
  };
  if (process.env.CHROME_BIN) launch.executablePath = process.env.CHROME_BIN;
  browser = await chromium.launch(launch);
  context = await browser.newContext({
    viewport: { width: 1280, height: 720 },
    recordVideo: { dir: join(OUT, "video"), size: { width: 1280, height: 720 } },
  });
  const page = await context.newPage();
  page.on("console", (m) => logs.push(`[console:${m.type()}] ${m.text()}`));
  page.on("pageerror", (e) => logs.push(`[pageerror] ${e.stack || e.message}`));
  page.on("requestfailed", (r) => logs.push(`[requestfailed] ${r.url()} ${r.failure()?.errorText || ""}`));

  await page.goto("http://127.0.0.1:4173/?boot=1", { waitUntil: "domcontentloaded", timeout: 120_000 });
  await page.waitForFunction(() => Boolean(window.rl?.model && window.rl?.data && window.rl?.ort), null, { timeout: 180_000 });
  await page.waitForFunction(() => !window.rl.inputLocked, null, { timeout: 90_000 });
  await page.evaluate(() => { window.rl.chaseCam = false; window.rl.render(); });
  await page.screenshot({ path: join(OUT, "01-ready.png") });
  const boot = await state(page);

  const walkStart = await state(page);
  await page.keyboard.down("w");
  await page.waitForTimeout(3000);
  await page.keyboard.up("w");
  await page.waitForTimeout(800);
  const walkEnd = await state(page);
  await page.screenshot({ path: join(OUT, "02-forward.png") });

  const turnStart = await state(page);
  await page.keyboard.down("a");
  await page.waitForTimeout(1500);
  await page.keyboard.up("a");
  await page.waitForTimeout(700);
  const turnEnd = await state(page);
  await page.screenshot({ path: join(OUT, "03-turn.png") });

  await reset(page);
  const leftBallStart = await positionBall(page, -0.035);
  const leftLaunched = await page.evaluate(() => window.rl.triggerKick("left", "realtime-video"));
  await page.waitForTimeout(1800);
  const leftEnd = await state(page);
  await page.screenshot({ path: join(OUT, "04-left-kick.png") });

  await reset(page);
  const rightBallStart = await positionBall(page, 0.035);
  const rightLaunched = await page.evaluate(() => window.rl.triggerKick("right", "realtime-video"));
  await page.waitForTimeout(1800);
  const rightEnd = await state(page);
  await page.screenshot({ path: join(OUT, "05-right-kick.png") });

  await reset(page);
  await page.evaluate(() => window.rl.debugPush(0, 0, 0, 0, 12, 0));
  const recoveryTrace = [];
  let enteredRecovery = false;
  let returnedToWalk = false;
  const recoveryT0 = Date.now();
  for (let i = 0; i < 42; i++) {
    await page.waitForTimeout(200);
    const s = await state(page);
    recoveryTrace.push(s);
    enteredRecovery ||= Boolean(s.recovery);
    if (enteredRecovery && !s.recovery && s.mode === "walk") {
      returnedToWalk = true;
      break;
    }
  }
  const recoveryWallSeconds = (Date.now() - recoveryT0) / 1000;
  await page.screenshot({ path: join(OUT, "06-recovery.png") });

  const walkDistance = distance(walkStart.root, walkEnd.root);
  const yawChangeDeg = wrap(turnEnd.yaw - turnStart.yaw) * 180 / Math.PI;
  const result = {
    schemaVersion: 1,
    finishedAt: new Date().toISOString(),
    source: {
      repository: "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
      commit: execFileSync("git", ["rev-parse", "HEAD"], { cwd: "..", encoding: "utf8" }).trim(),
      engine: "MuJoCo WASM",
      policyRuntime: "onnxruntime-web",
      controlLoop: "Official unmodified wall-clock loop",
    },
    browser: { version: browser.version(), executablePath: process.env.CHROME_BIN || "Playwright Chromium" },
    boot,
    walking: {
      wallCommandSeconds: 3,
      displacementM: round(walkDistance),
      observedMeanSpeedMps: round(walkDistance / 3),
      startXY: walkStart.root.slice(0, 2).map((n) => round(n)),
      endXY: walkEnd.root.slice(0, 2).map((n) => round(n)),
      finalGravityZ: round(walkEnd.gravityZ),
    },
    turning: {
      wallCommandSeconds: 1.5,
      yawChangeDeg: round(yawChangeDeg, 2),
      finalGravityZ: round(turnEnd.gravityZ),
    },
    kicking: {
      left: {
        launched: leftLaunched,
        ballDisplacementM: round(distance(leftBallStart, leftEnd.ball)),
        start: leftBallStart.map((n) => round(n)),
        end: leftEnd.ball.map((n) => round(n)),
        finalMode: leftEnd.mode,
        finalGravityZ: round(leftEnd.gravityZ),
      },
      right: {
        launched: rightLaunched,
        ballDisplacementM: round(distance(rightBallStart, rightEnd.ball)),
        start: rightBallStart.map((n) => round(n)),
        end: rightEnd.ball.map((n) => round(n)),
        finalMode: rightEnd.mode,
        finalGravityZ: round(rightEnd.gravityZ),
      },
    },
    recovery: {
      pushAngularVelocityRadS: 12,
      enteredRecovery,
      returnedToWalk,
      observedWallSeconds: round(recoveryWallSeconds),
      maxGravityZ: round(Math.max(...recoveryTrace.map((x) => x.gravityZ ?? -1))),
      minTrunkHeightM: round(Math.min(...recoveryTrace.map((x) => x.root[2]))),
      final: recoveryTrace.at(-1),
      trace: recoveryTrace,
    },
  };
  result.verdict = {
    movedUnderForwardCommand: result.walking.displacementM > 0.03,
    turnedUnderLeftCommand: Math.abs(result.turning.yawChangeDeg) > 5,
    leftKickMovedBall: result.kicking.left.ballDisplacementM > 0.03,
    rightKickMovedBall: result.kicking.right.ballDisplacementM > 0.03,
    recoveredAfterFall: enteredRecovery && returnedToWalk,
  };
  await writeFile(join(OUT, "results.json"), `${JSON.stringify(result, null, 2)}\n`);
  console.log("REALTIME_RESULT", JSON.stringify(result.verdict));

  await page.waitForTimeout(600);
  await page.close();
  await context.close();
  context = null;
  await writeFile(join(OUT, "console.log"), logs.join("\n"));
} catch (error) {
  logs.push(`[fatal] ${error.stack || error.message}`);
  await writeFile(join(OUT, "failure.json"), `${JSON.stringify({ error: error.stack || error.message }, null, 2)}\n`);
  await writeFile(join(OUT, "console.log"), logs.join("\n"));
  console.error(error);
  process.exitCode = 1;
} finally {
  await context?.close().catch(() => {});
  await browser?.close().catch(() => {});
  server.kill("SIGTERM");
}
