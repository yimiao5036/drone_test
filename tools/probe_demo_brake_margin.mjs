// 演示页「刹车包络」回归探针（Node >= 18，不需要浏览器）。
//
// 做法：读 docs/示意图/追踪梯形曲线-交互演示-2026-11-05.html，抽 <script>，
// 用最小 DOM 桩在 vm 里跑，检查「到点球不被穿」这条硬约束：
//   1) 修复后的默认（真值源、J=0）：最小距离应 ≥ 2.95 m（停在 3 m 到点球上）；
//   2) 打开 J=8：复现短距离穿球（已知的"包络 + 二阶限幅"两层结构问题，
//      结构修复待定，见 docs/接近段速度律-数学分析-2026-11-09.md §5.4）；
//   3) 默认页面（速度位 + 200 ms 时延 + σ=0）：收到点直接算，最小距离应 ≥ 2.95 m；
//   4) 0.8 m 噪声链路（可选工况）：报告实际最小距离，只要求不穿过目标；
//   5) 判词/相位/加速度图口径：起步不误报"追不上"、余量 0 正确报、人工固定点预测点不动、
//      加速度图有正有负、J=0 时指令加速度不恒 0（§5.9）；
//   6) 播放循环跑到追上为止：120 m / 接近速率 1 m/s 的工况要 >60 s 才到点，不能被 60 s 截断。
//
// 注意：DOM 桩里所有 input.value 都是 '0'，页面加载时 bindRange 会把 P 改成 0；
// 下面每个场景都显式 Object.assign 了相关参数，不要依赖页面默认值。
//
//   node tools/probe_demo_brake_margin.mjs

import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';

const ROOT = path.resolve(import.meta.dirname, '..');
const DEMO = path.join(ROOT, 'docs', '示意图', '追踪梯形曲线-交互演示-2026-11-05.html');
const source = fs.readFileSync(DEMO, 'utf8').match(/<script>([\s\S]*?)<\/script>/);
if (!source) throw new Error('演示页里找不到 <script>');

function fakeEl() {
  return {
    value: '0', textContent: '', innerHTML: '', hidden: false, style: {}, dataset: {},
    classList: { toggle() {}, add() {}, remove() {}, contains() { return false; } },
    addEventListener() {}, querySelectorAll() { return []; }, querySelector() { return null; },
    getBoundingClientRect() { return { width: 900, height: 400 }; },
    getContext() { return null; }, closest() { return null; },
  };
}

function makeContext() {
  const cache = new Map();
  const document = {
    getElementById(id) {
      if (!cache.has(id)) cache.set(id, fakeEl());
      return cache.get(id);
    },
    querySelector() { return fakeEl(); },
    addEventListener() {},
  };
  const sandbox = {
    document,
    window: { addEventListener() {}, devicePixelRatio: 1 },
    performance: { now: () => 0 },
    requestAnimationFrame: () => 0,
    console,
  };
  sandbox.globalThis = sandbox;
  const context = vm.createContext(sandbox);
  vm.runInContext(source[1], context, { filename: 'demo.js' });
  return context;
}

const RUN = `(function(cfg){
  Object.assign(P, cfg); resetSims(); P.paused = true;
  let t = 0, minD = 1e9, vAt3 = null;
  for (let i = 0; i < 40000; i++) {
    stepSim(A, DT); t += DT;
    const d = A.d, v = Math.hypot(A.vel.x, A.vel.y);
    minD = Math.min(minD, d);
    if (vAt3 === null && d <= 3.0) vAt3 = v;
    if (A.arrived) break;
  }
  return {arrived: !!A.arrived, t: +t.toFixed(2), minD: +minD.toFixed(2),
          vAt3: vAt3 === null ? null : +vAt3.toFixed(2), vpeak: +(A.vpeak || 0).toFixed(2)};
})`;

const BASE = {
  law: 'brake', policy: 'fast', end: 'match', accel: 3, decel: 3, slew: 3,
  vcap: 10, heading: 90, turn: 'straight',
};
const CASES = {
  '40 m 静止': { vt: 0, startD: 40 },
  '40 m 后退 2 m/s': { vt: 2, startD: 40 },
  '10 m 静止': { vt: 0, startD: 10 },
  '10 m 后退 2 m/s': { vt: 2, startD: 10 },
};

function runAll(label, cfg, context) {
  console.log(`\n${label}`);
  let worst = Infinity;
  for (const [name, c] of Object.entries(CASES)) {
    const r = new vm.Script(RUN).runInContext(context)({ ...BASE, ...cfg, ...c });
    worst = Math.min(worst, r.minD);
    console.log(`  ${name.padEnd(16)} 到点 ${String(r.t).padStart(6)} s  ` +
                `最小距离 ${r.minD.toFixed(2)} m  ` +
                `穿 3 m 速度 ${r.vAt3 === null ? '未穿' : r.vAt3.toFixed(2) + ' m/s'}  ` +
                `峰值 ${r.vpeak.toFixed(2)}`);
  }
  return worst;
}

const ctx = makeContext();
const fixed = runAll('① 修复后默认（真值、J=0）', { src: 'truth', lead: 0, jerk: 0 }, ctx);
const jerk = runAll('② 已知残留：J=8（短距离会冲过包络）', { src: 'truth', lead: 0, jerk: 8 }, ctx);
// DOM 桩把所有 input.value 当 '0'，所以这里显式填页面默认的噪声/频率/时延/速度位误差。
const published = runAll('③ 页面默认（速度位、200 ms 时延、σ=σ_v=0）',
  { src: 'vmsg', lead: 0, jerk: 0, latency: 200, noise: 0, measHz: 5, vsig: 0, sigmaA: 8 }, ctx);
const noisy = runAll('④ 噪声链路（σ=0.8 m、σ_v=0.1 m/s）',
  { src: 'vmsg', lead: 0, jerk: 0, latency: 200, noise: 0.8, measHz: 5, vsig: 0.1, sigmaA: 8 }, ctx);

let ok = true;
function check(cond, good, bad) {
  console.log((cond ? '✓ ' : '✗ ') + (cond ? good : bad));
  ok = ok && cond;
}
console.log('');
check(fixed >= 2.95, '修复后默认停在到点球上（最小距离 ≥ 2.95 m）',
      `修复后默认仍穿过：最小距离 ${fixed.toFixed(2)} m`);
check(jerk < 2.0, 'J=8 仍会穿球（已知残留，记录在案）',
      'J=8 未复现穿球，说明结构或参数已变，需复核分析文档 §5.4');
check(published >= 2.95, '默认页面不加噪声：停在 3 m 球上（最小 ≥ 2.95 m）',
      `默认页面仍冲进球内：${published.toFixed(2)} m`);
// 0.8 m 位置噪声是可选的"链路误差"工况：落点是噪声点上的 3 m 球，真实最小距离会低约 1 m。
// 这不是包络问题，只要求不穿过目标。
check(noisy >= 1.0, '0.8 m 噪声链路不穿过目标（最小距离 ≥ 1 m）',
      `噪声链路穿得太深：${noisy.toFixed(2)} m`);

// 对照组（无余量）必须追不上——页面图例的核心对照（制动曲线律下也要成立）。
const baseline = vm.runInContext(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', jerk:0, vcap:10, vt:2, heading:90,
    turn:'straight', startD:40, end:'match', accel:3, decel:3, slew:3, lead:0, noise:0, vsig:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 4000; i++) { stepSim(A, DT); stepSim(B, DT); }
  return {bArrived: !!B.arrived, bD: +B.d.toFixed(1)};
})()`, ctx);
check(!baseline.bArrived && baseline.bD > 35,
      `对照组（无余量）追不上，停在 ${baseline.bD} m`,
      `对照组也追上了（d=${baseline.bD}），图例的核心对照失效`);

// 判词/读数条是最容易在改口径时抛错的地方（TDZ、未定义变量），这里跑一次 updateHud。
const hudOk = vm.runInContext(`(function(){
  resetSims(); P.paused = true;
  for (let i = 0; i < 600; i++) stepSim(A, DT);
  updateHud();
  return document.getElementById('verdict').innerHTML.length > 0;
})()`, ctx);
check(hudOk, 'updateHud() 可运行（判词非空）', 'updateHud() 没有产出判词');

// 判词口径：起步不能误报"追不上"；余量 0 的保守策略必须报"追不上"。
const verdicts = vm.runInContext(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', jerk:0, vcap:10, vt:0,
    heading:90, turn:'straight', startD:40, end:'match', accel:3, decel:3, slew:3});
  resetSims(); P.paused = true;
  for (let i = 0; i < 25; i++) stepSim(A, DT);
  updateHud();
  const early = document.getElementById('verdict').innerHTML;
  Object.assign(P, {law:'env', profile:'trap', policy:'safe', margin:0, vt:2, vcap:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 3000; i++) stepSim(A, DT);
  updateHud();
  return {early, stuck: document.getElementById('verdict').innerHTML};
})()`, ctx);
check(!verdicts.early.includes('追不上'), '起步 0.5 s 不误报"追不上"', '起步 0.5 s 就误报"追不上"');
check(verdicts.stuck.includes('追不上'), '余量 0 的保守策略正确报"追不上"', '余量 0 未报"追不上"');

// 人工固定点（「静止」预设 = σ=0、σ_v=0）：预测点必须不动，飞机停在 3 m 球上。
const manual = vm.runInContext(`(function(){
  Object.assign(P, {src:'vmsg', law:'brake', policy:'fast', jerk:0, vcap:10, vt:0, vsig:0,
    noise:0, measHz:5, latency:200, lead:0, heading:90, turn:'straight', startD:40, end:'match',
    accel:3, decel:3, slew:3});
  resetSims(); P.paused = true;
  let maxPred = 0, minD = 1e9;
  for (let i = 0; i < 6000; i++) {
    stepSim(A, DT);
    if (A.est.pred) {
      maxPred = Math.max(maxPred, Math.hypot(A.est.pred.x - A.target.x,
                                            A.est.pred.y - A.target.y));
    }
    minD = Math.min(minD, A.d);
    if (A.arrived) break;
  }
  return {maxPred: +maxPred.toFixed(3), minD: +minD.toFixed(2), arrived: !!A.arrived};
})()`, ctx);
check(manual.maxPred < 0.05, `人工固定点预测点不抖（最大 ${manual.maxPred} m）`,
      `人工固定点预测点抖动 ${manual.maxPred} m`);
check(manual.minD >= 2.95, `人工固定点停在 3 m 球上（最小 ${manual.minD} m）`,
      `人工固定点冲进球内（最小 ${manual.minD} m）`);

// 加速度图口径：加速/减速必须分别有正负；J=0 时指令加速度不能恒 0（用 cmd 差分）。
const accel = vm.runInContext(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', jerk:0, vcap:10, vt:2, heading:90,
    turn:'straight', startD:40, end:'match', accel:3, decel:3, slew:3, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  let neg = 0, pos = 0, cmdMax = 0;
  for (const p of A.hist) {
    if (p.aAct < -0.5) neg++;
    if (p.aAct > 0.5) pos++;
    cmdMax = Math.max(cmdMax, Math.abs(p.aCmd));
  }
  return {neg, pos, cmdMax: +cmdMax.toFixed(2)};
})()`, ctx);
check(accel.neg > 0 && accel.pos > 0, `加速度图有正有负（加速 ${accel.pos} 拍 / 减速 ${accel.neg} 拍）`,
      `加速度图没有负值（neg=${accel.neg} pos=${accel.pos}）`);
check(accel.cmdMax > 1.0, `指令加速度不恒 0（最大 ${accel.cmdMax} m/s²）`,
      `指令加速度恒 0（J=0 时 s.acc 未更新）`);

// 目标运动模型：六种模式都要能跑、位置有限、能进 3 m 球。
const modes = ['straight', 'accel', 'circle', 'spiral', 'weave', 'jink'];
const badModes = [];
for (const turn of modes) {
  const r = new vm.Script(`(function(){
    Object.assign(P, {src:'truth', law:'brake', policy:'fast', jerk:0, vcap:10, vt:3, heading:90,
      turn:'${turn}', startD:40, end:'match', accel:3, decel:3, slew:3, lead:0, noise:0, vsig:0});
    resetSims(); P.paused = true;
    for (let i = 0; i < 3000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {arrived: !!A.arrived, t: +A.t.toFixed(1),
      finite: [A.target.x, A.target.y, A.tv.x, A.tv.y].every(Number.isFinite)};
  })`).runInContext(ctx)();
  if (!r.arrived || !r.finite) badModes.push(`${turn}(arrived=${r.arrived},finite=${r.finite})`);
}
check(badModes.length === 0, `六种目标运动都能跑完并到点（${modes.length} 种）`,
      `目标运动有问题：${badModes.join(' ')}`);

// 仿真不再设 60 s 截止：120 m、目标 2 m/s、机体上限 3（接近速率 1 m/s）要 118 s 才到点。
// 直接调 frame() 走真实的"播放循环"，把四个绘制函数换成空函数，只验步进/停止条件。
const longRun = vm.runInContext(`(function(){
  const realHud = updateHud;   // 2026-11-10：后面的回归用例还要读判词，跑完恢复
  drawScene = () => {}; drawDistanceChart = () => {}; drawSpeedChart = () => {};
  drawAccelChart = () => {}; updateHud = () => {};
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', jerk:0, vcap:3, vt:2, heading:90,
    turn:'straight', startD:120, end:'match', accel:3, decel:3, slew:3, lead:0, noise:0, vsig:0,
    speed:8, paused:false});
  resetSims();
  for (let i = 1; i <= 500; i++) frame(i * 50);   // 500×50 ms 真实时间 ×8 倍速 = 200 s 仿真
  updateHud = realHud;
  return {arrived: !!A.arrived, t: +A.t.toFixed(1),
          arrivedAt: A.arrivedAt === null ? null : +A.arrivedAt.toFixed(1)};
})()`, ctx);
check(longRun.arrived && longRun.t > 60 && longRun.t < 140,
      `播放循环跑到追上为止（${longRun.t} s，无 60 s 截止）`,
      `播放循环没有跑到追上：t=${longRun.t} arrived=${longRun.arrived}`);
// ==== 2026-11-10 修复回归 ==================================================

// ① 上限滑块 0：理论、速度律、判词统一按规划速度 6 m/s（原来理论按无限速、律按 6，
//    同屏出现"理论峰值 9.82 / 实际峰值 6.00"）。
const capZero = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:0, vt:0, heading:90, turn:'straight', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 4000; i++) { stepSim(A, DT); if (A.arrived) break; }
  return {thVp:+THEORY.vp.toFixed(2), thPl:THEORY.plateau, vAct:+(A.vactPeak||0).toFixed(2),
          vCmd:+(A.vpeak||0).toFixed(2), arrived:!!A.arrived};
})`).runInContext(ctx)();
check(capZero.thPl === 6 && capZero.thVp === 6 && capZero.vAct <= 6.05 && capZero.vCmd <= 6.05,
      `vcap=0 统一按 6 m/s（理论 ${capZero.thVp} / 实际 ${capZero.vAct}）`,
      `vcap=0 口径不一致：理论 ${capZero.thVp} plateau ${capZero.thPl} 实际 ${capZero.vAct} 指令 ${capZero.vCmd}`);

// ② end=stop + 静止目标：停在到点半径上（2026-11-10 前写死 0.6 m，与 handover 的 3 m 冲突）。
const stopStatic = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'stop', accel:3, decel:3,
    slew:3, vcap:10, vt:0, heading:90, turn:'straight', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  return {arrived:!!A.arrived, minD:+(A.minD ?? A.d).toFixed(2), t:+A.t.toFixed(2)};
})`).runInContext(ctx)();
check(stopStatic.arrived && stopStatic.minD >= 2.95 && stopStatic.minD <= 3.4,
      `end=stop 静止目标停在 3 m 球附近（${stopStatic.minD} m，${stopStatic.t} s）`,
      `end=stop 停车点不对：arrived=${stopStatic.arrived} minD=${stopStatic.minD}`);

// ③ end=stop + 动目标：物理上不相容（机速 0 时目标仍在离开），判词必须点明，
//    闭式解也必须判为不适用——不能装作"再跑一会儿就能到点"。
const stopMoving = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'stop', accel:3, decel:3,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 3000; i++) { stepSim(A, DT); if (A.arrived) break; }
  updateHud();
  return {arrived:!!A.arrived, minD:+(A.minD ?? A.d).toFixed(2),
          verdict: document.getElementById('verdict').innerHTML,
          applicable: theoryApplicable()};
})`).runInContext(ctx)();
check(!stopMoving.arrived && stopMoving.verdict.includes('不相容') && !stopMoving.applicable,
      `end=stop + 动目标判为不相容（最近接近 ${stopMoving.minD} m）`,
      `end=stop + 动目标没有正确提示：arrived=${stopMoving.arrived} applicable=${stopMoving.applicable}`);

// ④ 机动目标：只报"到点"会掩盖穿球，读数条要给出最小距离与包络越界。
const maneuver = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:12, vt:4, heading:90, turn:'circle', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  updateHud();
  return {arrived:!!A.arrived, minD:+(A.minD ?? A.d).toFixed(2), envPeak:+(A.envPeak||0).toFixed(2),
          note: document.getElementById('minNote').innerHTML};
})`).runInContext(ctx)();
check(maneuver.arrived && maneuver.minD < 3.0 && maneuver.envPeak > 0 && maneuver.note.includes('最小距离'),
      `机动工况暴露穿球/越界（最小 ${maneuver.minD} m、越界 ${maneuver.envPeak} m/s）`,
      `机动工况没有暴露穿球：minD=${maneuver.minD} envPeak=${maneuver.envPeak}`);

// ⑤ 默认加载（速度位 + σ=σ_v=0）：闭式解对匀速直线目标精确，理论行必须可见
//    （原来门控要求 src='truth'，默认页面永远是"—"）。
const defaults = new vm.Script(`(function(){
  Object.assign(P, {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    noise:0, vsig:0, latency:200, measHz:5});
  resetSims();
  return {applicable: theoryApplicable(), vp:+THEORY.vp.toFixed(2)};
})`).runInContext(ctx)();
check(defaults.applicable && defaults.vp === 10,
      `默认页面理论行可见（理论峰值 ${defaults.vp} m/s）`,
      `默认页面理论行仍不可用（applicable=${defaults.applicable}）`);

// ⑥ 纯横穿把速度预算吃满时判词必须是"追不上"（原来要求 v_t∥>0.05，会落进"时间最优"）。
//    几何取 100 m、目标 10 m/s 纯横穿，跑 2 拍：v_t∥≈0.04 < 0.05，closing≈0。
const crossVerdict = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:10, heading:0, turn:'straight', startD:100, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 2; i++) stepSim(A, DT);
  updateHud();
  return {verdict: document.getElementById('verdict').innerHTML,
          vtAlong:+A.vtAlong.toFixed(3), vtCross:+A.vtCross.toFixed(3)};
})`).runInContext(ctx)();
check(crossVerdict.verdict.includes('追不上'),
      `纯横穿预算吃满时报"追不上"（v_t∥=${crossVerdict.vtAlong}，v_t⊥=${crossVerdict.vtCross}）`,
      '纯横穿落进"时间最优"分支（vtAlong 门控又回来了？）');

// ⑦ 长跑防护：误差数组必须有上限（2026-11-10 前无界，300 s 涨到上万条）。
const bounded = new vm.Script(`(function(){
  Object.assign(P, {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:12, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    noise:0.8, vsig:0.1, latency:200, measHz:5});
  resetSims(); P.paused = true;
  for (let i = 0; i < 30000; i++) stepSim(A, DT);
  return {p:A.est.errPred.length, m:A.est.errMeas.length, e:A.est.errEst.length};
})`).runInContext(ctx)();
check(bounded.p <= 600 && bounded.m <= 200 && bounded.e <= 200,
      `长跑误差数组有上限（${bounded.p}/${bounded.m}/${bounded.e}）`,
      `长跑误差数组无界增长（${bounded.p}/${bounded.m}/${bounded.e}）`);

// ⑧ 刹车点读数与律同口径：d_brake = R + u²/(2·(a_d/1.3))（去掉多余的 0.5 m 垫片）。
const brakeRing = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 200; i++) stepSim(A, DT);
  const aEff = P.decel / BRAKE_SAFETY;
  return {dbrk: A.dbrk, expect: ARRIVAL_RADIUS_M + A.vc * A.vc / (2 * aEff)};
})`).runInContext(ctx)();
check(Math.abs(brakeRing.dbrk - brakeRing.expect) < 1e-6,
      `刹车点读数与包络同口径（${brakeRing.dbrk.toFixed(2)} m）`,
      `刹车点读数偏离包络：${brakeRing.dbrk} vs ${brakeRing.expect}`);

// ⑨ 蛇形 7 m/s（维护者原始提问）：修转弯约束后必须能到点——旧约束把 7 m/s 目标的
//    前馈一起削到 1.4 m/s，导致"进过球但相对速度压不到 0.5、永远不置位"。
const weave7 = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:7, heading:90, turn:'weave', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  updateHud();
  return {arrived:!!A.arrived, t:+A.t.toFixed(2), minD:+(A.minD??A.d).toFixed(2),
          minDRel:+(A.minDRel||0).toFixed(2), tgtA:+A.tgtAccelPeak.toFixed(2),
          verdict: document.getElementById('verdict').innerHTML};
})`).runInContext(ctx)();
check(weave7.arrived && weave7.t > 12 && weave7.t < 25 && weave7.minD <= 3.05,
      `蛇形 7 m/s 可到点（${weave7.t} s，最近 ${weave7.minD} m，a_t=${weave7.tgtA}）`,
      `蛇形 7 m/s 仍到不了：arrived=${weave7.arrived} t=${weave7.t} minD=${weave7.minD}`);

// ⑩ 能力边界由加速度决定、不是速度上限：蛇形 8 m/s（a_t=4.22）时 a=4.5 能到点，
//    vcap 提到 16 仍不收敛（目标横向扫过还会穿到球内）。
const weave8 = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match',
    vt:8, heading:90, turn:'weave', startD:40, jerk:0, lead:0};
  const runOne = extra => {
    Object.assign(P, base, {accel:3, decel:3, slew:3, vcap:10}, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {arrived:!!A.arrived, minD:+(A.minD??A.d).toFixed(2)};
  };
  return {base: runOne({}), a45: runOne({accel:4.5, decel:4.5, slew:4.5}),
          cap16: runOne({vcap:16})};
})`).runInContext(ctx)();
check(!weave8.base.arrived && weave8.a45.arrived && !weave8.cap16.arrived,
      `蛇形 8 m/s 的瓶颈是加速度（a=4.5 到点，a=3 最近 ${weave8.base.minD} m）而非速度上限`,
      `瓶颈判断失败：base=${weave8.base.arrived} a4.5=${weave8.a45.arrived} vcap16=${weave8.cap16.arrived}`);

// ⑪ 转弯约束只削"额外接近"、不削前馈：绕圈 7 m/s（向心只需 2.45 < a=3）必须能到点；
//    约束仍会在相对横向速度大时 bind，但不能把目标速度一起削掉。
const circle7 = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:12, vt:7, heading:90, turn:'circle', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  updateHud();
  return {arrived:!!A.arrived, t:+A.t.toFixed(2), tgtA:+A.tgtAccelPeak.toFixed(2),
          turnBind:A.turnBind, capMin: isFinite(A.turnCapMin) ? +A.turnCapMin.toFixed(2) : null,
          verdict: document.getElementById('verdict').innerHTML};
})`).runInContext(ctx)();
check(circle7.arrived && circle7.t >= 15 && circle7.t <= 40 && circle7.turnBind && circle7.tgtA < 3,
      `绕圈 7 m/s 可到点（${circle7.t} s，约束仍 bind=${circle7.turnBind}，不削前馈）`,
      `绕圈 7 m/s 不到点：arrived=${circle7.arrived} t=${circle7.t} bind=${circle7.turnBind}`);

// ⑫ 未进球时也要报能力边界：绕圈 8 m/s（a_t=3.2）必须说"目标机动超出机体能力"，
//    不能落进"时间最优 / 可追上"。
const circle8 = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:12, vt:8, heading:90, turn:'circle', startD:40, jerk:0, lead:0});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
  updateHud();
  return {arrived:!!A.arrived, minD:+(A.minD??A.d).toFixed(2), tgtA:+A.tgtAccelPeak.toFixed(2),
          verdict: document.getElementById('verdict').innerHTML};
})`).runInContext(ctx)();
check(!circle8.arrived && circle8.minD > 3 && circle8.verdict.includes('目标机动超出'),
      `绕圈 8 m/s 未进球时报能力边界（最近 ${circle8.minD} m，a_t=${circle8.tgtA}）`,
      `绕圈 8 m/s 归因不对：arrived=${circle8.arrived} minD=${circle8.minD}`);

process.exit(ok ? 0 : 1);
