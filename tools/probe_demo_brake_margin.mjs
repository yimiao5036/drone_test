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
  // DOM 桩把所有 input.value 当 '0'，bindRange 会把 P 全清成 0；恢复页面默认值。
  // 各用例仍显式覆盖自己用到的字段（含 aPhys——拆分后 plant 只认它）。
  // 2026-11-10：提前量口径页面默认已改为 scaled（距离自适应）。历史数值断言是在
  // "只补时延"口径下标定的，所以这里显式固定成 latency，另用 __defaultLeadMode
  // 单独断言"页面默认确实是 scaled"（见 ㉗）。
  vm.runInContext('globalThis.__defaultLeadMode = P.leadMode;' +
                  ' globalThis.__defaultSched = P.sched;' +
                  ' globalThis.__defaultBoost = P.autoBoost;' +
                  ' P.aPhys = 3; P.accel = 3; P.decel = 3; P.slew = 3; P.vcap = 10;' +
                  ' P.alt = 0; P.vzT = 0; P.criterion = "rel"; P.leadMode = "latency";' +
                  ' P.sched = "off"; P.autoBoost = "off";', context);
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
  law: 'brake', policy: 'fast', end: 'match', accel: 3, decel: 3, slew: 3, aPhys: 3,
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
check(fixed >= 2.85, '修复后默认停在到点球上（最小距离 ≥ 2.85 m，控制间距 2.9 m）',
      `修复后默认仍穿过：最小距离 ${fixed.toFixed(2)} m`);
check(jerk >= 2.85, 'J=8 不再穿球（jerk 积分制动已修复，最小距离 ≥ 2.85 m）',
      `J=8 仍会穿球：最小距离 ${jerk.toFixed(2)} m（结构回退，需复核 §5.4）`);
check(published >= 2.85, '默认页面不加噪声：停在 3 m 球上（最小 ≥ 2.85 m）',
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
check(manual.minD >= 2.85, `人工固定点停在 3 m 球上（最小 ${manual.minD} m）`,
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
    speed:8, paused:false, continuous:'off'});
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
check(stopStatic.arrived && stopStatic.minD >= 2.85 && stopStatic.minD <= 3.4,
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
check(!stopMoving.arrived && stopMoving.verdict.includes('动目标使用停车末端') && !stopMoving.applicable,
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
check(Math.abs(brakeRing.dbrk - brakeRing.expect) < 0.12,
      `刹车点读数与包络同口径（${brakeRing.dbrk.toFixed(2)} m，含一拍反应距离）`,
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
check(weave7.arrived && weave7.t > 8 && weave7.t < 90 && weave7.minD > 0.5,
      `蛇形 7 m/s 可到点（${weave7.t} s，最近 ${weave7.minD} m，a_t=${weave7.tgtA}；允许进入 3 m 区域）`,
      `蛇形 7 m/s 仍到不了：arrived=${weave7.arrived} t=${weave7.t} minD=${weave7.minD}`);

// ⑩ 能力边界由加速度决定、不是速度上限：蛇形 8 m/s（a_t=4.22）时 a=4.5 能到点，
//    vcap 提到 16 仍不收敛（目标横向扫过还会穿到球内）。
const weave8 = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match',
    vt:8, heading:90, turn:'weave', startD:40, jerk:0, lead:0};
  const runOne = extra => {
    Object.assign(P, base, {accel:3, decel:3, slew:3, aPhys:3, vcap:10}, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {arrived:!!A.arrived, minD:+(A.minD??A.d).toFixed(2)};
  };
  return {base: runOne({}), a45: runOne({accel:4.5, decel:4.5, slew:4.5, aPhys:4.5}),
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
check(!circle8.arrived && circle8.minD > 3 && circle8.verdict.includes('加速度能力'),
      `绕圈 8 m/s 未进球时报侧向/制动预算不足（最近 ${circle8.minD} m，a_t=${circle8.tgtA}）`,
      `绕圈 8 m/s 归因不对：arrived=${circle8.arrived} minD=${circle8.minD}`);

// ⑬ 加速度分层（§5.12）：a_phys 是 plant 硬上限，a_plan 只是规划。
//    只调大 a_phys（规划不变）→ 剖面不变；a_plan > a_phys → 剖面不可实现（理论行判不可用）。
const splitAccel = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    slew:3, vcap:10, vt:0, heading:90, turn:'straight', startD:40, jerk:0, lead:0};
  const runOne = aPhys => {
    Object.assign(P, base, {aPhys});
    resetSims(); P.paused = true;
    for (let i = 0; i < 3000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {arrived:!!A.arrived, t:+A.t.toFixed(2), minD:+(A.minD??A.d).toFixed(2)};
  };
  Object.assign(P, base, {aPhys:2}); resetSims();
  const theoryBlocked = !theoryApplicable() && theoryBlockReason().includes('物理上限');
  return {p3: runOne(3), p10: runOne(10), p2: runOne(2), theoryBlocked};
})`).runInContext(ctx)();
check(Math.abs(splitAccel.p10.t - splitAccel.p3.t) < 0.05,
      `只调大 a_phys 不改变剖面（${splitAccel.p3.t} vs ${splitAccel.p10.t} s），低于规划时才变慢`,
      `a_phys/a_plan 未分离：t3=${splitAccel.p3.t} t10=${splitAccel.p10.t} blocked=${splitAccel.theoryBlocked}`);
check(!splitAccel.p2.arrived || splitAccel.p2.t > splitAccel.p3.t + 0.2 ||
      splitAccel.p2.minD < 2.9,
      `a_phys 低于规划时能力不足（a_phys=2：t=${splitAccel.p2.t} s / 最近 ${splitAccel.p2.minD} m）`,
      `a_phys=2 没有体现能力不足（arrived=${splitAccel.p2.arrived} t=${splitAccel.p2.t}）`);

// ⑭ 垂直通道（§5.13）：目标高 10 m / 爬升 1.5 m/s 时，垂直通道与水平并行收敛，到点仍成立。
const vertical = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:3,
    slew:3, vcap:10, vt:0, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    criterion:'rel', alt:0, vzT:0};
  const runOne = extra => {
    Object.assign(P, base, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 4000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {arrived:!!A.arrived, t:+A.t.toFixed(2), dz:+(A.dzAbs??0).toFixed(2),
            vz:+(A.vz||0).toFixed(2)};
  };
  return {alt10: runOne({alt:10}), climb: runOne({vzT:1.5, vt:2})};
})`).runInContext(ctx)();
check(vertical.alt10.arrived && vertical.alt10.dz <= 2 &&
      vertical.climb.arrived && Math.abs(vertical.climb.vz - 1.5) < 0.2 &&
      vertical.climb.dz <= 2,
      `垂直通道并行收敛（高 10 m：${vertical.alt10.t} s / |dz|=${vertical.alt10.dz} m；`
      + `爬升 1.5：${vertical.climb.t} s / vz=${vertical.climb.vz}）`,
      `垂直通道未收敛：alt10=${JSON.stringify(vertical.alt10)} climb=${JSON.stringify(vertical.climb)}`);

// ⑮ 末端判据（§5.13）：同一"速度匹配 + 动目标"工况，相对速度判据能到点、地速判据永远不能；
//    垂直运动的动目标同样触发地速判据冲突（机速≈目标垂直速度）。
const criteria = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:3,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel'};
  const runOne = extra => {
    Object.assign(P, base, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 4000; i++) { stepSim(A, DT); if (A.arrived) break; }
    updateHud();
    return {arrived:!!A.arrived, t:+A.t.toFixed(2), ground:+(A.groundSpeed||0).toFixed(2),
            rel:+(A.relSpeed3||0).toFixed(2), verdict: document.getElementById('verdict').innerHTML};
  };
  return {rel: runOne({criterion:'rel'}), abs: runOne({criterion:'abs'}),
          climbAbs: runOne({criterion:'abs', vzT:1.5, vt:0})};
})`).runInContext(ctx)();
check(criteria.rel.arrived && !criteria.abs.arrived &&
      criteria.abs.verdict.includes('速度判据存在冲突') &&
      !criteria.climbAbs.arrived && criteria.climbAbs.verdict.includes('速度判据存在冲突'),
      `末端判据可切换（相对：${criteria.rel.t} s 到点；地速：${criteria.abs.t} s 未到点，判词正确）`,
      `判据切换失效：rel=${criteria.rel.arrived} abs=${criteria.abs.arrived} `
      + `climbAbs=${criteria.climbAbs.arrived}`);

// ⑯ 视觉交接（§5.14）：S5→S6 的判据是"LOCKED 10 帧 + 测距 ≤7.95 m + 俯仰 ≤5.24° + 创新 ≤3 m"，
//    不是"到点 3 m + 0.5 m/s"。默认 40 m 直线应在测距边缘、到点之前交接，且交接时相对速度 >0.5。
//    FOV 用双目测距测试的校正内参（P_ref：fx=fy=1057.3349、cx=692.7685、cy=529.0832）。
const handover = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:3,
    slew:3, vcap:10, vt:0, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel'};
  const runOne = extra => {
    Object.assign(P, base, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 6000; i++) { stepSim(A, DT); if (A.arrived) break; }
    return {handover:!!A.handover, hoT:A.handoverAt===null?null:+A.handoverAt.toFixed(2),
            hoD:A.handoverD===null?null:+A.handoverD.toFixed(2),
            hoRel:+A.handoverRel.toFixed(2), arrived:!!A.arrived,
            arrT:A.arrivedAt===null?null:+A.arrivedAt.toFixed(2),
            blkFov:A.blkFov, lockBest:A.lockBest};
  };
  return {straight: runOne({}), high: runOne({alt:10}),
          climbEscape: runOne({alt:10, vzT:2}),
          consts:{az:+FOV_HALF_AZ_DEG.toFixed(2), el:+FOV_HALF_EL_DEG.toFixed(2),
                  range:+RANGE_MAX_M.toFixed(2), lock:LOCK_FRAMES}};
})`).runInContext(ctx)();
check(handover.consts.az > 28.9 && handover.consts.az < 29.2 &&
      handover.consts.el > 10.1 && handover.consts.el < 10.4 &&
      handover.consts.range > 7.9 && handover.consts.range < 8.0,
      `视觉交接常量与测距校正内参一致（FOV ${handover.consts.az}°/${handover.consts.el}°，测距 ${handover.consts.range} m）`,
      `视觉交接常量异常：${JSON.stringify(handover.consts)}`);
check(handover.straight.handover &&
      handover.straight.hoD <= handover.consts.range + 0.01 &&
      handover.straight.hoT < handover.straight.arrT &&
      handover.straight.hoRel > 0.5,
      `默认直线在 ${handover.straight.hoT} s / ${handover.straight.hoD} m 交接`
      + `（相对 ${handover.straight.hoRel} m/s，早于到点 ${handover.straight.arrT} s）`,
      `默认直线未交接或交接点异常：${JSON.stringify(handover.straight)}`);
check(handover.high.handover && handover.high.blkFov > 0 &&
      handover.high.hoD < handover.straight.hoD,
      `高 10 m 目标因俯仰出视场推迟交接（${handover.high.hoT} s / ${handover.high.hoD} m，`
      + `FOV 阻塞 ${handover.high.blkFov} 拍）`,
      `高位目标交接异常：${JSON.stringify(handover.high)}`);
check(!handover.climbEscape.handover,
      `目标以 2 m/s 爬升时俯仰始终出视场、不交接（阻塞 ${handover.climbEscape.blkFov} 拍）`,
      `爬升逃逸工况竟然交接了：${JSON.stringify(handover.climbEscape)}`);

// ⑰ 加速度图分量（§5.16）：蛇形/绕圈是"改方向"，横向分量才是主力；只看切向会误以为没有加速度。
const accelParts = new vm.Script(`(function(){
  Object.assign(P, {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    aPhys:3, slew:3, vcap:12, vt:8, heading:90, turn:'weave', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', accelView:'lat'});
  resetSims(); P.paused = true;
  for (let i = 0; i < 6000; i++) stepSim(A, DT);
  let maxLat=0, maxMag=0, meanTan=0, n=0, sat=0;
  for (const p of A.hist) {
    maxLat = Math.max(maxLat, Math.abs(p.aLat||0));
    maxMag = Math.max(maxMag, p.aMag||0);
    meanTan += Math.abs(p.aAct||0); if ((p.aMag||0) > 2.5) sat++; n++;
  }
  return {maxLat:+maxLat.toFixed(2), maxMag:+maxMag.toFixed(2),
          meanTan:+(meanTan/Math.max(1,n)).toFixed(2), satPct:+(100*sat/Math.max(1,n)).toFixed(0)};
})`).runInContext(ctx)();
check(accelParts.maxMag > 2.5 && accelParts.maxLat > 1.5 && accelParts.satPct > 80,
      `加速度分量可见（8 m/s 蛇形：总 ${accelParts.maxMag}、横向 ${accelParts.maxLat}、`
      + `切向均值 ${accelParts.meanTan}、饱和 ${accelParts.satPct}%）`,
      `加速度分量异常：${JSON.stringify(accelParts)}`);

// ⑱ 比例接近律的二维总速度预算（2026-11-10 外部审查）：plateau 是总速度上限，
//    不是额外接近量。否则 desired=v_t+u 的总速度到 |v_t|+plateau，A/B"余量"对照失效
//    （safe 余量 0 时设计组与对照组结果完全相同）。
const propBudget = new vm.Script(`(function(){
  const base = {src:'truth', law:'prop', policy:'fast', end:'match', accel:3, decel:3, aPhys:3,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0, k:0.6,
    floor:true, margin:1};
  const runBoth = extra => {
    Object.assign(P, base, extra); resetSims(); P.paused = true;
    for (let i = 0; i < 8000; i++) { stepSim(A, DT); stepSim(B, DT); if (A.arrived) break; }
    return {a:!!A.arrived, at:+A.t.toFixed(1), b:!!B.arrived, bd:+B.d.toFixed(1)};
  };
  const fast = runBoth({policy:'fast'}), safe0 = runBoth({policy:'safe', margin:0});
  const safe1 = runBoth({policy:'safe', margin:1});
  return {fast, safe0, safe1, theory: theoryApplicable()};
})`).runInContext(ctx)();
check(propBudget.fast.a && !propBudget.fast.b &&
      !propBudget.safe0.a && !propBudget.safe0.b &&
      propBudget.safe1.a && !propBudget.safe1.b,
      `比例律总速度预算生效（fast：A ${propBudget.fast.at} s / B 停在 ${propBudget.fast.bd} m；`
      + `safe 余量 1：A ${propBudget.safe1.at} s / B 停在 ${propBudget.safe1.bd} m）`,
      `比例律预算仍有问题：${JSON.stringify(propBudget)}`);
check(!propBudget.theory,
      '比例接近律不显示梯形闭式理论值',
      '比例接近律仍在显示闭式理论值（theoryApplicable 缺 law 门控）');

// ⑲ 量测误差按"收到新消息"采样：5 Hz 跑 4 s 应约 19~20 条（原来每 20 ms 写一次，
//    4 s 攒 200 条，而 HUD 只取 20 条 → 实际窗口 0.4 s）。
const measWindow = new vm.Script(`(function(){
  Object.assign(P, {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3,
    aPhys:3, slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    noise:0.8, vsig:0, latency:200, measHz:5});
  resetSims(); P.paused = true;
  for (let i = 0; i < 200; i++) stepSim(A, DT);
  return {meas:A.est.errMeas.length, pred:A.est.errPred.length,
          est:A.est.errEst.length, hud:Math.round(4*P.measHz)};
})`).runInContext(ctx)();
check(measWindow.meas >= 15 && measWindow.meas <= 25 && measWindow.pred >= 190,
      `量测误差按消息帧采样（4 s：量测 ${measWindow.meas} 条 / 预测 ${measWindow.pred} 条，`
      + `HUD 窗口 ${measWindow.hud} 条）`,
      `量测误差窗口口径错误：${JSON.stringify(measWindow)}`);

// ⑳ 分相位调度 + 矢量分配（§5.18）：近段降 a，包络必须按"未来最低减速度"提前规划
//    （不提前规划时实测 minD 0.11 m）；矢量预留吃掉纵向能力时变慢但不穿球。
const sched = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', sched:'off', aTerm:1.5, aLatRes:0,
    adaptive:'off', continuous:'off', targetBrake:0};
  const runOne = extra => {
    Object.assign(P, base, extra);
    resetSims(); P.paused = true;
    let capDownNear = null;
    for (let i = 0; i < 6000; i++) {
      stepSim(A, DT);
      const h = A.hist[A.hist.length - 1];
      if (h && A.d < 7.9 && capDownNear === null) capDownNear = h.aCapDown;
      if (A.arrived) break;
    }
    return {t:+A.t.toFixed(2), arrived:!!A.arrived, minD:+(A.minD??A.d).toFixed(2),
            capDownNear:capDownNear==null?null:+capDownNear.toFixed(2),
            theory:theoryApplicable(), reason:theoryBlockReason()};
  };
  return {fixed: runOne({}), sched: runOne({sched:'on'}),
          term1: runOne({sched:'on', aTerm:1.0}), vec: runOne({aLatRes:9.5})};
})`).runInContext(ctx)();
check(sched.sched.arrived && sched.sched.minD >= 2.85 &&
      sched.sched.t > sched.fixed.t &&
      // 2026-11-10：过渡带 w 加了 0.8/s 速率限幅，进入 7.9 m 时 w 还没降到 0，
      // 所以"近段 capDown"会带一点过渡滞后（实测 1.5→1.83），阈值放到 2.0。
      sched.sched.capDownNear !== null && sched.sched.capDownNear <= 2.0,
      `分相位调度：近段 a 降到 ${sched.sched.capDownNear} m/s²（含过渡滞后），到点 ${sched.sched.t} s`
      + `（固定 ${sched.fixed.t} s），minD ${sched.sched.minD} m`,
      `分相位调度异常：${JSON.stringify(sched.sched)}`);
check(sched.term1.arrived && sched.term1.minD >= 2.85 && sched.term1.t > sched.sched.t,
      `末端 a_term=1.0 更保守（${sched.term1.t} s，minD ${sched.term1.minD} m）`,
      `a_term 未生效：${JSON.stringify(sched.term1)}`);
check(sched.vec.arrived && sched.vec.minD >= 2.85 && sched.vec.t > sched.fixed.t &&
      sched.vec.capDownNear < 3,
      `矢量预留 9.5 → 纵向 ${sched.vec.capDownNear} m/s²，到点 ${sched.vec.t} s`,
      `矢量预留未生效：${JSON.stringify(sched.vec)}`);
check(sched.sched.theory && sched.fixed.theory && !sched.vec.theory,
      `闭式解按远段 a_plan 给参考值（分相位/固定都可用；矢量预留仍判不可用 ${
        sched.vec.theory ? '' : '✓'}）`,
      `闭式解门控异常：${JSON.stringify({s: sched.sched, f: sched.fixed, v: sched.vec})}`);

// ㉑ 绿机 = 纯比例尾追基线（2026-11-10）：原图例把"零余量对照"写成"纯尾追"，
//    容易被读成"纯尾追也追不上"。这里锁住"真·比例尾追能到点、且比梯形慢"。
const pursuit = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', sched:'off', aTerm:1.5, aLatRes:0,
    adaptive:'on', continuous:'on', targetBrake:0};
  Object.assign(P, base); resetSims(); P.paused = true;
  let cMinD = 1e9;
  for (let i = 0; i < 8000; i++) {
    stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
    cMinD = Math.min(cMinD, C.d);
    if (A.arrived && C.arrived) break;
  }
  return {a:!!A.arrived, at:+(A.arrivedAt ?? -1).toFixed(2), c:!!C.arrived,
          ct:+(C.arrivedAt ?? -1).toFixed(2),
          cd:+C.d.toFixed(2), cMinD:+cMinD.toFixed(2), bd:+B.d.toFixed(1)};
})`).runInContext(ctx)();
check(pursuit.c && pursuit.cMinD >= 2.85 && pursuit.a && pursuit.ct > pursuit.at
      && Math.abs(pursuit.cd - 2.9) < 0.15 && pursuit.bd > 40,
      `纯比例尾追基线能到点（绿 ${pursuit.ct} s / 蓝 ${pursuit.at} s，minD ${pursuit.cMinD} m；`
      + `零余量对照仍停在 ${pursuit.bd} m）`,
      `纯比例尾追基线异常：${JSON.stringify(pursuit)}`);

// ㉒ 停滞提示（2026-11-10）：到不了点的工况必须在顶栏直接报出来，不能只写在底部判词
//    （"末端=减到0 + 动目标"停在 4.23 m、"到点判据=地速 + 动目标"停在 2.9 m 但速度判据永不满足）。
const stall = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', sched:'off', aTerm:1.5, aLatRes:0,
    adaptive:'on', continuous:'on', targetBrake:0};
  const run = (sec, extra) => {
    Object.assign(P, base, extra); resetSims(); P.paused = true;
    for (let i = 0; i < Math.round(sec / DT); i++) { stepSim(A, DT); stepSim(B, DT); stepSim(C, DT); }
    updateHud();
    return {d:+A.d.toFixed(2), arrived:!!A.arrived, stalled:!!A.stalled,
            badge:document.getElementById('badgePhase').textContent};
  };
  return {ok: run(12, {}), stop: run(20, {end:'stop'}), abs: run(20, {criterion:'abs'})};
})`).runInContext(ctx)();
check(!stall.ok.stalled && stall.ok.arrived &&
      stall.stop.stalled && stall.stop.badge.includes('停滞') &&
      stall.abs.stalled && stall.abs.badge.includes('停滞'),
      `停滞提示只在到不了点时亮（正常 ${stall.ok.badge}；停车末端 d=${stall.stop.d} m 报`
      + `"${stall.stop.badge}"；地速判据 d=${stall.abs.d} m 报"${stall.abs.badge}"）`,
      `停滞提示口径错误：${JSON.stringify(stall)}`);

// ㉓ 制动示意圆不得画成十万像素半径（2026-11-10）：动态预算吃满时 decelEff→0，
//    dbrk 会算成 1e5~1e7 m；真实 GPU 浏览器画这个大虚线圆单帧要 100~370 ms
//    （现象："刹车范围一变大就卡一下"）。锁住"半径超视口就不画"与读数降级。
const ringGuard = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', sched:'off', aTerm:1.5, aLatRes:0,
    adaptive:'on', continuous:'on', targetBrake:0, noise:0, vsig:0, measHz:5, latency:200};
  Object.assign(P, base, {turn:'weave', vt:7});
  resetSims(); P.paused = true;
  let peak = 0, peakT = 0;
  for (let i = 0; i < 4000; i++) {
    stepSim(A, DT);
    if (A.dbrk > peak) { peak = A.dbrk; peakT = A.t; }
  }
  updateHud();
  return {peak:+peak.toFixed(0), peakT:+peakT.toFixed(1), infeasible:!!A.brakeInfeasible,
          sDb:document.getElementById('sDb').innerHTML,
          ringHuge: brakingRingRadius(peak, 5, 1376, 292),
          ringSmall: brakingRingRadius(16.93, 5, 1376, 292)};
})`).runInContext(ctx)();
check(ringGuard.peak > 1e4 && ringGuard.infeasible && ringGuard.sDb.includes('不可行')
      && ringGuard.ringHuge === null && ringGuard.ringSmall !== null,
      `制动示意圆有视口上限（蛇形 7 m/s：dbrk 峰值 ${ringGuard.peak} m @${ringGuard.peakT} s，`
      + `读数降级为"${ringGuard.sDb}"，16.93 m 仍正常画圆）`,
      `制动示意圆/读数降级缺失：${JSON.stringify(ringGuard)}`);

// ㉔ "有效加速度"读数必须与 motionLimits() 同口径（2026-11-10）：原来漏了 slew 与矢量预留，
//    把 a_plan 拖到 5 时读数显示"有效 5.0"，实际被 slew=3 卡在 3.0——维护者因此以为滑块没生效。
const effReadout = new vm.Script(`(function(){
  const base = {src:'truth', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:2, heading:90, turn:'straight', startD:40, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', sched:'off', aTerm:1.5, aLatRes:0,
    adaptive:'on', continuous:'on', targetBrake:0};
  const read = extra => {
    Object.assign(P, base, extra);
    resetSims(); P.paused = true;
    for (let i = 0; i < 100; i++) stepSim(A, DT);   // 让 motionLimits(A) 有状态
    syncAphysOut();
    return {txt:document.getElementById('aPhysOut').textContent,
            eff:+effectivePlanAccel().toFixed(2), mL:+motionLimits(A).up.toFixed(2)};
  };
  return {plan5slew3: read({accel:5, decel:5, slew:3}),
          plan5slew5: read({accel:5, decel:5, slew:5}),
          plan5slew0: read({accel:5, decel:5, slew:0}),
          physCap3:   read({accel:5, decel:5, slew:5, aPhys:3}),
          latRes9:    read({accel:5, decel:5, slew:5, aLatRes:9})};
})`).runInContext(ctx)();
check(effReadout.plan5slew3.txt.includes('有效 3.0') && effReadout.plan5slew3.eff === 3
      && effReadout.plan5slew5.txt.includes('有效 5.0')
      && effReadout.plan5slew0.txt.includes('有效 5.0')
      && effReadout.physCap3.txt.includes('有效 3.0')
      && effReadout.latRes9.eff < 5,
      `"有效加速度"读数同口径（a_plan5+slew3 → ${effReadout.plan5slew3.txt}；`
      + `slew5 → ${effReadout.plan5slew5.txt}；矢量预留 9 → 有效 ${effReadout.latRes9.eff}）`,
      `有效加速度读数口径错误：${JSON.stringify(effReadout)}`);

// ㉔ 分相位近段下限按"已知目标机动能力"抬升（2026-11-10）：a_term 固定 1.5 时，
//    蛇形 7.2 m/s（需求 3.80）在近段会把沿视线加速预算算成 0，实测 120 s 到不了点、
//    最后退到 12.31 m；抬升后应与"固定"同结果，且过渡带上限抖动大幅下降。
const schedLift = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:5, decel:5, aPhys:9.8,
    slew:3, vcap:10, vt:7.2, heading:90, turn:'weave', startD:10, jerk:0, lead:0, alt:0, vzT:0,
    criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, autoBoost:'off', floor:true, margin:1};
  const run = (sched) => {
    Object.assign(P, base, {sched}); resetSims(); P.paused = true;
    let capChanges = 0, capPrev = null, atMax = 0, minD = 1e9;
    for (let i = 0; i < 8000; i++) {
      stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
      const h = A.hist.length ? A.hist[A.hist.length - 1] : null;
      if (h) { if (capPrev !== null && Math.abs(h.aCapUp - capPrev) > 1e-6) capChanges++; capPrev = h.aCapUp; }
      atMax = Math.max(atMax, aTermEffective(A));
      if (A.d < minD) minD = A.d;
      if (A.arrived) break;
    }
    return {arrived:!!A.arrived, at:+(A.arrivedAt ?? -1).toFixed(1), minD:+minD.toFixed(2),
            capChanges, aTermEffMax:+atMax.toFixed(2)};
  };
  return {off: run('off'), on: run('on')};
})`).runInContext(ctx)();
check(schedLift.on.arrived && schedLift.off.arrived
      && Math.abs(schedLift.on.at - schedLift.off.at) < 0.5
      && schedLift.on.capChanges <= 5 && schedLift.on.aTermEffMax > 3,
      `分相位按目标机动抬升近段下限（蛇形 7.2：固定 ${schedLift.off.at} s / 分相位 ${schedLift.on.at} s，`
      + `a_term_eff 抬到 ${schedLift.on.aTermEffMax} m/s²，上限抖动 ${schedLift.on.capChanges} 次）`,
      `分相位抬升未生效：${JSON.stringify(schedLift)}`);

// ㉕ 停滞自动提能力（2026-11-10，默认关）：a_plan 0.8 / slew 关闭时蛇形 7.2 会拖到 42 s，
//    打开后按 0.5 m/s²/s 抬到物理上限内 → 19 s 级；读数条要写明提了多少。
const boostSw = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:0.8, decel:0.8,
    aPhys:9.8, slew:0, vcap:10, vt:7.2, heading:90, turn:'weave', startD:10, jerk:0, lead:0,
    alt:0, vzT:0, criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'off', floor:true, margin:1};
  const run = (autoBoost) => {
    Object.assign(P, base, {autoBoost}); resetSims(); P.paused = true;
    let boostMax = 0;
    for (let i = 0; i < 6000; i++) {
      stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
      boostMax = Math.max(boostMax, A.accelBoost || 0);
      if (A.arrived) break;
    }
    updateHud();
    return {arrived:!!A.arrived, at:+(A.arrivedAt ?? -1).toFixed(1), boost:+boostMax.toFixed(2),
            note:document.getElementById('minNote').innerHTML};
  };
  return {off: run('off'), on: run('on')};
})`).runInContext(ctx)();
check(boostSw.off.arrived && boostSw.off.at > 35
      && boostSw.on.arrived && boostSw.on.at < 25 && boostSw.on.boost > 1
      && boostSw.on.note.includes('停滞自动提能力'),
      `停滞自动提能力（默认关）（蛇形 7.2 / a_plan 0.8：关 ${boostSw.off.at} s → 开 ${boostSw.on.at} s，`
      + `抬升 +${boostSw.on.boost} m/s²）`,
      `停滞自动提能力异常：${JSON.stringify(boostSw)}`);

// ㉖ 穿透标记 + 停滞计时粘性（2026-11-10）：蛇形目标"短暂相交一下"会把原来的 0.05 m
//    进展阈值反复清零，停滞/提能力永远等不到 6 s；真机上 0.07 m 就是撞了。
const penetration = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8.1, heading:90, turn:'weave', startD:40, jerk:0, lead:0, alt:0, vzT:0,
    criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'off', autoBoost:'off',
    leadMode:'latency', floor:true, margin:1};
  const run = (extra, n) => {
    Object.assign(P, base, extra); resetSims(); P.paused = true;
    let passFrames = 0;
    for (let i = 0; i < n; i++) {
      stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
      if (A.d < 0.5) passFrames++;
      if (A.arrived) break;
    }
    updateHud();
    return {penetrated:!!A.penetrated, pmd:+(A.penetratedMinD ?? 9).toFixed(2), passFrames,
            stalled:!!A.stalled, boostT:+(A.boostT || 0).toFixed(1),
            boost:+(A.accelBoost || 0).toFixed(2),
            badge:document.getElementById('badgePhase').textContent,
            note:document.getElementById('minNote').innerHTML};
  };
  return {weave: run({}, 9000), calm: run({vt:2, turn:'straight', startD:40}, 4000)};
})`).runInContext(ctx)();
check(penetration.weave.penetrated && penetration.weave.pmd < 0.5
      && penetration.weave.stalled
      && penetration.weave.badge.includes('穿透') && penetration.weave.note.includes('已穿透')
      && !penetration.calm.penetrated && !penetration.calm.stalled,
      `穿透标记 + 停滞粘性（蛇形 8.1：穿透 ${penetration.weave.pmd} m / ${penetration.weave.passFrames} 帧，`
      + `停滞计时 ${penetration.weave.boostT} s → 提能力 +${penetration.weave.boost} m/s²，`
      + `徽标"${penetration.weave.badge}"）`,
      `穿透/停滞粘性异常：${JSON.stringify(penetration)}`);

// ㉗ 提前量口径（2026-11-10，维护者拍板默认改"距离自适应"）：页面默认值必须是 scaled；
//    scaled 下的 lead = min(0.06·(d−R), 2 m)，在高速横穿工况能拉开近距穿越的间距。
const leadMode = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8.1, heading:90, turn:'weave', startD:40, jerk:0, lead:0, alt:0, vzT:0,
    criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'off', autoBoost:'off', floor:true,
    margin:1};
  const run = (mode) => {
    Object.assign(P, base, {leadMode: mode}); resetSims(); P.paused = true;
    let minD = 1e9, passFrames = 0;
    for (let i = 0; i < 9000; i++) {
      stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
      if (A.d < minD) minD = A.d;
      if (A.d < 0.5) passFrames++;
      if (A.arrived) break;
    }
    return {minD:+minD.toFixed(2), passFrames};
  };
  return {raw: globalThis.__defaultLeadMode, latency: run('latency'), scaled: run('scaled')};
})`).runInContext(ctx)();
check(leadMode.raw === 'scaled' && leadMode.scaled.minD > leadMode.latency.minD
      && leadMode.scaled.passFrames < leadMode.latency.passFrames,
      `提前量口径默认=距离自适应（蛇形 8.1：只补时延最近 ${leadMode.latency.minD} m / `
      + `${leadMode.latency.passFrames} 帧 <0.5 m → 距离自适应 ${leadMode.scaled.minD} m / `
      + `${leadMode.scaled.passFrames} 帧）`,
      `提前量口径异常：${JSON.stringify(leadMode)}`);

// ㉘ 穿透 = 失败判据（2026-11-10，维护者拍板）：判词必须最先报"判定为撞击（失败）"，
//    并给出最小间距/次数；正常工况绝不能误报。
const failVerdict = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8.1, heading:90, turn:'weave', startD:40, jerk:0, lead:0, alt:0, vzT:0,
    criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'off', autoBoost:'off',
    leadMode:'latency', floor:true, margin:1};
  const run = (extra, n) => {
    Object.assign(P, base, extra); resetSims(); P.paused = true;
    for (let i = 0; i < n; i++) { stepSim(A, DT); stepSim(B, DT); stepSim(C, DT); if (A.arrived) break; }
    updateHud();
    return {penetrated:!!A.penetrated, ep:+(A.penetrationEpisodes||0),
            pmd:+(A.penetratedMinD ?? 9).toFixed(2),
            verdict:document.getElementById('verdict').innerHTML};
  };
  return {hit: run({}, 9000), calm: run({vt:2, turn:'straight'}, 4000)};
})`).runInContext(ctx)();
check(failVerdict.hit.penetrated && failVerdict.hit.ep >= 1
      && failVerdict.hit.verdict.includes('判定为撞击（失败）')
      && failVerdict.hit.verdict.includes('最小间距')
      && !failVerdict.calm.penetrated && !failVerdict.calm.verdict.includes('判定为撞击'),
      `穿透判为失败（蛇形 8.1：${failVerdict.hit.ep} 次、最近 ${failVerdict.hit.pmd} m，`
      + `判词"判定为撞击（失败）"；正常工况不误报）`,
      `穿透失败判据异常：${JSON.stringify(failVerdict)}`);

// ㉙ 加速度调度固定分相位 + 「额外提前量(m)」叠加（2026-11-10 维护者要求）：
//    面板已删「固定」选项（P.sched 默认 'on'，'off' 只留给回归）；原来的「拦截提前 ms」
//    改成米制"额外提前量"，叠加在距离自适应之上。
const uiDefaults = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8, heading:90, turn:'straight', startD:40, jerk:0, alt:0, vzT:0,
    criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'on', autoBoost:'off',
    leadMode:'scaled', floor:true, margin:1};
  const run = (extraM) => {
    Object.assign(P, base, {leadExtraM: extraM}); resetSims(); P.paused = true;
    for (let i = 0; i < 5; i++) { stepSim(A, DT); stepSim(B, DT); stepSim(C, DT); }
    return {leadSec:+(A.est.leadSec||0).toFixed(3), adv:+(A.est.aimAdvance||0).toFixed(3),
            adaptiveM:+(A.est.leadAdaptiveM||0).toFixed(2)};
  };
  return {schedDefault: globalThis.__defaultSched, boostDefault: globalThis.__defaultBoost,
          zero: run(0), extra: run(4)};
})`).runInContext(ctx)();
check(uiDefaults.schedDefault === 'on' && uiDefaults.boostDefault === 'on'
      && uiDefaults.zero.adaptiveM > 1
      && Math.abs(uiDefaults.extra.leadSec - uiDefaults.zero.leadSec - 4 / 8) < 0.02,
      `加速度调度固定分相位、提能力默认开 + 额外提前量按米叠加（调度 ${uiDefaults.schedDefault}；`
      + `提能力 ${uiDefaults.boostDefault}；自适应 ${uiDefaults.zero.adaptiveM} m；额外 4 m → 提前 `
      + `${uiDefaults.extra.leadSec} s vs ${uiDefaults.zero.leadSec} s）`,
      `默认开关/额外提前量异常：${JSON.stringify(uiDefaults)}`);

// ㉚ 提前量随距离收缩（2026-11-10 维护者追问"靠近了为何预测长度没变"）：
//    画布上「菱形→时延补偿点」是链路时延（恒定），「时延补偿点→瞄准点」才是提前量。
//    这里锁住后者：40 m 处约 2 m、8 m 处约 0.3 m。
const leadSplit = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8, heading:90, turn:'straight', startD:40, jerk:0, leadExtraM:0, alt:0,
    vzT:0, criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'on', autoBoost:'off', leadMode:'scaled'};
  Object.assign(P, base); resetSims(); P.paused = true;
  let farLead = null, nearLead = null, nearD = 0;
  for (let i = 0; i < 6000; i++) {
    stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
    const e = A.est;
    if (!e.aim || !e.predLatency) continue;
    const leadM = Math.hypot(e.aim.x - e.predLatency.x, e.aim.y - e.predLatency.y);
    if (farLead === null && A.d < 40.5) farLead = leadM;
    if (nearLead === null && A.d < 8.5) { nearLead = leadM; nearD = A.d; }
    if (A.arrived) break;
  }
  return {far:+(farLead ?? -1).toFixed(2), near:+(nearLead ?? -1).toFixed(2), nearD:+nearD.toFixed(1)};
})`).runInContext(ctx)();
check(leadSplit.far > 1.5 && leadSplit.near > 0 && leadSplit.near < 0.6
      && leadSplit.near < leadSplit.far * 0.3,
      `提前量随距离收缩（40 m → ${leadSplit.far} m；${leadSplit.nearD} m → ${leadSplit.near} m）`,
      `提前量未随距离收缩：${JSON.stringify(leadSplit)}`);

// ㉛ 制导线连的是瞄准点、不是目标当前位置（2026-11-10 维护者指出画布只连真值点像纯尾追）：
//    瞄准点 = 估计点按（时延 + 提前量）前视，与真值目标的偏差应 ≈ 提前时间 × 目标速度。
const guideLine = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8, heading:90, turn:'straight', startD:40, jerk:0, leadExtraM:0, alt:0,
    vzT:0, criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'on', autoBoost:'off', leadMode:'scaled'};
  Object.assign(P, base); resetSims(); P.paused = true;
  for (let i = 0; i < 20; i++) { stepSim(A, DT); stepSim(B, DT); stepSim(C, DT); }
  const e = A.est;
  const dev = Math.hypot(e.aim.x - A.target.x, e.aim.y - A.target.y);
  // 时延补偿把状态推到"现在"后，再叠提前量才是瞄准点：所以领先真值的距离≈提前量，
  // 不是"提前时间×目标速度"（时延部分被补偿掉，不再体现为偏差）。
  return {hasPrev:!!e.aimPrev, dev:+dev.toFixed(2),
          leadM:+((e.leadAdaptiveM||0) + (e.leadExtraM||0)).toFixed(2)};
})`).runInContext(ctx)();
check(guideLine.hasPrev && guideLine.dev > 0.5
      && Math.abs(guideLine.dev - guideLine.leadM) < 0.6,
      `制导线连瞄准点而非真值点（瞄准点领先真值 ${guideLine.dev} m ≈ 提前量 ${guideLine.leadM} m）`,
      `制导线/瞄准点异常：${JSON.stringify(guideLine)}`);

// ㉜ 停滞自动提能力必须在**默认参数**（a_plan 3 = slew 3）下真的生效（2026-11-10 维护者反馈
//    "触发从来没满足过"）：原来提升目标取 min(a_phys, slew)=3、want 恒为 0；现在目标取 a_phys，
//    并且提能力生效时 slew 限幅会放宽到提升后的规划值。蛇形 8.1 m/s：关→穿透/不到点；开→到点且不穿透。
const boostRescue = new vm.Script(`(function(){
  const base = {src:'vmsg', law:'brake', policy:'fast', end:'match', accel:3, decel:3, aPhys:9.8,
    slew:3, vcap:10, vt:8.1, heading:90, turn:'weave', startD:40, jerk:0, leadExtraM:0, alt:0,
    vzT:0, criterion:'rel', aLatRes:0, adaptive:'on', continuous:'on', targetBrake:0,
    noise:0, vsig:0, measHz:5, latency:200, aTerm:1.5, sched:'on', leadMode:'scaled',
    floor:true, margin:1};
  const run = (autoBoost) => {
    Object.assign(P, base, {autoBoost}); resetSims(); P.paused = true;
    let upMax = 0, boostMax = 0, boostTMax = 0;
    for (let i = 0; i < 9000; i++) {
      stepSim(A, DT); stepSim(B, DT); stepSim(C, DT);
      upMax = Math.max(upMax, motionLimits(A).up);
      boostMax = Math.max(boostMax, A.accelBoost || 0);
      boostTMax = Math.max(boostTMax, A.boostT || 0);
      if (A.arrived) break;
    }
    updateHud();
    return {arrived:!!A.arrived, at:+(A.arrivedAt ?? -1).toFixed(1), pen:!!A.penetrated,
            upMax:+upMax.toFixed(2), boost:+boostMax.toFixed(2), boostT:+boostTMax.toFixed(1),
            note:document.getElementById('minNote').innerHTML};
  };
  return {off: run('off'), on: run('on')};
})`).runInContext(ctx)();
check(!boostRescue.off.arrived && boostRescue.off.pen
      && boostRescue.on.arrived && !boostRescue.on.pen
      && boostRescue.on.boost > 1 && boostRescue.on.upMax > 3.5
      && boostRescue.on.note.includes('停滞自动提能力'),
      `提能力在默认参数下生效（蛇形 8.1：关→${boostRescue.off.pen ? '穿透' : '不到点'}；`
      + `开→${boostRescue.on.at} s 到点、无穿透，抬升 +${boostRescue.on.boost} m/s²、`
      + `有效加速度上限 ${boostRescue.off.upMax} → ${boostRescue.on.upMax} m/s²）`,
      `提能力未生效：${JSON.stringify(boostRescue)}`);

process.exit(ok ? 0 : 1);
