/* ==========================================================================
 * 联机注入_light.js — 1928_front_light（细化前端，19 轴）联机扩展
 *
 * 与 html_model/联机注入.js 的关系：
 *   旧注入假设「工程轴数 == 协议帧轴数 == 21」，并按 slides[i] 下标绑定控件；
 *   细化工程是 19 轴（3 根 E_PIN 合成 1 根 WS_EPin），因此旧脚本 boot 里
 *   `project.axes.length !== FRAME_AXIS_IDS.length` 会直接 return 并无限重试
 *   —— 表现就是「页面能开、永远不连」。
 *
 *   本脚本把「协议帧（21 槽）」与「前端的轴（19 根）」拆成两层：
 *     - 帧侧：顺序/单位沿用《前后端数据协议_V1》（直线 mm、旋转 rad、108B）；
 *     - 前端侧：按 MAP 表投影，每根轴带 sign / offset / scale / clamp，
 *       旋转轴在此边界做 rad↔deg（TwinDOM 模型内部是 deg）。
 *
 * 后端：light_test/backend（旁路副本，默认 9030）。可用 ?ws= 覆盖端口。
 * 依赖：light_test/tools/build_light_online.py 注入的 window.__TD_VIZ__ 钩子
 *   {scene, axes, controls, setAxis, obj3d, defs, Box3, V3, project}
 * ========================================================================== */
(function () {
  'use strict';

  var DEFAULT_WS = 'ws://127.0.0.1:9030/ws';
  var FRAME_LEN = 108;
  var FRAME_MAGIC = 0xA1;
  var FRAME_VERSION = 1;
  var FRAME_N_AXIS = 21;
  var EXTRAPOLATE_MAX_S = 0.02;
  var RECONNECT_MS = 1000;
  var MOVE_THROTTLE_MS = 33;
  var RAD2DEG = 180 / Math.PI;
  var DEG2RAD = Math.PI / 180;

  var WS_URL = (function () {
    var m = /[?&]ws=(\d+)/.exec(location.search);
    return m ? 'ws://127.0.0.1:' + m[1] + '/ws' : DEFAULT_WS;
  })();

  /* ---------- 投影表：帧槽 → 细化前端轴 ----------
   * light : 细化工程轴 id（null = 该槽在模型上没有对应，丢弃）
   * rot   : 帧里是 rad、模型要 deg
   * sign  : 仿真正方向 → 模型正方向（标定得出，见 tools/fit_light_axes.py）
   * offset: 零位差（模型值 = 仿真值 × scale × sign + offset）
   * scale : 量程比（默认 1；E_PIN 三合一另配）
   * combine: 多个槽合并时取 'mean' | 'lead' | 'span'
   */
  var MAP = [
    { light: 'axis_WS_X',            sim: 'WS_LS_X',    rot: false, sign: +1, offset: 0 },
    { light: 'axis_WS_Y',            sim: 'WS_LS_Y',    rot: false, sign: +1, offset: 0 },
    { light: 'axis_WS_RZ',           sim: 'WS_Rz',      rot: true,  sign: +1, offset: 0 },
    { light: 'axis_WS_EPIN_Z',       sim: 'E_EPIN_Z',   rot: false, sign: +1, offset: 0,
      combine: 'mean', scale: 13 / 30 },
    { light: 'axis_WS_EPIN_Z',       sim: 'E_EPIN_Z',   rot: false, sign: +1, offset: 0,
      combine: 'mean', scale: 13 / 30 },
    { light: 'axis_WS_EPIN_Z',       sim: 'E_EPIN_Z',   rot: false, sign: +1, offset: 0,
      combine: 'mean', scale: 13 / 30 },
    { light: 'axis_BS_Z',            sim: 'BS_Arm_Z',   rot: false, sign: +1, offset: 0 },
    { light: 'axis_BS_Rx',           sim: 'BS_CAL_Rx',  rot: true,  sign: +1, offset: 0 },
    { light: 'axis_BS_Ry',           sim: 'BS_CAL_Ry',  rot: true,  sign: +1, offset: 0 },
    { light: 'axis_AS_X',            sim: 'AS_X',       rot: false, sign: +1, offset: 0 },
    { light: 'axis_AS_Y',            sim: 'AS_Y',       rot: false, sign: +1, offset: 0 },
    { light: 'axis_AS_Z',            sim: 'AS_Z',       rot: false, sign: +1, offset: 0 },
    { light: 'axis_AS_Rx',           sim: 'AS_Rx',      rot: true,  sign: +1, offset: 0 },
    { light: 'axis_AS_Ry',           sim: 'AS_Ry',      rot: true,  sign: +1, offset: 0 },
    { light: 'axis_ISS_FA_X',        sim: 'FA_X',       rot: false, sign: +1, offset: 0 },
    { light: 'axis_ISS_FA_Z',        sim: 'FA_Z',       rot: false, sign: -1, offset: 0 },
    { light: 'axis_ISS_FA_RX',       sim: 'FA_Rx',      rot: true,  sign: -1, offset: 0 },
    { light: 'axis_ISS_FA_RZ',       sim: 'CH_Rz',      rot: true,  sign: +1, offset: 0 },
    { light: 'axis_filmstage_x_0001', sim: 'ES_X',      rot: false, sign: +1, offset: 0 },
    { light: 'axis_filmstage_y_0001', sim: 'ES_Y',      rot: false, sign: +1, offset: 0 },
    { light: 'axis_filmpin_z_0001',  sim: 'ES_PunchZ',  rot: false, sign: +1, offset: 0 }
  ];

  /* ---------- 帧解码 ---------- */
  function decodeFrame(buf) {
    var dv = new DataView(buf);
    if (dv.byteLength !== FRAME_LEN) return null;
    if (dv.getUint8(0) !== FRAME_MAGIC) return null;
    if (dv.getUint8(1) !== FRAME_VERSION) return null;
    if (dv.getUint16(2, true) !== FRAME_N_AXIS) return null;
    var values = new Float64Array(FRAME_N_AXIS);
    for (var i = 0; i < FRAME_N_AXIS; i++) values[i] = dv.getFloat32(24 + i * 4, true);
    return {
      simTime: dv.getFloat64(4, true),
      wallTime: dv.getFloat64(12, true),
      seq: dv.getUint32(20, true),
      values: values
    };
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { decodeFrame: decodeFrame, MAP: MAP, FRAME_LEN: FRAME_LEN };
  }
  if (typeof window === 'undefined' || typeof document === 'undefined') return;

  /* ---------- 错误捕获 ---------- */
  var scriptError = '', scriptErrorCount = 0;
  function reportError(ctx, e) {
    scriptErrorCount++;
    scriptError = ctx + ': ' + ((e && (e.message || e.reason || e)) + '') +
      (scriptErrorCount > 1 ? ' (x' + scriptErrorCount + ')' : '');
    try { console.error('[联机light] ' + ctx, e); } catch (_) {}
    try { updateHud(); } catch (_) {}
  }
  function safe(ctx, fn) {
    return function () {
      try { return fn.apply(this, arguments); } catch (e) { reportError(ctx, e); }
    };
  }
  window.addEventListener('error', function (ev) {
    if (ev && ev.message && ev.message !== 'Script error.') {
      reportError('window', ev.message + (ev.lineno ? ' @' + ev.lineno : ''));
    }
  });

  /* ---------- 运行时状态 ---------- */
  var hook = null;          // window.__TD_VIZ__
  var axesById = {};        // 细化工程轴 id -> axis 定义
  var controls = {};        // 细化工程轴 id -> {input, renderValue}
  var slots = [];           // 帧槽 -> {axis, spec}
  var dragging = {};        // 轴 id -> 正在拖
  var overflow = {};        // 轴 id -> 被夹取的次数
  var ws = null, connected = false;
  var lastFrame = null, prevFrame = null, lastSeq = -1;
  var droppedFrames = 0, frameArrivals = [], latencySamples = [];

  /* ---------- 值换算 ---------- */
  function comboValue(group, values) {
    // 同一 light 轴由多个槽喂（EPIN×3）：mean / lead / span
    var mode = group[0].spec.combine || 'lead';
    var list = group.map(function (g) { return g.idx; });
    if (mode === 'mean') {
      var sum = 0;
      for (var i = 0; i < list.length; i++) sum += values[list[i]];
      return sum / list.length;
    }
    if (mode === 'span') {
      var mn = Infinity, mx = -Infinity;
      for (var j = 0; j < list.length; j++) {
        var v = values[list[j]];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }
      return mx - mn;
    }
    return values[list[0]];
  }

  function toModel(spec, wireValue) {
    var v = spec.rot ? wireValue * RAD2DEG : wireValue;
    return v * (spec.scale == null ? 1 : spec.scale) * spec.sign + (spec.offset || 0);
  }
  function toWire(spec, modelValue) {
    var v = (modelValue - (spec.offset || 0)) / (spec.scale == null ? 1 : spec.scale) * spec.sign;
    return spec.rot ? v * DEG2RAD : v;
  }
  function clampAxis(axis, v) {
    var lo = Number(axis.min), hi = Number(axis.max);
    if (!isFinite(lo) || !isFinite(hi)) return v;
    if (v < lo) { countOverflow(axis.id); return lo; }
    if (v > hi) { countOverflow(axis.id); return hi; }
    return v;
  }
  var overflowLast = 0;
  function countOverflow(id) {
    overflow[id] = (overflow[id] || 0) + 1;
    overflowLast = performance.now() / 1000;
  }

  /* ---------- 应用一帧到模型 ---------- */
  function applyFrame(values, extrapDt) {
    for (var i = 0; i < slots.length; i++) {
      var slot = slots[i];
      if (!slot || !slot.axis) continue;
      if (dragging[slot.axis.id]) continue;
      var wire = slot.value(values);
      if (extrapDt > 0) wire = slot.extrapolated(values, extrapDt);
      var v = clampAxis(slot.axis, toModel(slot.spec, wire));
      setAxisValue(slot, v);
    }
  }

  function setAxisValue(slot, v) {
    var axis = slot.axis;
    var ctl = controls[axis.id];
    // 细化查看器带「接触传动 / 弹簧回位」层：直接改 axis.current（或 setAxis 第三参
    // 为 true）会在下一帧被动力学弹回；滑块 oninput 才是运行时的正规应用链。
    // 因此统一走 input.value → origOninput，与用户拖动完全同一条路径。
    if (ctl && ctl.input) {
      if (Math.abs(Number(ctl.input.value) - v) < 1e-9) return;
      ctl.input.value = v;                       // 浏览器按 min/max/step 夹取
      if (typeof ctl.origOninput === 'function') ctl.origOninput.call(ctl.input);
      return;
    }
    if (Math.abs((axis.current || 0) - v) < 1e-9) return;
    hook.setAxis(axis, v, false);
  }

  /* ---------- boot ---------- */
  var bootTries = 0;
  function boot() {
    hook = window.__TD_VIZ__;
    if (!hook || !Array.isArray(hook.axes) || !hook.setAxis) {
      if (++bootTries % 20 === 1) {
        console.warn('[联机light] 等待 __TD_VIZ__ 钩子（页面需由 tools/build_light_online.py 生成）');
      }
      return setTimeout(boot, 100);
    }
    var i;
    for (i = 0; i < hook.axes.length; i++) axesById[hook.axes[i].id] = hook.axes[i];
    if (hook.controls && hook.controls.forEach) {
      hook.controls.forEach(function (v, k) { controls[k] = v; });
    }

    // 建立帧槽 → light 轴（同轴多槽时先聚合 specs）
    var byLight = {};
    for (i = 0; i < MAP.length; i++) {
      var spec = MAP[i], axis = axesById[spec.light];
      if (!axis) continue;                          // 模型缺该轴 → 该槽丢弃
      (byLight[spec.light] = byLight[spec.light] || []).push({ idx: i, spec: spec });
    }
    slots = new Array(FRAME_N_AXIS);
    Object.keys(byLight).forEach(function (lightId) {
      var group = byLight[lightId];
      var axis = axesById[lightId];
      var first = group[0].spec;
      var idxs = group.map(function (g) { return g.idx; });
      slots[group[0].idx] = {
        axis: axis,
        spec: first,
        group: group,
        value: function (values) {
          if (group.length === 1) return values[idxs[0]];
          return comboValue(group, values);
        }
      };
    });
    // 清理：只保留组内第一个槽负责应用（其余槽位不再重复驱动同一根轴）
    Object.keys(byLight).forEach(function (lightId) {
      var group = byLight[lightId];
      if (group.length > 1) {
        // 取第一槽负责写入，其余置空
        for (var k = 1; k < group.length; k++) slots[group[k].idx] = null;
      }
    });

    var mapped = Object.keys(byLight).length;
    var missing = MAP.filter(function (s) { return !axesById[s.light]; })
      .map(function (s) { return s.light; });

    bindSliders();
    initHud();
    updateHud();
    console.log('[联机light] 就绪：模型轴 ' + hook.axes.length + '，投影 ' + mapped +
      ' 根' + (missing.length ? '，缺轴 ' + missing.join(',') : ''));
    connect();
    requestAnimationFrame(renderLoop);
  }

  /* ---------- 滑块联动 ---------- */
  function bindSliders() {
    Object.keys(controls).forEach(function (id) {
      var ctl = controls[id];
      if (!ctl || !ctl.input) return;
      ctl.origOninput = ctl.input.oninput;
      ctl.input.step = 'any';
      ctl.input.addEventListener('pointerdown', function () { dragging[id] = true; });
      window.addEventListener('pointerup', function () { dragging[id] = false; });
    });
  }
  var pendingMove = {};
  function setSliderOnlineMode(online) {
    Object.keys(controls).forEach(function (id) {
      var ctl = controls[id];
      if (!ctl || !ctl.input) return;
      if (!online) { ctl.input.oninput = ctl.origOninput; return; }
      ctl.input.oninput = safe('slider.oninput', function () {
        var slot = slotOfAxis(id);
        if (!slot) return;
        var v = Number(ctl.input.value);
        var now = performance.now();
        var send = function () {
          ctl.lastMoveSent = performance.now();
          sendCmd({ cmd: 'move', axis: slot.spec.sim, target: toWire(slot.spec, v) });
        };
        if (now - (ctl.lastMoveSent || 0) >= MOVE_THROTTLE_MS) send();
        else {
          if (pendingMove[id]) clearTimeout(pendingMove[id]);
          pendingMove[id] = setTimeout(safe('slider.pending', send), MOVE_THROTTLE_MS);
        }
      });
    });
  }
  function slotOfAxis(id) {
    for (var i = 0; i < slots.length; i++) if (slots[i] && slots[i].axis.id === id) return slots[i];
    return null;
  }

  /* ---------- WebSocket ---------- */
  function connect() {
    try { ws = new WebSocket(WS_URL); } catch (e) { reportError('connect', e); return setTimeout(connect, RECONNECT_MS); }
    ws.binaryType = 'arraybuffer';
    ws.onopen = safe('ws.onopen', function () {
      connected = true; lastSeq = -1;
      setSliderOnlineMode(true);
      updateHud();
    });
    ws.onmessage = safe('ws.onmessage', function (ev) {
      if (typeof ev.data === 'string') {
        try {
          var ack = JSON.parse(ev.data);
          if (ack && ack.ok === false) { lastReject = { axis: ack.axis || '', error: String(ack.error || ''), t: performance.now() / 1000 }; }
        } catch (_) {}
        return;
      }
      var f = decodeFrame(ev.data);
      if (!f) return;
      var now = performance.now() / 1000;
      prevFrame = lastFrame;
      lastFrame = { tRecv: now, values: f.values };
      if (lastSeq >= 0 && f.seq > lastSeq + 1) droppedFrames += f.seq - lastSeq - 1;
      lastSeq = f.seq;
      frameArrivals.push(now);
      while (frameArrivals.length && frameArrivals[0] < now - 1) frameArrivals.shift();
      latencySamples.push((Date.now() / 1000 - f.wallTime) * 1000);
      if (latencySamples.length > 100) latencySamples.shift();
    });
    ws.onclose = safe('ws.onclose', function () {
      connected = false; setSliderOnlineMode(false); updateHud();
      setTimeout(connect, RECONNECT_MS);
    });
    ws.onerror = function () {};
  }
  function sendCmd(obj) {
    if (connected && ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }
  var lastReject = null;

  /* ---------- 渲染循环：线性外推 ---------- */
  var fpsCount = 0, fpsTime = 0, renderFps = 0, hudTimer = 0;
  function renderLoop() {
    requestAnimationFrame(renderLoop);
    try { renderTick(); } catch (e) { reportError('renderLoop', e); }
  }
  function renderTick() {
    var now = performance.now() / 1000;
    fpsCount++;
    if (now - fpsTime >= 1) { renderFps = fpsCount; fpsCount = 0; fpsTime = now; }
    if (connected && lastFrame && !window.__LIGHT_CALIB__) {
      // window.__LIGHT_CALIB__ = true 时让出驱动权（标定/量几何脚本独占轴）
      var extrap = Math.min(Math.max(now - lastFrame.tRecv, 0), EXTRAPOLATE_MAX_S);
      var dt = prevFrame ? lastFrame.tRecv - prevFrame.tRecv : 0;
      var dtExtrap = (dt > 1e-6 && extrap > 0) ? extrap : 0;
      for (var i = 0; i < slots.length; i++) {
        var slot = slots[i];
        if (!slot || !slot.axis || dragging[slot.axis.id]) continue;
        var wire = slot.value(lastFrame.values);
        if (dtExtrap > 0) {
          wire += (slot.value(lastFrame.values) - slot.value(prevFrame.values)) / dt * dtExtrap;
        }
        setAxisValue(slot, clampAxis(slot.axis, toModel(slot.spec, wire)));
      }
    }
    if (now - hudTimer >= 0.25) { hudTimer = now; updateHud(); }
  }

  /* ---------- HUD ---------- */
  var hudEl = null;
  function initHud() {
    hudEl = document.createElement('div');
    hudEl.style.cssText =
      'position:fixed;right:12px;bottom:12px;z-index:9999;padding:10px 12px;' +
      'border-radius:8px;min-width:230px;background:rgba(15,25,40,.85);color:#e7eef8;' +
      'border:1px solid #30445d;font:12px/1.7 Consolas,Microsoft YaHei,monospace;' +
      'pointer-events:none;white-space:pre';
    hudEl.id = 'lightHud';
    document.body.appendChild(hudEl);
  }
  function p50(arr) {
    if (!arr.length) return null;
    var s = arr.slice().sort(function (a, b) { return a - b; });
    return s[Math.floor((s.length - 1) / 2)];
  }
  function overflowSummary() {
    var ids = Object.keys(overflow);
    if (!ids.length) return '无';
    if (performance.now() / 1000 - overflowLast > 6) return '无';
    var parts = ids.map(function (k) { return k.replace(/^axis_/, '') + '×' + overflow[k]; });
    var s = parts.join(' ');
    return s.length > 40 ? s.slice(0, 40) + '…' : s;
  }
  function updateHud() {
    if (!hudEl) return;
    var lat = p50(latencySamples);
    var nowS = performance.now() / 1000;
    var rejectFresh = lastReject && (nowS - lastReject.t) < 6;
    var lines = [
      '模型轴     ' + (hook ? hook.axes.length : 0) + ' 根（投影 ' + slots.filter(Boolean).length + ' 槽）',
      '联机状态   ' + (connected ? '● 已连接 ' + WS_URL.replace('ws://', '') : '○ 未连接（1s 重连）'),
      '推送频率   ' + (connected ? frameArrivals.length + ' Hz' : '—'),
      '单向延迟   ' + (lat !== null ? lat.toFixed(2) + ' ms (p50)' : '—'),
      '丢帧       ' + droppedFrames,
      '渲染 fps   ' + renderFps,
      '量程夹取   ' + overflowSummary(),
      '指令回执   ' + (rejectFresh ? '✗ ' + (lastReject.axis || '') + ' ' + lastReject.error : '无'),
      '脚本错误   ' + (scriptError || '无')
    ];
    hudEl.textContent = lines.join('\n');
    hudEl.style.borderColor = !connected ? '#c0392b' : (rejectFresh ? '#e67e22' : '#2fbf71');
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
