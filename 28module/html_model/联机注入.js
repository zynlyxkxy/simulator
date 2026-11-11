/* ==========================================================================
 * 联机注入.js — TwinDOM 便携查看器联机扩展（源文件）
 * 构建：backend/tools/build_online_html.py 把本文件整块内联进
 *       d2w_online.html（不再用 <script src> 引用）——
 *       file:// 页面用 src 引外部脚本属跨域（opaque origin），异常会被
 *       浏览器洗成 "Script error."，内联后 window.onerror 能拿到真实消息。
 * 健壮性：boot / WS 回调 / renderLoop 全部 try/catch，异常只写 console
 *       和 HUD（"脚本错误"行），绝不冒泡到查看器的 window.onerror 弹窗。
 *
 * 机制（逆向自压缩运行时）：
 *  - window.__TWIN_DOM_PROJECT__ 即运行时内部项目对象 Mn（同一引用），
 *    axes[i].current 为轴当前值；
 *  - 左下角轴面板按 Mn.axes 顺序生成 21 个 input[type=range] 滑块，
 *    滑块的 oninput 属性处理器 = "写 axis.current → nu(重算位姿) → 刷新数值显示"；
 *  - nu / Oc / Xf 等运行时符号在 IIFE 闭包内，外部不可直接调用，
 *    因此本脚本通过"设 slider.value + 直接调用原 oninput 处理器"来复用
 *    运行时的完整应用链（等价于用户拖滑块），无需修改运行时源码。
 * 协议：《前后端数据协议_V1.md》—— ws://127.0.0.1:9010/ws，
 * 108 字节定长小端二进制帧 500Hz 推 21 轴值（直线 mm / 旋转 rad）；
 * 文本 JSON 下发指令。模型内部旋转轴为 deg，rad↔deg 换算在本脚本边界完成。
 * ======================================================================== */
(function () {
  'use strict';

  /* ---------- 协议常量 ---------- */
  var WS_URL = 'ws://127.0.0.1:9010/ws';
  var FRAME_LEN = 108;
  var FRAME_MAGIC = 0xA1;
  var FRAME_VERSION = 1;
  var EXTRAPOLATE_MAX_S = 0.02;      // 线性外推上限 20ms
  var RECONNECT_MS = 1000;
  var MOVE_THROTTLE_MS = 33;         // 联机滑块 move 指令节流 ~30Hz

  // 协议 §3 固定轴顺序：帧内 index → 前端轴 id
  var FRAME_AXIS_IDS = [
    'axis_mssl5svn_ols740',  // 0  WS_LS_X
    'axis_msslggo6_6d6mzm',  // 1  WS_LS_Y
    'axis_msslrthn_qbthvp',  // 2  WS_Rz
    'axis_epin1_z_0001',     // 3  E_EPIN_Z_1
    'axis_epin2_z_0001',     // 4  E_EPIN_Z_2
    'axis_epin3_z_0001',     // 5  E_EPIN_Z_3
    'axis_mssq9yhm_gmwadb',  // 6  BS_Arm_Z
    'axis_mssrgvw5_7ds4bo',  // 7  BS_CAL_Rx
    'axis_mssrheu1_cfw18w',  // 8  BS_CAL_Ry
    'axis_msst4og7_eby5r6',  // 9  AS_X
    'axis_msstd5bj_poz6p6',  // 10 AS_Y
    'axis_alignz_0001',      // 11 AS_Z
    'axis_mssu8pry_ijkgc7',  // 12 AS_Rx
    'axis_mssu8zgl_vi1rbv',  // 13 AS_Ry
    'axis_mswkylsp_nmcvcb',  // 14 FA_X
    'axis_mswlez0m_3z4lwg',  // 15 FA_Z
    'axis_mswlx2e6_v4wbj4',  // 16 FA_Rx
    'axis_mswlzliz_t1ybur',  // 17 CH_Rz
    'axis_filmstage_x_0001', // 18 ES_X
    'axis_filmstage_y_0001', // 19 ES_Y
    'axis_filmpin_z_0001'    // 20 ES_PunchZ
  ];

  // 协议 V1 起帧中旋转轴单位为 rad，TwinDOM 模型内部为 deg，边界换算在此。
  // 帧 index → 是否旋转轴（协议 §3 单位列 = rad 的 7 根）
  var ROTARY_FRAME_INDEX = { 2: 1, 7: 1, 8: 1, 12: 1, 13: 1, 16: 1, 17: 1 };
  var RAD2DEG = 180 / Math.PI;
  var DEG2RAD = Math.PI / 180;
  // 帧值（直线 mm / 旋转 rad）→ 模型原生值（直线 mm / 旋转 deg）
  function frameToModel(idx, v) { return ROTARY_FRAME_INDEX[idx] ? v * RAD2DEG : v; }
  // 模型原生值 → 线路上值（move 指令 target，旋转轴发 rad）
  function modelToWire(idx, v) { return ROTARY_FRAME_INDEX[idx] ? v * DEG2RAD : v; }

  /* ---------- 帧解码（独立导出便于 node 单测） ---------- */
  function decodeFrame(buf) {
    var dv = new DataView(buf);
    if (dv.byteLength !== FRAME_LEN) return null;
    if (dv.getUint8(0) !== FRAME_MAGIC) return null;
    if (dv.getUint8(1) !== FRAME_VERSION) return null;
    var count = dv.getUint16(2, true);
    if (count !== FRAME_AXIS_IDS.length) return null;
    var values = new Float64Array(count);
    for (var i = 0; i < count; i++) values[i] = dv.getFloat32(24 + i * 4, true);
    return {
      simTime: dv.getFloat64(4, true),
      wallTime: dv.getFloat64(12, true),
      seq: dv.getUint32(20, true),
      values: values
    };
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { decodeFrame: decodeFrame, FRAME_AXIS_IDS: FRAME_AXIS_IDS, FRAME_LEN: FRAME_LEN };
  }
  if (typeof window === 'undefined' || typeof document === 'undefined') return; // node 单测环境到此为止

  /* ---------- 错误捕获：只写 console + HUD，不冒泡 ---------- */
  var scriptError = '';   // 最近一次脚本错误（真实 message），HUD 展示
  var scriptErrorCount = 0;
  function reportError(ctx, e) {
    scriptErrorCount++;
    var msg = (e && (e.message || e.reason || e)) + '';
    scriptError = ctx + ': ' + msg + (scriptErrorCount > 1 ? ' (x' + scriptErrorCount + ')' : '');
    try { console.error('[联机] ' + ctx, e); } catch (_) {}
    try { updateHud(); } catch (_) {}
  }
  function safe(ctx, fn) {
    return function () {
      try { return fn.apply(this, arguments); } catch (e) { reportError(ctx, e); }
    };
  }
  // 内联后能拿到真实 message；捕获一次存 HUD 便于诊断（含查看器自身报错）
  window.addEventListener('error', function (ev) {
    if (ev && ev.message && ev.message !== 'Script error.') {
      scriptErrorCount++;
      scriptError = 'window: ' + ev.message +
        (ev.lineno ? ' @' + ev.filename + ':' + ev.lineno : '');
      try { updateHud(); } catch (_) {}
    }
  });

  /* ---------- 等待查看器 UI 就绪 ---------- */
  // 轴表提升到 IIFE 作用域：renderTick / setSliderOnlineMode 也要访问，
  // 若为 bootInner 局部变量，渲染循环会 ReferenceError
  var axesById = {};
  function boot() {
    try {
      bootInner();
    } catch (e) {
      reportError('boot', e);
    }
  }
  function bootInner() {
    var project = window.__TWIN_DOM_PROJECT__;
    var sliders = document.querySelectorAll('input[type=range]');
    if (!project || !Array.isArray(project.axes) ||
        project.axes.length !== FRAME_AXIS_IDS.length ||
        sliders.length !== project.axes.length) {
      return setTimeout(boot, 100); // 运行时尚未构建完面板，重试
    }

    // 轴表：id → {axis, slider, origOninput, dragging, lastMoveSent, pendingMove}
    axesById = {};
    for (var i = 0; i < project.axes.length; i++) {
      (function (axis, input) {
        axesById[axis.id] = {
          axis: axis,
          input: input,
          frameIndex: FRAME_AXIS_IDS.indexOf(axis.id), // 协议帧内序号，-1 表示不在帧中
          origOninput: input.oninput, // 运行时应用链：写 current → nu() → 刷新显示
          dragging: false,
          lastMoveSent: 0,
          pendingMove: null
        };
        input.addEventListener('pointerdown', function () { axesById[axis.id].dragging = true; });
        window.addEventListener('pointerup', function () { axesById[axis.id].dragging = false; });
        // 运行时把 step 硬编码为 0.1，range input 会把 value 吸附到步进网格，
        // 对 ±0.1 的微调旋转轴等于锁死；改为 any 关闭吸附（单机拖动也更细）
        input.step = 'any';
      })(project.axes[i], sliders[i]);
    }
    // AS_Y 行程下限覆盖：与后端 axes_config 保持一致（-30）
    var asYRec = axesById['axis_msstd5bj_poz6p6'];
    if (asYRec && asYRec.axis) {
      asYRec.axis.min = -30;
      asYRec.input.min = -30;
    }
    // 校验帧顺序表里的 id 都存在于工程
    var missing = FRAME_AXIS_IDS.filter(function (id) { return !axesById[id]; });
    if (missing.length) console.warn('[联机] 工程中缺少轴 id:', missing);

    initHud();
    initViz();
    connect();
    requestAnimationFrame(renderLoop);
  }

  /* ---------- 真空/工件可视化（/api/viz 10Hz 轮询） ----------
   * scene 接入点：联机版 html 由 tools/build_online_html.py 生成，构建时在
   * 查看器运行时尾部注入 window.__TD_VIZ__ = {scene, make, obj3d, defs, V3, Box3}
   * （scene=THREE.Scene；obj3d=对象id→Object3D；make=工程对象定义→Object3D 工厂；
   *   V3=Vector3、Box3=Box3 类）。拿到钩子→场景内可视化（吸盘绿色发光 + 工件网格）；
   * 拿不到（未构建页面等）→ 退化为 HUD 下方状态灯带。viz 断线 3 次→全部熄灭/恢复。 */
  var HTTP_BASE = (function () {
    var m = /[?&]api=([^&]+)/.exec(location.search);
    if (m) return decodeURIComponent(m[1]).replace(/\/$/, '');
    return (location.protocol.indexOf('http') === 0 && location.host === '127.0.0.1:9010')
           ? '' : 'http://127.0.0.1:9010';
  })();

  // 吸盘：/api/viz vacuum 键名关键词 → 工程对象名（name 匹配 Mn.objects）
  // keys 含后端 /api/viz 的 vacuum 键名（精确）与中文别名；映射假设见注释
  var VIZ_CUPS = [
    { keys: ['WT_WaferVac', '晶圆吸盘', 'wafer_chuck', 'waferchuck'], objName: '晶圆吸盘' },                 // WT-晶圆吸附
    { keys: ['WT_BondingVac_1', 'WT_BondingVac_2', 'WT_BondingVac_3', 'ChipVac', '键合头', 'bond_head'], objName: '键合头吸盘' }, // 键合头三吸盘+芯片吸附
    { keys: ['ChuckVac', '芯片吸盘', 'die_chuck', 'diechuck'],       objName: '芯片吸盘' },                 // ChuckVac=翻转吸盘（假设，待真机确认）
    { keys: ['WT_ChipVac', '蓝膜', 'film'],                          objName: '蓝膜台圆形台体' }            // WT-芯片吸附=蓝膜侧（假设）
  ];
  // die 载体 → 载体对象名（pos 为载体局部坐标 mm，rz 为 rad）
  var VIZ_CARRIERS = { film: '蓝膜', flip: '芯片吸盘', head: '键合头吸盘', wafer: '晶圆吸盘' };
  // 光源/相机：/api/io 观察点 → 工程对象名（name 匹配 Mn.objects，找不到则跳过不报错）
  var VIZ_LIGHTS = [
    { io: 'SystemMachine.SS_Bonding.M_AS.oAS_LightOn',       objName: '粗对准相机' },     // 粗对准光源
    { io: 'SystemMachine.SS_Bonding.M_AS.oLightOn_1',        objName: '上相机镜头' },     // 对准光源1
    { io: 'SystemMachine.SS_Bonding.M_AS.oLightOn_2',        objName: '下相机镜头' },     // 对准光源2
    { io: 'SystemMachine.SS_Bonding.M_AS.oAS_CameraOn',      objName: '粗对准相机镜头' }  // 粗对准相机触发
  ];
  // 共焦光束：probe → 工程对象名（传感器本体）+ 出射方向（本地 Z 向）
  var VIZ_CONFOCAL = [
    { probe: 'down', objName: '下光谱共焦传感器', dir: -1 },
    { probe: 'up',   objName: '上光谱共焦传感器', dir:  1 }
  ];
  // 工件几何参数：默认值 = 默认处方（处方未取到时的兜底）；
  // 处方轮询（RECIPE_POLL_MS）签名变化时按 recipe 重建 wafer / die / die map
  var vizGeom = { waferR: 150, waferT: 1.0, dieSX: 4.0, dieSY: 4.0, dieT: 0.5 };
  var WAFER_COLOR = '#5F9FB5', WAFER_OPACITY = 0.68;  // 半透明晶圆（参考样式）
  var NOTCH_HALF = 0.052, NOTCH_DEPTH = 8;            // 三角缺口 rad / mm（朝 -X）
  var RECIPE_POLL_MS = 2000;                          // 处方签名轮询 0.5Hz

  var viz = { hook: null, fails: 0, polling: false, active: false,
              cups: [], waferMesh: null, waferBase: null, dieMapMesh: null,
              waferMarksMesh: null,
              dieMeshes: {}, strip: null, stripCells: {},
              ctors: null, ctorsReady: false, dieAssets: null,
              recipeSig: '', recipePolling: false,
              sitesCache: null, sitesFetching: false,
              bondedLocal: [], bondedSig: '',
              heaterDisc: null,
              filmTopLocalZ: null,
              siteDieSX: 4.0, siteDieSY: 4.0,
              // 标记：默认值 = 默认处方（film.marks 比例标记 / wafer.marks 晶圆级）
              filmMarks: { shape: 'box', size_mm: 0.15,
                           ratio: [[-0.76, -0.76], [0.76, -0.76],
                                   [-0.76, 0.76], [0.76, 0.76]],
                           per_die: 4 },
              waferMarks: [] };

  function initViz() {
    viz.hook = window.__TD_VIZ__ || null;
    if (viz.hook) {
      // 预解析吸盘/载体对象（工程对象名 → Object3D）
      VIZ_CUPS.forEach(function (c) { c.obj = findObject3D(c.objName); });
      viz.carrierObjs = {};
      Object.keys(VIZ_CARRIERS).forEach(function (k) {
        viz.carrierObjs[k] = findObject3D(VIZ_CARRIERS[k]);
      });
      // 蓝膜顶面在“蓝膜”局部系的 z（刚性部件解一次即可；film die 贴面浮起用）
      var filmObj = viz.carrierObjs.film;
      if (filmObj) {
        var fb = new viz.hook.Box3().setFromObject(filmObj);
        var fc = fb.getCenter(new viz.hook.V3());
        viz.filmTopLocalZ = filmObj.worldToLocal(new viz.hook.V3(fc.x, fc.y, fb.max.z)).z;
      }
      VIZ_LIGHTS.forEach(function (L) { L.obj = findObject3D(L.objName); });
      VIZ_CONFOCAL.forEach(function (C) {
        C.obj = findObject3D(C.objName);
        C.beam = ensureConfocalBeam(C.obj, C.dir);
      });
      // 脉冲加热器发光圆盘：挂键合头吸盘上方（随键合头轴动）
      viz.heaterDisc = ensureHeaterDisc(findObject3D('键合头吸盘'));
      setInterval(safe('viz.io', pollVizIO), 500);   // 光源/相机观察点 2Hz
      setInterval(safe('viz.recipe', pollRecipe), RECIPE_POLL_MS);  // 处方签名 0.5Hz
      pollRecipe();                                  // 立即取一次，尽早按处方重建
    } else {
      buildVizStrip();
      console.warn('[联机] 未找到 __TD_VIZ__ 场景钩子，真空/工件用状态灯带兜底');
    }
    setInterval(safe('viz.poll', pollViz), 100);
  }

  /* ---- 光源/相机：/api/io 观察点 → 对应对象 emissive 暖白微亮 ---- */
  var vizIO = { polling: false };
  function pollVizIO() {
    if (!viz.hook || vizIO.polling) return;
    vizIO.polling = true;
    fetch(HTTP_BASE + '/api/io').then(function (r) { return r.json(); }).then(function (j) {
      vizIO.polling = false;
      if (!j.ok) return;
      var pts = j.points || {};
      var onN = 0;
      VIZ_LIGHTS.forEach(function (L) {
        var on = pts[L.io] === true;
        L._on = on;
        if (on) onN++;
        if (L.obj) setLightGlow(L.obj, on);
      });
      viz.lightsOn = onN;
    }).catch(function () { vizIO.polling = false; });
  }

  function findObject3D(name) {
    var proj = window.__TWIN_DOM_PROJECT__;
    if (!proj || !viz.hook) return null;
    // 两遍匹配：先全表精确同名，再包含式兜底——避免“蓝膜”被“蓝膜台整体”抢先命中
    var i, o;
    for (i = 0; i < proj.objects.length; i++) {
      o = proj.objects[i];
      if (o.name === name) return viz.hook.obj3d.get(o.id) || null;
    }
    for (i = 0; i < proj.objects.length; i++) {
      o = proj.objects[i];
      if (o.name && o.name.indexOf(name) >= 0) return viz.hook.obj3d.get(o.id) || null;
    }
    return null;
  }

  function pollViz() {
    if (viz.polling) return;
    viz.polling = true;
    fetch(HTTP_BASE + '/api/viz').then(function (r) { return r.json(); }).then(function (d) {
      viz.polling = false;
      viz.fails = 0;
      viz.active = true;
      applyViz(d);
    }).catch(function () {
      viz.polling = false;
      if (++viz.fails >= 3 && viz.active) { viz.active = false; vizOff(); }
    });
  }

  /* ---- emissive 发光：光源·相机暖白微亮（真空吸盘上色已移除，2026-08-20 用户要求） ----
   * saveKey 保存原色，关闭时恢复 */
  function setObjGlow(obj, on, hex, k, saveKey) {
    if (!obj) return;
    obj.traverse(function (m) {
      var mats = m.material ? (Array.isArray(m.material) ? m.material : [m.material]) : [];
      mats.forEach(function (mat) {
        if (!mat.emissive) return;
        if (on) {
          if (!mat.userData[saveKey]) {
            mat.userData[saveKey] = { hex: mat.emissive.getHex(), k: mat.emissiveIntensity };
          }
          mat.emissive.setHex(hex);
          mat.emissiveIntensity = k;
        } else if (mat.userData[saveKey]) {
          mat.emissive.setHex(mat.userData[saveKey].hex);
          mat.emissiveIntensity = mat.userData[saveKey].k;
          delete mat.userData[saveKey];
        }
      });
    });
  }
  function setLightGlow(obj, on) { setObjGlow(obj, on, 0xffe9b0, 0.6, '__vizSavedL'); }

  /* ---- 共焦光束（调焦调平可视化） ----
   * 在传感器本体下挂一根半透明暖色圆柱，测量时点亮 250ms（/api/viz confocal）。 */
  function ensureConfocalBeam(sensorObj, dir) {
    if (!sensorObj) return null;
    var name = '__viz_confocal_beam_' + (dir > 0 ? 'up' : 'down');
    if (sensorObj.getObjectByName) {
      var old = sensorObj.getObjectByName(name);
      if (old) { old.visible = false; return old; }
    }
    var L = 45;   // 光束长度 mm（示意范围，不反映真实量程）
    var r = 0.7;  // 光束半径 mm
    var beam = viz.hook.make({
      geometry: { type: 'Cylinder', radius: r, height: L },
      material: { color: '#ffaa55', opacity: 0.42, transparent: true,
                  emissive: '#ff6600', emissiveIntensity: 0.7,
                  metalness: 0.1, roughness: 0.4 }
    });
    beam.name = name;
    // Cylinder 默认沿 Y 轴；转 X 90° 后沿 Z 轴，再按 dir 伸向下方/上方
    beam.rotation.x = Math.PI / 2;
    beam.position.z = dir * L / 2;
    beam.visible = false;
    sensorObj.add(beam);
    return beam;
  }

  /* ---- 脉冲加热器圆盘（热压键合可视化，2026-08-28 迭代 8） ----
   * 键合头吸盘上方挂一个小圆盘（子节点随轴动），/api/viz heater.temp_C
   * 驱动发光：25℃ 不发光 → 300℃+ 橙红（emissive 强度随温度爬升）。 */
  function ensureHeaterDisc(cupObj) {
    if (!cupObj) return null;
    var name = '__viz_heater_disc';
    if (cupObj.getObjectByName) {
      var old = cupObj.getObjectByName(name);
      if (old) return old;
    }
    var disc = viz.hook.make({
      geometry: { type: 'Cylinder', radius: 6, height: 1.5 },
      material: { color: '#3a3f46', opacity: 0.95, transparent: true,
                  emissive: '#000000', emissiveIntensity: 0,
                  metalness: 0.3, roughness: 0.6 }
    });
    disc.name = name;
    // Cylinder 默认沿 Y 轴；转 X 90° 后沿 Z 轴（水平圆盘）
    disc.rotation.x = Math.PI / 2;
    disc.position.z = 8;           // 吸盘上方（局部系）
    cupObj.add(disc);
    return disc;
  }
  function applyHeater(tempC) {
    var disc = viz.heaterDisc;
    if (!disc) return;
    // 温度→发光：≤25℃ 不发光；300℃ 满橙红
    var k = Math.max(0, Math.min(1, (Number(tempC) - 25) / 275));
    if (!isFinite(k)) k = 0;
    disc.traverse(function (m) {
      var mats = m.material ? (Array.isArray(m.material) ? m.material : [m.material]) : [];
      mats.forEach(function (mat) {
        if (!mat.emissive) return;
        mat.emissive.setHex(k > 0 ? 0xff4400 : 0x000000);
        mat.emissiveIntensity = k * 1.2;
      });
    });
  }

  /* ---- 工件网格 ----
   * bundle 未暴露 THREE 命名空间（window.__THREE__ 只是版本串），贴图/合批
   * 类均不可用；自定义几何靠从 make 产物原型链上溯取得的 BufferGeometry /
   * BufferAttribute / Mesh 构造器（ensureCtors），材质一律借 make 工厂产出
   * （含 Plane/Circle 的 polygonOffset:-1 防 z-fighting 约定）。 */
  function ensureCtors() {
    if (viz.ctorsReady) return viz.ctors;
    viz.ctorsReady = true;
    viz.ctors = null;
    if (!viz.hook) return null;
    var probe = viz.hook.make({ geometry: { type: 'Box', width: 1, depth: 1, height: 1 },
                                material: { color: '#000000' } });
    var BA = probe.geometry.attributes.position.constructor;
    var Mesh = probe.constructor;
    // 原型链上溯：取“新实例带 isBufferGeometry”的最高层类 = BufferGeometry 基类
    var BG = null, p = Object.getPrototypeOf(probe.geometry), hops = 0;
    while (p && hops++ < 8) {
      var c = p.constructor;
      if (c) { try { if ((new c()).isBufferGeometry) BG = c; } catch (_) {} }
      p = Object.getPrototypeOf(p);
    }
    viz.ctors = { BG: BG, BA: BA, Mesh: Mesh };
    return viz.ctors;
  }

  // quads: [{x,y,z,sx,sy,rz?,color?}] → 非索引 BufferGeometry（法线 +Z，俯视 CCW）
  function buildQuadsGeometry(quads) {
    var c = ensureCtors();
    if (!c || !c.BG) return null;
    var n = quads.length, i, j;
    var pos = new Float32Array(n * 18), nor = new Float32Array(n * 18);
    var col = null;
    for (i = 0; i < n; i++) if (quads[i].color) { col = new Float32Array(n * 18); break; }
    var k = 0, ORD = [0, 1, 2, 0, 2, 3];
    for (i = 0; i < n; i++) {
      var q = quads[i], hx = q.sx / 2, hy = q.sy / 2;
      var cs = q.rz ? Math.cos(q.rz) : 1, sn = q.rz ? Math.sin(q.rz) : 0;
      var cx = [-hx, hx, hx, -hx], cy = [-hy, -hy, hy, hy];
      for (j = 0; j < 6; j++) {
        var o = ORD[j], lx = cx[o], ly = cy[o];
        pos[k]     = q.x + lx * cs - ly * sn;
        pos[k + 1] = q.y + lx * sn + ly * cs;
        pos[k + 2] = q.z;
        nor[k] = 0; nor[k + 1] = 0; nor[k + 2] = 1;
        if (col) {
          var qc = q.color || [1, 1, 1];
          col[k] = qc[0]; col[k + 1] = qc[1]; col[k + 2] = qc[2];
        }
        k += 3;
      }
    }
    var g = new c.BG();
    g.setAttribute('position', new c.BA(pos, 3));
    g.setAttribute('normal', new c.BA(nor, 3));
    if (col) g.setAttribute('color', new c.BA(col, 3));
    return g;
  }

  // 带三角缺口的圆片（缺口朝 -X）：顶/底扇面 + 外侧壁（含缺口两径向壁），
  // 每边 4 三角形，逐面法线（顶/底 ±Z，侧壁外向=(dy,-dx)，轮廓俯视 CCW）
  function buildWaferGeometry(R, T) {
    var c = ensureCtors();
    if (!c || !c.BG) return null;
    var SEG = 96;
    var h = Math.max(0, Math.min(NOTCH_HALF, 1.0));
    var nd = Math.max(0, Math.min(NOTCH_DEPTH, R * 0.5));
    var zt = T / 2, zb = -T / 2;
    var pts = [], i;
    var a0 = Math.PI + h, span = 2 * (Math.PI - h);
    for (i = 0; i <= SEG; i++) {
      var a = a0 + span * i / SEG;
      pts.push([R * Math.cos(a), R * Math.sin(a)]);
    }
    if (nd > 0 && h > 0) pts.push([-(R - nd), 0]);   // 缺口顶点
    var np = pts.length;
    var pos = new Float32Array(np * 4 * 9), nor = new Float32Array(np * 4 * 9);
    var k = 0;
    function tri(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz) {
      pos[k] = ax; pos[k + 1] = ay; pos[k + 2] = az;
      nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz; k += 3;
      pos[k] = bx; pos[k + 1] = by; pos[k + 2] = bz;
      nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz; k += 3;
      pos[k] = cx; pos[k + 1] = cy; pos[k + 2] = cz;
      nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz; k += 3;
    }
    for (i = 0; i < np; i++) {
      var p0 = pts[i], p1 = pts[(i + 1) % np];
      tri(0, 0, zt, p0[0], p0[1], zt, p1[0], p1[1], zt, 0, 0, 1);   // 顶
      tri(0, 0, zb, p1[0], p1[1], zb, p0[0], p0[1], zb, 0, 0, -1);  // 底
      var dx = p1[0] - p0[0], dy = p1[1] - p0[1];
      var L = Math.sqrt(dx * dx + dy * dy) || 1;
      var nx = dy / L, ny = -dx / L;
      tri(p0[0], p0[1], zb, p1[0], p1[1], zb, p1[0], p1[1], zt, nx, ny, 0);
      tri(p0[0], p0[1], zb, p1[0], p1[1], zt, p0[0], p0[1], zt, nx, ny, 0);
    }
    var g = new c.BG();
    g.setAttribute('position', new c.BA(pos, 3));
    g.setAttribute('normal', new c.BA(nor, 3));
    return g;
  }

  function removeMesh(m) {
    if (m && m.parent) m.parent.remove(m);
  }
  // 移除并释放 GPU 资源（deep=true 时含子节点；勿用于共享资产仍被引用的 mesh）
  function dropMesh(m, deep) {
    if (!m) return;
    if (deep) m.traverse(function (c) {
      if (c.geometry && c.geometry.dispose) c.geometry.dispose();
      if (c.material && c.material.dispose) c.material.dispose();
    });
    removeMesh(m);
  }

  /* ---- 晶圆：半透明圆片 + die map 贴片（子节点随动） ---- */
  function ensureWaferMesh() {
    if (viz.waferMesh || !viz.hook) return viz.waferMesh;
    var R = vizGeom.waferR, T = vizGeom.waferT;
    var m = viz.hook.make({ geometry: { type: 'Cylinder', radius: R, height: T, segments: 48 },
                            material: { color: WAFER_COLOR, metalness: 0.15, roughness: 0.4,
                                        opacity: WAFER_OPACITY } });
    var geo = buildWaferGeometry(R, T);
    if (geo) { m.geometry.dispose(); m.geometry = geo; }   // 换成带缺口圆片
    m.name = '__viz_wafer';
    m.renderOrder = 1;          // 透明顺序：晶圆体(1) 先画，die map(2) 后画
    var chuck = viz.carrierObjs.wafer;
    if (chuck) {
      // 以吸盘当前包围盒顶面为承载面，世界坐标转吸盘局部后挂为子节点（随轴运动）。
      // 浮起量 2.54mm（2026-09-06 迭代 17 标定，旧值 0.5）：TwinDOM 晶圆吸盘
      // 网格顶 882.28 / 陶瓷面网格顶 881.9 比后端物理陶瓷面 CHUCK_TOP_Z=884.9
      // 低 ~2.6mm，而键合头 z 链是轴驱动物理准确的（有效接触位 -167.43 时
      // head die 底面世界 z=885.82≈物理 885.9）——为让键合时 head 上的 die
      // 底面贴合晶圆面（此前视觉缝 ~2.0mm，"上芯片没接触上"的显示侧），
      // 晶圆顶面对齐头侧真值链：浮起=885.82−882.28−T(1.0)≈2.54。
      // die map/标记/已键合 die 全是晶圆体子节点或 waferBase 派生，
      // 一并随动，内部一致性不变。
      var b = new viz.hook.Box3().setFromObject(chuck);
      var cc = b.getCenter(new viz.hook.V3());
      var local = chuck.worldToLocal(new viz.hook.V3(cc.x, cc.y, b.max.z + T / 2 + 2.54));
      viz.waferBase = local.clone();
      m.position.copy(local);
      chuck.add(m);
    } else {
      viz.waferBase = null;
      viz.hook.scene.add(m);
    }
    viz.waferMesh = m;
    rebuildDieMap();
    rebuildWaferMarks();
    return m;
  }
  // 处方尺寸变化 → 重建晶圆体（保留可见性与含 offset 的当前位置）
  function rebuildWaferMesh() {
    var old = viz.waferMesh;
    if (!old) return;
    var vis = old.visible, pos = old.position.clone();
    dropMesh(old, true);
    viz.waferMesh = null;
    viz.dieMapMesh = null;
    viz.waferMarksMesh = null;
    var m = ensureWaferMesh();
    if (m) { m.visible = vis; m.position.copy(pos); }
  }
  // 标记 quad 生成（处方 film.marks schema）：位置 = ratio × (sx/2, sy/2)
  // （逐轴等比，die 尺寸变标记自动跟随），尺寸 = size_um/1000 mm 真实比例
  // （150µm 在 4mm die 上 ~1/30 边长，远景看不见属正常，不为显眼放大）；
  // box=方块，cross=两正交条。(cx,cy) 为可选中心偏移（die map 的 site 中心用）
  function pushMarkQuads(quads, marks, sx, sy, z, color, cx, cy) {
    var ms = Math.max(0.01, marks.size_mm || 0.15);
    var n = Math.min(marks.per_die || 0, (marks.ratio || []).length);
    for (var i = 0; i < n; i++) {
      var mx = (cx || 0) + marks.ratio[i][0] * sx / 2;
      var my = (cy || 0) + marks.ratio[i][1] * sy / 2;
      if (marks.shape === 'cross') {
        quads.push({ x: mx, y: my, z: z, sx: ms, sy: ms * 0.12, color: color });
        quads.push({ x: mx, y: my, z: z, sx: ms * 0.12, sy: ms, color: color });
      } else {   // box（默认）
        quads.push({ x: mx, y: my, z: z, sx: ms, sy: ms, color: color });
      }
    }
  }

  // 晶圆上片误差变换（设计坐标 → 吸盘局部坐标）：offset 平移 + Rz 旋转。
  // 与 applyViz 中对 waferMesh 施加的摆放（waferBase+offset, rotation.z=rz）
  // 同源；键合落位的 die（carrier=wafer）用同一套变换才能贴到晶圆上。
  function waferToChuck(x, y, w) {
    var off = (w && w.offset) || [0, 0];
    var rz = (w && w.rz) || 0;
    var c = Math.cos(rz), s = Math.sin(rz);
    return [x * c - y * s + (off[0] || 0), x * s + y * c + (off[1] || 0)];
  }

  // die map：全量 site 合并几何（金色半透明 pad + film.marks 驱动的标记），
  // 顶点色区分，单 mesh 单 drawcall，挂晶圆体顶面 +0.03mm；
  // 已键合 site 的 pad/标记不再绘制（位置与落位 die 重叠会 z-fight 盖住 die）
  function rebuildDieMap() {
    if (viz.dieMapMesh) { dropMesh(viz.dieMapMesh, true); viz.dieMapMesh = null; }
    if (!viz.waferMesh || !viz.sitesCache || !viz.sitesCache.positions.length) return;
    var sx = viz.siteDieSX * 0.92, sy = viz.siteDieSY * 0.92;
    var z = vizGeom.waferT / 2 + 0.03;
    var gold = [0.83, 0.69, 0.22], cyan = [0.90, 0.98, 1.0];
    var bonded = viz.bondedLocal || [];
    var quads = [];
    viz.sitesCache.positions.forEach(function (p) {
      var isBonded = false;
      for (var bi = 0; bi < bonded.length; bi++) {
        if (Math.abs(bonded[bi][0] - p[0]) < 0.01 &&
            Math.abs(bonded[bi][1] - p[1]) < 0.01) { isBonded = true; break; }
      }
      if (isBonded) return;      // 已键合 site：pad 留给落位 die 显示
      quads.push({ x: p[0], y: p[1], z: z, sx: sx, sy: sy, color: gold });
      // 与 film die 贴花同源：每 site 4 个标记 box（同比例同尺寸）
      pushMarkQuads(quads, viz.filmMarks, viz.siteDieSX, viz.siteDieSY,
                    z + 0.004, cyan, p[0], p[1]);
    });
    var g = buildQuadsGeometry(quads);
    if (!g) return;
    var m = viz.hook.make({ geometry: { type: 'Plane', width: 1, depth: 1 },
                            material: { color: '#ffffff', opacity: 0.30,
                                        metalness: 0.6, roughness: 0.4 } });
    m.geometry.dispose();
    m.geometry = g;
    m.material.vertexColors = true;
    m.material.needsUpdate = true;
    m.name = '__viz_die_map';
    m.renderOrder = 2;
    viz.waferMesh.add(m);
    viz.dieMapMesh = m;
  }
  // 晶圆级对准标记（wafer.marks[]：十字等，晶圆局部绝对坐标，size_um 真实
  // 尺寸；下相机视觉检测对象，3D 里应可见）。独立小 mesh 挂晶圆体，
  // 与 die map 解耦（不依赖站点取数）
  function rebuildWaferMarks() {
    if (viz.waferMarksMesh) { dropMesh(viz.waferMarksMesh, true); viz.waferMarksMesh = null; }
    if (!viz.waferMesh || !viz.waferMarks.length) return;
    var z = vizGeom.waferT / 2 + 0.03;
    var cyan = [0.90, 0.98, 1.0];
    var quads = [];
    viz.waferMarks.forEach(function (mk) {
      var ms = Math.max(0.05, (Number(mk.size_um) || 2000) / 1000);
      var x = Number(mk.x_mm) || 0, y = Number(mk.y_mm) || 0;
      if (mk.shape === 'cross') {
        quads.push({ x: x, y: y, z: z, sx: ms, sy: ms * 0.12, color: cyan });
        quads.push({ x: x, y: y, z: z, sx: ms * 0.12, sy: ms, color: cyan });
      } else {
        quads.push({ x: x, y: y, z: z, sx: ms, sy: ms, color: cyan });
      }
    });
    var g = buildQuadsGeometry(quads);
    if (!g) return;
    var m = viz.hook.make({ geometry: { type: 'Plane', width: 1, depth: 1 },
                            material: { color: '#ffffff', opacity: 0.9,
                                        metalness: 0.3, roughness: 0.5 } });
    m.geometry.dispose();
    m.geometry = g;
    m.material.vertexColors = true;
    m.material.needsUpdate = true;
    m.name = '__viz_wafer_marks';
    m.renderOrder = 2;
    viz.waferMesh.add(m);
    viz.waferMarksMesh = m;
  }

  /* ---- die 三层：灰基体(72%厚) + 金电路面(28%厚) + 顶面贴花 ----
   * 几何/材质全部共享（按处方尺寸建一次），每 die 3 个 Mesh 实例 */
  function ensureDieAssets() {
    if (viz.dieAssets || !viz.hook) return viz.dieAssets;
    var sx = vizGeom.dieSX, sy = vizGeom.dieSY, t = vizGeom.dieT;
    var tb = t * 0.72, tg = t - tb;
    var base = viz.hook.make({ geometry: { type: 'Box', width: sx, depth: sy, height: tb },
                               material: { color: '#6F7D8C', metalness: 0.35, roughness: 0.55 } });
    var gold = viz.hook.make({ geometry: { type: 'Box', width: sx, depth: sy, height: tg },
                               material: { color: '#D4AF37', metalness: 0.85, roughness: 0.3 } });
    var dg = buildDieDecalGeometry(sx, sy);
    var decal = null;
    if (dg) {
      decal = viz.hook.make({ geometry: { type: 'Plane', width: sx, depth: sy },
                              material: { color: '#ffffff', opacity: 0.92,
                                          metalness: 0.5, roughness: 0.35 } });
      decal.geometry.dispose();
      decal.geometry = dg;
      decal.material.vertexColors = true;
      decal.material.needsUpdate = true;
    }
    viz.dieAssets = {
      baseGeo: base.geometry, baseMat: base.material,
      goldGeo: gold.geometry, goldMat: gold.material,
      decalGeo: decal ? decal.geometry : null,
      decalMat: decal ? decal.material : null,
      goldZ: tb / 2 + tg / 2,            // 各层相对 die 原点（基体中心）的 z
      decalZ: tb / 2 + tg + 0.004
    };
    return viz.dieAssets;
  }
  // 顶面贴花：棕色互连网格 + 3×3 金焊盘（装饰层）+ film.marks 驱动的标记
  // （顶点色，同面 z=0；材质透明 depthWrite=false → 按缓冲顺序叠画：
  //   网格→焊盘→标记）。film 与 head/flip 上的 die 共用同一份标记定义。
  function buildDieDecalGeometry(sx, sy) {
    var s = Math.min(sx, sy);
    var brown = [0.42, 0.27, 0.14], pad = [0.98, 0.85, 0.45], cyan = [0.90, 0.98, 1.0];
    var quads = [], i, r, c2;
    for (i = -2; i <= 2; i++) {
      if (i === 0) continue;             // 中线留空（视觉层次）
      quads.push({ x: i * s * 0.17, y: 0, z: 0, sx: s * 0.022, sy: sy * 0.94, color: brown });
      quads.push({ x: 0, y: i * s * 0.17, z: 0, sx: sx * 0.94, sy: s * 0.022, color: brown });
    }
    for (r = -1; r <= 1; r++)
      for (c2 = -1; c2 <= 1; c2++)
        quads.push({ x: c2 * s * 0.28, y: r * s * 0.28, z: 0,
                     sx: s * 0.17, sy: s * 0.17, color: pad });
    pushMarkQuads(quads, viz.filmMarks, sx, sy, 0, cyan);
    return buildQuadsGeometry(quads);
  }
  // 尺寸变 → die 池全清（下一帧 applyViz 按新资产自动重建）+ 释放旧共享资产
  function rebuildDieAssets() {
    Object.keys(viz.dieMeshes).forEach(function (id) {
      removeMesh(viz.dieMeshes[id]);
      delete viz.dieMeshes[id];
    });
    var a = viz.dieAssets;
    if (a) {
      [a.baseGeo, a.goldGeo, a.decalGeo].forEach(function (g) { if (g) g.dispose(); });
      [a.baseMat, a.goldMat, a.decalMat].forEach(function (m) { if (m) m.dispose(); });
      viz.dieAssets = null;
    }
  }
  function ensureDieMesh(id) {
    var m = viz.dieMeshes[id];
    if (m || !viz.hook) return m;
    var a = ensureDieAssets();
    var c = ensureCtors();
    if (!a || !c) {   // 构造器不可用时退化为单层小方块（旧样式）
      m = viz.hook.make({ geometry: { type: 'Box', width: vizGeom.dieSX, depth: vizGeom.dieSY,
                                      height: vizGeom.dieT },
                          material: { color: '#8fd0ff', metalness: 0.2, roughness: 0.5 } });
    } else {
      m = new c.Mesh(a.baseGeo, a.baseMat);
      var g = new c.Mesh(a.goldGeo, a.goldMat);
      g.position.z = a.goldZ;
      m.add(g);
      if (a.decalGeo) {
        var d = new c.Mesh(a.decalGeo, a.decalMat);
        d.position.z = a.decalZ;
        d.name = '__viz_die_decal';
        m.add(d);
      }
    }
    m.name = '__viz_die_' + id;
    viz.dieMeshes[id] = m;
    return m;
  }

  /* ---- 处方联动：GET /api/recipe 签名轮询 → 重建 wafer/die/die map ----
   * 签名 = 影响 3D 几何的字段（wafer 直径/厚度、film.dies 尺寸/厚度、
   * wafer_sites 布局配置）；apply/restore_default 不改处方 JSON，不会误触发 */
  function pollRecipe() {
    if (!viz.hook || viz.recipePolling) return;
    viz.recipePolling = true;
    fetch(HTTP_BASE + '/api/recipe').then(function (r) { return r.json(); }).then(function (j) {
      viz.recipePolling = false;
      if (j && j.ok) applyRecipe(j.recipe || {});
    }).catch(function () { viz.recipePolling = false; });
  }
  function recipeSignature(rec) {
    var w = rec.wafer || {}, film = rec.film || {}, fd = film.dies || {},
        ws = rec.wafer_sites || {};
    return JSON.stringify([w.diameter_mm, w.thickness_mm,
                           fd.size_x_mm, fd.size_y_mm, fd.thickness_mm,
                           ws.mode, ws.size_x_mm, ws.size_y_mm,
                           ws.gap_x_mm, ws.gap_y_mm, ws.edge_margin_mm,
                           ws.rows, ws.cols, ws.pitch_x_mm, ws.pitch_y_mm,
                           ws.origin_x_mm, ws.origin_y_mm,
                           film.marks || null, w.marks || null]);
  }
  function applyRecipe(rec) {
    var sig = recipeSignature(rec);
    if (sig === viz.recipeSig) return;
    viz.recipeSig = sig;
    var w = rec.wafer || {}, film = rec.film || {}, fd = film.dies || {},
        ws = rec.wafer_sites || {};
    vizGeom.waferR = Math.max(1, Number(w.diameter_mm) || 300) / 2;
    vizGeom.waferT = Math.max(0.1, Number(w.thickness_mm) || 1);
    vizGeom.dieSX = Math.max(0.1, Number(fd.size_x_mm) || 4);
    vizGeom.dieSY = Math.max(0.1, Number(fd.size_y_mm) || 4);
    vizGeom.dieT = Math.max(0.05, Number(fd.thickness_mm) || 0.5);
    // wafer_sites.size 为 null 时跟随 film.dies（后端同规则）
    viz.siteDieSX = ws.size_x_mm != null ? Number(ws.size_x_mm) : vizGeom.dieSX;
    viz.siteDieSY = ws.size_y_mm != null ? Number(ws.size_y_mm) : vizGeom.dieSY;
    // 标记：film.marks（比例化，film/head/flip 的 die 与 die map 共用）
    var fm = film.marks || {};
    viz.filmMarks = {
      shape: fm.shape || 'box',
      size_mm: Math.max(0.01, (Number(fm.size_um) || 150) / 1000),
      ratio: Array.isArray(fm.ratio) ? fm.ratio : [],
      per_die: Math.max(0, Number(fm.per_die) || 0)
    };
    viz.waferMarks = Array.isArray(w.marks) ? w.marks : [];
    viz.sitesCache = null;      // 先作废旧站点：wafer 重建时跳过 die map，待重取后补画
    rebuildDieAssets();
    rebuildWaferMesh();
    fetchWaferSites();
  }
  function fetchWaferSites() {
    if (viz.sitesFetching) return;
    viz.sitesFetching = true;
    fetch(HTTP_BASE + '/api/recipe/wafer_sites?limit=100000')
      .then(function (r) { return r.json(); }).then(function (j) {
        viz.sitesFetching = false;
        if (!j || !j.ok) return;
        viz.sitesCache = { positions: j.positions || [] };
        rebuildDieMap();
      }).catch(function () { viz.sitesFetching = false; });
  }

  /* ---- 退路：HUD 下方状态灯带 ---- */
  function buildVizStrip() {
    var el = document.createElement('div');
    el.style.cssText =
      'position:fixed;right:12px;top:190px;z-index:9999;padding:8px 10px;border-radius:8px;' +
      'background:rgba(15,25,40,.85);color:#e7eef8;border:1px solid #30445d;' +
      'font:12px/1.8 Consolas,Microsoft YaHei,monospace;pointer-events:none;white-space:pre';
    document.body.appendChild(el);
    viz.strip = el;
  }
  function renderStrip(d) {
    if (!viz.strip) return;
    var cups = VIZ_CUPS.map(function (c) {
      return (c._on ? '●' : '○') + c.objName;
    }).join('  ');
    var counts = { film: 0, flip: 0, head: 0, wafer: 0 };
    (d && d.dies || []).forEach(function (die) { counts[die.carrier] = (counts[die.carrier] || 0) + 1; });
    var w = d && d.wafer;
    viz.strip.textContent =
      '真空  ' + (viz.active ? cups : '○'.repeat(0) + '（离线）') + '\n' +
      '工件  wafer ' + (w && w.present ? '●在位' : '○无') +
      '  dies film:' + counts.film + ' flip:' + counts.flip +
      ' head:' + counts.head + ' wafer:' + counts.wafer;
  }

  /* ---- 应用一帧 viz 数据 ---- */
  function applyViz(d) {
    // 真空：键名与吸盘关键词双向包含匹配
    var vac = (d && d.vacuum) || {};
    VIZ_CUPS.forEach(function (c) {
      var on = false;
      Object.keys(vac).forEach(function (name) {
        var nl = name.toLowerCase();
        c.keys.forEach(function (k) {
          if (nl.indexOf(k.toLowerCase()) >= 0 || k.indexOf(name) >= 0) on = on || !!vac[name];
        });
      });
      c._on = on;   // 真空状态只进灯带/调试出口；3D 上色功能已按用户要求移除（2026-08-20）
    });
    // wafer
    var w = (d && d.wafer) || {};
    if (viz.hook) {
      if (w.present) {
        var wm = ensureWaferMesh();
        if (wm) {
          wm.visible = true;
          var off = w.offset || [0, 0];
          if (viz.waferBase) {
            wm.position.set(viz.waferBase.x + (off[0] || 0),
                            viz.waferBase.y + (off[1] || 0), viz.waferBase.z);
            wm.rotation.z = w.rz || 0;   // 上片 Rz 误差（rad，全线统一）
          }
        }
      } else if (viz.waferMesh) {
        viz.waferMesh.visible = false;
      }
      // dies：按 id 池化，carrier 变化换父节点
      var seen = {};
      var bondedList = [];
      (d.dies || []).forEach(function (die) {
        seen[die.id] = 1;
        var m = ensureDieMesh(die.id);
        var parent = viz.carrierObjs[die.carrier];
        if (!m || !parent) return;
        if (m.parent !== parent) parent.add(m);   // pos 为载体系局部坐标，直接换父即可
        var p = die.pos || [0, 0, 0];
        var px = p[0] || 0, py = p[1] || 0;
        var dz = p[2] || 0;
        var drz = die.rz || 0;
        if (die.carrier === 'film' && viz.filmTopLocalZ != null)
          dz += viz.filmTopLocalZ + vizGeom.dieT * 0.36 + 0.12;  // 膜面顶 + 基体半高 + 0.12 可视浮起
        if (die.carrier === 'flip') {
          // 芯片吸盘=杯板（厚 1mm，局部原点在板心，真空面=+0.5）：die 恒贴
          // 真空面，随 FA_Rx 一起翻转——呈现位 Rx=0 在板上方（背面朝上），
          // 取片位 Rx=−π 挂板下（标记面贴面朝上）
          dz += 0.5 + vizGeom.dieT * 0.5;
          m.rotation.x = Math.PI;               // 标记面(贴花)朝吸盘面
        } else if (die.carrier === 'head') {
          // 键合头吸盘（厚 1.2mm）：die 挂下表面（背面贴吸盘），标记面朝下
          // （up 相机从下方看标记）
          dz -= 0.6 + vizGeom.dieT * 0.5;
          m.rotation.x = Math.PI;
        } else if (die.carrier === 'wafer') {
          // 键合落位的 die：设计坐标叠加上片误差（offset+rz，与 waferMesh
          // 摆放同一套变换），贴晶圆顶面（吸盘局部 z = 晶圆体中心 z +
          // 晶圆半厚 + die 半高 + 0.02 可视浮起）
          bondedList.push([px, py]);
          var wp2 = waferToChuck(px, py, w);
          px = wp2[0]; py = wp2[1];
          dz = (viz.waferBase ? viz.waferBase.z : 0) +
               vizGeom.waferT / 2 + vizGeom.dieT / 2 + 0.02;
          drz += (w && w.rz) || 0;
          // 倒装键合后背面朝上：电路面（金层+贴花）贴晶圆面，灰基体朝上
          m.rotation.x = Math.PI;
        } else {
          m.rotation.x = 0;
        }
        // flip 载体的标记面贴死真空面（夹心，物理上任何角度不可见）——
        // 隐藏贴花。否则贴花（透明 depthWrite=false）翻到芯片体下方 ~1mm
        // 处，远距离深度缓冲量化精度不足时会 z-fight 渗绘到芯片顶面，
        // 远看"正面"凭空出现（2026-08-25 用户报告）。head 标记面朝下暴露
        // （up 相机语义）保留；上方始终有键合头本体遮挡，无渗绘路径。
        // wafer 同理：键合后贴花贴晶圆面不可见，同样隐藏防渗绘。
        var dc = m.children && m.children.length > 1 ? m.children[1] : null;
        if (dc && dc.name === '__viz_die_decal')
          dc.visible = (die.carrier !== 'flip' && die.carrier !== 'wafer');
        m.position.set(px, py, dz);
        m.rotation.z = drz;                         // rz 全线统一 rad
        m.visible = true;
      });
      // 已键合 die 集合变化 → 重画 die map（隐藏被占 site 的金 pad）
      var bsig = bondedList.map(function (b) { return b[0] + ',' + b[1]; }).join(';');
      if (bsig !== viz.bondedSig) {
        viz.bondedSig = bsig;
        viz.bondedLocal = bondedList;
        rebuildDieMap();
      }
      Object.keys(viz.dieMeshes).forEach(function (id) {
        if (!seen[id]) { removeMesh(viz.dieMeshes[id]); delete viz.dieMeshes[id]; }
      });
    }
    renderStrip(d);
    // 共焦光束开关
    if (viz.hook && d && d.confocal) {
      VIZ_CONFOCAL.forEach(function (C) {
        if (C.beam) C.beam.visible = !!d.confocal[C.probe];
      });
    }
    // 脉冲加热器发光（/api/viz heater.temp_C 驱动，confocal 同款先例）
    if (viz.hook && d && d.heater) applyHeater(d.heater.temp_C);
    // 无 DOM 痕迹的调试出口：headless dump-dom 可核对
    if (hudEl) {
      var confOn = 0;
      if (d && d.confocal) {
        Object.keys(d.confocal).forEach(function (k) { if (d.confocal[k]) confOn++; });
      }
      hudEl.setAttribute('data-viz', JSON.stringify({
        mode: viz.hook ? 'scene' : 'strip', active: viz.active,
        cupsFound: VIZ_CUPS.filter(function (c) { return c.obj; }).length,
        cupsOn: VIZ_CUPS.filter(function (c) { return c._on; }).length,
        lightsFound: VIZ_LIGHTS.filter(function (L) { return L.obj; }).length,
        lightsOn: VIZ_LIGHTS.filter(function (L) { return L._on; }).length,
        confocalFound: VIZ_CONFOCAL.filter(function (C) { return C.beam; }).length,
        confocalOn: confOn,
        heaterFound: viz.heaterDisc ? 1 : 0,
        heaterTemp: (d && d.heater) ? d.heater.temp_C : null,
        wafer: !!(w && w.present), dies: (d && d.dies || []).length,
        recipe: viz.recipeSig ? 1 : 0,
        sites: viz.sitesCache ? viz.sitesCache.positions.length : 0,
        dieMap: !!viz.dieMapMesh,
        waferR: vizGeom.waferR,
        dieSize: vizGeom.dieSX + 'x' + vizGeom.dieSY + 'x' + vizGeom.dieT,
        filmMarks: viz.filmMarks.per_die + 'x' + viz.filmMarks.shape +
                   '@' + viz.filmMarks.size_mm + 'mm',
        waferMarks: viz.waferMarks.length
      }));
    }
  }
  function vizOff() {
    VIZ_CUPS.forEach(function (c) { c._on = false; });
    VIZ_LIGHTS.forEach(function (L) { L._on = false; if (viz.hook && L.obj) setLightGlow(L.obj, false); });
    VIZ_CONFOCAL.forEach(function (C) { if (C.beam) C.beam.visible = false; });
    applyHeater(0);
    if (viz.waferMesh) viz.waferMesh.visible = false;
    Object.keys(viz.dieMeshes).forEach(function (id) { removeMesh(viz.dieMeshes[id]); delete viz.dieMeshes[id]; });
    renderStrip(null);
    if (hudEl) hudEl.setAttribute('data-viz', JSON.stringify({ mode: viz.hook ? 'scene' : 'strip', active: false }));
  }

  /* ---------- 轴值应用（复用运行时 oninput 链，绕过 DOM 事件） ---------- */
  function applyAxisValue(rec, v) {
    var input = rec.input;
    // range input 会按 min/max 截断，数据越界时动态扩界，保证位姿正确
    if (v < Number(input.min)) input.min = v;
    if (v > Number(input.max)) input.max = v;
    if (Math.abs(Number(input.value) - v) < 1e-9) return;
    input.value = v;
    if (rec.origOninput) rec.origOninput.call(input); // = axis.current=v; nu(); 刷新显示
  }

  /* ---------- WebSocket ---------- */
  var ws = null;
  var connected = false;
  var lastFrame = null;   // {tRecv, wallTime, values}
  var prevFrame = null;
  var lastSeq = -1;
  var droppedFrames = 0;
  var frameArrivals = []; // 最近到达时间戳，算推送频率
  var latencySamples = []; // 单向延迟滚动样本 (ms)

  function connect() {
    try { ws = new WebSocket(WS_URL); } catch (e) {
      reportError('connect', e);
      return scheduleReconnect();
    }
    ws.binaryType = 'arraybuffer';
    ws.onopen = safe('ws.onopen', function () {
      connected = true;
      lastSeq = -1;
      setSliderOnlineMode(true);
      updateHud();
    });
    ws.onmessage = safe('ws.onmessage', function (ev) {
      if (typeof ev.data === 'string') {
        // 服务端回执：拒绝（ok:false，如沙盒干涉/限位）→ HUD 提示 + 滑块回弹
        try {
          var ack = JSON.parse(ev.data);
          if (ack && ack.ok === false) onCmdRejected(ack);
        } catch (_) {}
        return;
      }
      var f = decodeFrame(ev.data);
      if (!f) return;
      var now = performance.now() / 1000;
      prevFrame = lastFrame;
      lastFrame = { tRecv: now, wallTime: f.wallTime, values: f.values };
      // 丢帧统计（seq 跳号）
      if (lastSeq >= 0 && f.seq > lastSeq + 1) droppedFrames += f.seq - lastSeq - 1;
      lastSeq = f.seq;
      // 推送频率：1s 滑动窗
      frameArrivals.push(now);
      while (frameArrivals.length && frameArrivals[0] < now - 1) frameArrivals.shift();
      // 单向延迟（本机同时钟）
      var lat = (Date.now() / 1000 - f.wallTime) * 1000;
      latencySamples.push(lat);
      if (latencySamples.length > 100) latencySamples.shift();
    });
    ws.onclose = safe('ws.onclose', function () {
      connected = false;
      setSliderOnlineMode(false);
      updateHud();
      scheduleReconnect();
    });
    ws.onerror = function () { /* onclose 会跟随触发 */ };
  }
  function scheduleReconnect() {
    setTimeout(connect, RECONNECT_MS);
  }

  function sendCmd(obj) {
    if (connected && ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  /* ---------- 指令拒绝反馈：后端 409（干涉/限位）→ HUD 红字 + 滑块回弹 ---------- */
  var lastReject = null; // {axis, error, t}
  function onCmdRejected(ack) {
    lastReject = { axis: ack.axis || '', error: String(ack.error || ''),
                   t: performance.now() / 1000 };
    var rec = ack.axis && axesById[ack.axis];
    if (rec && !rec.dragging && lastFrame && rec.frameIndex >= 0) {
      // 滑块回弹到后端实际位置（写入前换算回模型原生单位 deg）
      rec.input.value = frameToModel(rec.frameIndex, lastFrame.values[rec.frameIndex]);
    }
    updateHud();
  }

  /* ---------- 滑块联动：联机时拖滑块 → 发 move 指令；断线恢复单机行为 ---------- */
  function setSliderOnlineMode(online) {
    FRAME_AXIS_IDS.forEach(function (id) {
      var rec = axesById[id];
      if (!rec) return;
      if (online) {
        rec.input.oninput = safe('slider.oninput', function () { onUserDrag(rec); });
      } else {
        rec.input.oninput = rec.origOninput; // 恢复运行时原处理器
      }
    });
  }
  function onUserDrag(rec) {
    var v = Number(rec.input.value);
    var now = performance.now();
    if (now - rec.lastMoveSent >= MOVE_THROTTLE_MS) {
      rec.lastMoveSent = now;
      // 旋转轴模型内为 deg，线上协议为 rad，发送前换算
      sendCmd({ cmd: 'move', axis: rec.axis.id, target: modelToWire(rec.frameIndex, v) });
    } else {
      // 拖尾补发，保证最终值送达
      if (rec.pendingMove) clearTimeout(rec.pendingMove);
      rec.pendingMove = setTimeout(safe('slider.pendingMove', function () {
        rec.lastMoveSent = performance.now();
        sendCmd({ cmd: 'move', axis: rec.axis.id,
                  target: modelToWire(rec.frameIndex, Number(rec.input.value)) });
      }), MOVE_THROTTLE_MS);
    }
  }

  /* ---------- 渲染循环：线性外推 + 应用流值 + HUD ---------- */
  var fpsCount = 0, fpsTime = 0, renderFps = 0;
  var hudTimer = 0;

  function renderLoop() {
    requestAnimationFrame(renderLoop); // 先排下一帧，异常也不会停循环
    try {
      renderTick();
    } catch (e) {
      reportError('renderLoop', e);
    }
  }
  function renderTick() {
    var now = performance.now() / 1000;

    // 渲染 fps
    fpsCount++;
    if (now - fpsTime >= 1) { renderFps = fpsCount; fpsCount = 0; fpsTime = now; }

    if (connected && lastFrame) {
      // 帧间速度（最近 2 帧）
      var dt = prevFrame ? lastFrame.tRecv - prevFrame.tRecv : 0;
      var extrap = Math.min(Math.max(now - lastFrame.tRecv, 0), EXTRAPOLATE_MAX_S);
      for (var i = 0; i < FRAME_AXIS_IDS.length; i++) {
        var rec = axesById[FRAME_AXIS_IDS[i]];
        if (!rec || rec.dragging) continue; // 用户正在拖的轴不被流覆盖
        var v = lastFrame.values[i];
        if (dt > 1e-6 && extrap > 0) {
          v += (lastFrame.values[i] - prevFrame.values[i]) / dt * extrap;
        }
        // 外推在线路单位（rad）上做，写入前统一换算为模型原生单位（deg）；
        // 线性外推与线性换算可交换，先后顺序不影响结果
        applyAxisValue(rec, frameToModel(i, v));
      }
    }

    if (now - hudTimer >= 0.25) { hudTimer = now; updateHud(); }
  }

  /* ---------- HUD ---------- */
  var hudEl = null;
  function initHud() {
    hudEl = document.createElement('div');
    hudEl.style.cssText =
      // 右下角：默认视角下蓝膜台在画面右上区，HUD 放顶部会盖住蓝膜
      'position:fixed;right:12px;bottom:12px;z-index:9999;' +
      'padding:10px 12px;border-radius:8px;min-width:210px;' +
      'background:rgba(15,25,40,.85);color:#e7eef8;border:1px solid #30445d;' +
      'font:12px/1.7 Consolas,Microsoft YaHei,monospace;pointer-events:none;white-space:pre';
    document.body.appendChild(hudEl);
    updateHud();
  }
  function p50(arr) {
    if (!arr.length) return null;
    var s = arr.slice().sort(function (a, b) { return a - b; });
    return s[Math.floor((s.length - 1) / 2)];
  }
  function updateHud() {
    if (!hudEl) return;
    var lat = p50(latencySamples);
    var nowS = performance.now() / 1000;
    var rejectFresh = lastReject && (nowS - lastReject.t) < 6;
    var rejectText = '无';
    if (rejectFresh) {
      rejectText = '✗ ' + (lastReject.axis || '') + ' ' + lastReject.error;
      if (rejectText.length > 44) rejectText = rejectText.slice(0, 44) + '…';
    }
    var lines = [
      '联机状态  ' + (connected ? '● 已连接' : '○ 未连接（1s 重连中）'),
      '推送频率  ' + (connected ? frameArrivals.length + ' Hz' : '—'),
      '单向延迟  ' + (lat !== null ? lat.toFixed(2) + ' ms (p50)' : '—'),
      '丢帧计数  ' + droppedFrames,
      '渲染 fps  ' + renderFps,
      '滑块模式  ' + (connected ? '联机（拖动→后端 move）' : '单机（本地驱动）'),
      '指令回执  ' + rejectText,
      '脚本错误  ' + (scriptError || '无')
    ];
    hudEl.textContent = lines.join('\n');
    hudEl.style.borderColor = !connected ? '#c0392b' :
      (rejectFresh ? '#e67e22' : '#2fbf71');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();
