# -*- coding: utf-8 -*-
"""
panel_mock.py — 轴控面板最小 mock 后端（纯标准库 http.server）

- GET  /                托管面板页面（panel/index.html），便于 headless 浏览器验证
- GET  /api/axes        21 轴静态假数据（值随时间正弦微动，含 actual 误差、config、errormap）
- GET  /api/errormaps   {"ok":true,"enabled":...}
- POST /api/axis/move|jog|home|config, /api/stop|estop|reset|speed, /api/errormaps
  → 打印报文并回 {"ok":true}；/api/errormaps 记录开关状态
- GET  /api/io 328 点假数据；POST /api/io/command 带语义副作用（气路三态/塔灯/电源/
  温控/相机触发/光源/互锁 Reset/Mask/塔灯 UpdateMode）；未注册点位回 ok+spare 备注
- GET  /api/viz（vacuum bool 字典 + vacuum_detail{on,pressure,flow,sealed,grip_ok}、
  wafer、dies、interlock、tower）；POST /api/workpiece/wafer|reset、/api/interlock
- GET  /api/camera/captures 相机拍照记录（触发类命令产生）
- GET  /online 托管联机版 3D，便于 headless 验证真空/工件可视化

用法：python panel_mock.py [端口]  （Ctrl+C 退出；默认 9010，如 9025）
"""
import json
import math
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST, PORT = '127.0.0.1', 9010
import sys
if len(sys.argv) > 1:
    PORT = int(sys.argv[1])   # 9010 被占用时可指定备用端口，如 9025
PANEL_HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'panel', 'index.html')

# (name, controller, rotary, limits, 正弦幅值)
AXES = [
    ('WS_LS_X',    'ACS', False, (-200, 200),     50),
    ('WS_LS_Y',    'ACS', False, (-200, 200),     50),
    ('WS_Rz',      'ACS', True,  (-0.0175, 0.0175), 0.014),
    ('E_EPIN_Z_1', 'PLC', False, (0, 30),         5),
    ('E_EPIN_Z_2', 'PLC', False, (0, 30),         5),
    ('E_EPIN_Z_3', 'PLC', False, (0, 30),         5),
    ('BS_Arm_Z',   'PLC', False, (-190, 0),       30),
    ('BS_CAL_Rx',  'PLC', True,  (-0.00175, 0.00175), 0.0014),
    ('BS_CAL_Ry',  'PLC', True,  (-0.00175, 0.00175), 0.0014),
    ('AS_X',       'ACS', False, (-30, 80),       20),
    ('AS_Y',       'ACS', False, (-30, 120),      30),
    ('AS_Z',       'ACS', False, (-2, 6),         2),
    ('AS_Rx',      'ACS', True,  (-0.00175, 0.00175), 0.0014),
    ('AS_Ry',      'ACS', True,  (-0.00175, 0.00175), 0.0014),
    ('FA_X',       'ACS', False, (-1240, 0),      300),
    ('FA_Z',       'PLC', False, (-40, 40),       20),
    ('FA_Rx',      'PLC', True,  (-math.pi, math.pi), 1.57),
    ('CH_Rz',      'PLC', True,  (-math.pi, math.pi), 2.09),
    ('ES_X',       'PLC', False, (-285, 85),      100),
    ('ES_Y',       'PLC', False, (-175, 195),     100),
    ('ES_PunchZ',  'ACS', False, (0, 5),          2),
]

state = {'em_enabled': True, 'speed': 1.0, 't0': time.time(),
         'limits_override': {}, 'interlock_enabled': True,
         'wafer': {'present': False, 'offset': [0, 0]},
         'temp_target': {}, 'temp_on': {},
         'captures': [], 'cap_seq': 0}

# ---------------- IO 点位（按 runtime/io_points.json 真实路径生成假数据） ----------------
IO_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'runtime', 'io_points.json')
AIR = 'SystemMachine.SS_ME.M_AIR.'


def _io_defaults():
    """path → (kind, 默认值)。语义默认值：门关/锁闭=True，EMO/报警=False，塔灯绿灯亮。"""
    pts = {}
    with open(IO_JSON, encoding='utf-8') as f:
        reg = json.load(f)['points']
    for path, meta in reg.items():
        kind = meta.get('kind')
        if kind == 'bool':
            v = False
            if 'DoorStatus' in path or 'LockStatus' in path:
                v = True
            elif 'oSignalTower_GreenLight' in path:
                v = True
            elif re_pow_on(path):
                v = True
            elif path == AIR + 'WT_WaferVac.oNegtiveState':
                v = True
            elif path in (AIR + 'WT_WaferVac', AIR + 'WT_BondingVac_1', AIR + 'WT_ChipVac'):
                v = True
        elif kind == 'real':
            v = _real_base(path)
        else:
            v = None
        pts[path] = {'kind': kind, 'dir': meta.get('dir'), 'base': v}
    return pts


def re_pow_on(path):
    return path.startswith('GVL_IO.o') and path.endswith('_PowerOn')


def _real_base(path):
    if 'oCurTemp' in path or 'Temper' in path:
        return 25.0
    if 'PressureSwitch' in path:
        return -80.0 if ('Vac' in path or 'Neg' in path) else 400.0
    if 'FlowMeter' in path:
        return 2.0
    if 'Facility_NegPressure' in path:
        return -88.0
    if 'Facility_PosPressure' in path or 'Facility_N2Pressure' in path:
        return 500.0
    return 1.0


IO = _io_defaults()


def io_snapshot():
    """扁平 {path: value}，real 点随时间微动，温控向目标爬坡"""
    t = time.time() - state['t0']
    out = {}
    for path, rec in IO.items():
        v = rec['base']
        if rec['kind'] == 'real' and isinstance(v, (int, float)):
            if 'oCurTemp' in path:
                tgt = state['temp_target'].get(path)
                if state['temp_on'].get(path) and tgt is not None:
                    # 一阶升温/降温
                    v = rec['cur'] = rec.get('cur', 25.0) + (tgt - rec.get('cur', 25.0)) * 0.15
                else:
                    v = rec['cur'] = rec.get('cur', 25.0) + (25.0 - rec.get('cur', 25.0)) * 0.05
            else:
                v = v + 0.05 * abs(v) * math.sin(t * 1.7 + hash(path) % 10)
        out[path] = v
    return out


def io_command(body):
    """应用 IO 命令副作用（简化语义模拟）"""
    path, cmd = body.get('path', ''), body.get('cmd', '')
    args = body.get('args')
    if args is None:
        args = {}
    elif isinstance(args, list):
        # 带参数命令的数组形式：[v0, v1, ...] → value + 位置参数
        args = {'value': args[0] if args else True, '_argv': list(args)}
    applied = None
    if path.startswith(AIR) and cmd in ('OpenPositive', 'OpenNegtive', 'OpenAir'):
        circuit = path[len(AIR):].split('.')[0]
        base = AIR + circuit
        for leaf, key in (('oPositiveState', 'OpenPositive'),
                          ('oNegtiveState', 'OpenNegtive'), ('oAirState', 'OpenAir')):
            p = base + '.' + leaf
            if p in IO:
                IO[p]['base'] = (cmd == key)
        if base in IO:
            IO[base]['base'] = (cmd != 'OpenAir')
        ps = base + '.PressureSwitch.oValue'
        if ps in IO:
            IO[ps]['base'] = {'OpenPositive': 400.0, 'OpenNegtive': -80.0,
                              'OpenAir': 0.0}[cmd]
        applied = '%s.%s' % (circuit, cmd)
    elif 'E_SignalTower' in path and cmd == 'SetBool':
        leaf = path.split('.')[-1]
        gp = 'GVL_IO.oSignalTower_' + leaf
        if gp in IO:
            IO[gp]['base'] = bool(args.get('value'))
            applied = gp
    elif 'M_PowerSwitch' in path and cmd in ('Pulse', 'SetBool'):
        seg = path.split('.')[-1]
        m = seg.replace('M_PowerSwitch', '')
        if m.endswith('_PowerOn') or m.endswith('_PowerOff'):
            sysname = m.rsplit('_Power', 1)[0]
            on = m.endswith('_PowerOn') if cmd == 'Pulse' else bool(args.get('value'))
            for suffix, val in (('_PowerOn', on), ('_PowerOff', not on)):
                gp = 'GVL_IO.o' + sysname + suffix
                if gp in IO:
                    IO[gp]['base'] = val
            applied = 'GVL_IO.o%s_Power*' % sysname
        elif path in IO:  # Fan/IonBar 等单 bool 命令
            IO[path]['base'] = bool(args.get('value')) if cmd == 'SetBool' else True
            applied = path
    elif 'SetTargetTempWithPlan' in path and cmd == 'SetValue':
        # 找对应 oCurTemp
        cur = ('SystemMachine.M_Heater.PulseHeat_Up.oCurTemp' if 'PulseHeat_Up' in path
               else 'SystemMachine.M_Heater.Heat_Dw.oCurTemp')
        state['temp_target'][cur] = float(args.get('value', 25))
        state['temp_on'][cur] = True
        applied = cur + ' → ' + str(args.get('value'))
    elif 'StopTargetTempWithPlan' in path:
        cur = ('SystemMachine.M_Heater.PulseHeat_Up.oCurTemp' if 'PulseHeat_Up' in path
               else 'SystemMachine.M_Heater.Heat_Dw.oCurTemp')
        state['temp_on'][cur] = False
        applied = cur + ' stop'
    elif 'AlignCameraTigger' in path or 'AS_CameraOn' in path:
        # 相机触发（粗对准=AS_CameraOn，精对准=AlignCameraTigger_1/2）：记录拍照
        if 'AlignCameraTigger_1' in path:
            src = '对准相机1'
        elif 'AlignCameraTigger_2' in path:
            src = '对准相机2'
        else:
            src = '粗对准相机'
        if cmd == 'Pulse' or args.get('value'):
            state['cap_seq'] += 1
            snap = {}
            for a in axes_payload()['axes'][:6]:
                snap[a['name']] = round(a['value'], 4)
            state['captures'].append({'id': 'cap_%04d' % state['cap_seq'],
                                      't': round(time.time() - state['t0'], 3),
                                      'source': src, 'axes': snap})
            state['captures'] = state['captures'][-50:]
            leaf = path.rsplit('.', 1)[-1]
            op = ('SystemMachine.SS_Bonding.M_AS.o' + leaf
                  if not leaf.startswith('o') else path)
            if op in IO:
                IO[op]['base'] = True
        applied = 'capture:%s' % src
    elif path.startswith('SystemMachine.SS_Bonding.M_AS.') and \
            ('LightOn' in path or 'WaferHeatReset' in path):
        # 光源开关 / WaferHeatReset：命令点 + 同名 o 观察点镜像
        leaf = path.rsplit('.', 1)[-1]
        val = bool(args.get('value')) if cmd == 'SetBool' else True
        if path in IO:
            IO[path]['base'] = val
        op = 'SystemMachine.SS_Bonding.M_AS.o' + leaf
        if op in IO:
            IO[op]['base'] = val
        applied = '%s=%s' % (leaf, val)
    elif 'AramPosSensorCalibrate' in path:
        op = path.replace('.AramPosSensorCalibrate', '.oAramPosSensorCalibrate')
        if op in IO:
            IO[op]['base'] = True
        applied = 'AramPosSensorCalibrate done'
    elif 'M_InterLock.AllDoors_Mask' in path:
        val = bool(args.get('value')) if cmd == 'SetBool' else True
        for p in ('GVL_IO.oInterLock_AllDoors_Mask',
                  'SystemMachine.SS_ME.M_InterLock.oAllDoors_Mask'):
            if p in IO:
                IO[p]['base'] = val
        state['interlock_enabled'] = not val   # 屏蔽 → 联锁失效
        applied = 'AllDoors_Mask=%s（联锁%s）' % (val, '已屏蔽' if val else '已使能')
    elif 'M_InterLock.AllDoors_Reset' in path:
        for p in ('GVL_IO.oInterLock_AllDoors_Reset',
                  'SystemMachine.SS_ME.M_InterLock.oAllDoors_Reset'):
            if p in IO:
                IO[p]['base'] = True
        # 复位：EMO/门磁/门锁全部恢复正常
        for p in IO:
            if 'EMO' in p and p.startswith('GVL_IO.i'):
                IO[p]['base'] = False
            elif 'DoorStatus' in p or 'LockStatus' in p:
                IO[p]['base'] = True
        applied = 'AllDoors_Reset done'
    elif 'E_SignalTower' in path and path.endswith('.UpdateMode'):
        # 塔灯闪烁模式：UpdateMode(FlashOnPeriod, FlashOffPeriod)，对象/数组 args 均可
        applied = '%s args=%s' % (path.rsplit('.', 2)[-2],
                                  json.dumps(args, ensure_ascii=False))
    elif path in IO and cmd in ('SetBool', 'SetValue', 'Pulse'):
        IO[path]['base'] = (bool(args.get('value')) if cmd == 'SetBool'
                            else args.get('value') if cmd == 'SetValue' else True)
        applied = path
    elif path and path not in IO:
        applied = 'spare: 备用/预留点位，仅记录不执行'
    return applied


# ---------------- /api/viz 工件与真空 ----------------
VIZ_CUP_CIRCUITS = {'晶圆吸盘': 'WT_WaferVac', '键合头吸盘': 'WT_BondingVac_1',
                    '芯片吸盘': 'WT_ChipVac', '蓝膜台': 'WT_AirfootCompVac'}


def cup_detail(circuit):
    """vacuum_detail 单通道：on/pressure/flow/sealed/grip_ok（密封=吸附且压强达标 -60kPa）"""
    base = AIR + circuit
    rec = IO.get(base + '.oNegtiveState') or IO.get(base) or {}
    on = bool(rec.get('base'))
    pres = IO.get(base + '.PressureSwitch.oValue', {}).get('base')
    flow = IO.get(base + '.FlowMeter.oValue', {}).get('base')
    sealed = bool(on and isinstance(pres, (int, float)) and pres <= -60)
    return {'on': on, 'pressure': pres, 'flow': flow,
            'sealed': sealed, 'grip_ok': bool(on and sealed)}


def viz_payload():
    t = time.time() - state['t0']
    dies = []
    # 蓝膜上静态 8 颗
    for i in range(8):
        dies.append({'id': 'film_d%d' % i, 'carrier': 'film',
                     'pos': [-40 + (i % 4) * 25, -30 + (i // 4) * 25, 2], 'rz': 0.0})
    if state['wafer']['present']:
        # 晶圆上 12 颗（栅格）+ 翻转吸盘 1 颗 + 键合头 1 颗（随时间轮换 id）
        for i in range(12):
            dies.append({'id': 'wf_d%d' % i, 'carrier': 'wafer',
                         'pos': [-45 + (i % 4) * 30, -45 + (i // 4) * 30, 3], 'rz': 0.0})
        pick = int(t / 3) % 12
        dies.append({'id': 'flip_cur', 'carrier': 'flip', 'pos': [0, 0, 2], 'rz': 0.1 * t})
        dies.append({'id': 'head_cur', 'carrier': 'head', 'pos': [0, 0, 2],
                     'rz': 0.0, 'src': 'wf_d%d' % pick})
    vacuum = {
        '晶圆吸盘': bool(IO.get(AIR + 'WT_WaferVac.oNegtiveState', {}).get('base')),
        '键合头吸盘': bool(IO.get(AIR + 'WT_BondingVac_1', {}).get('base')),
        '芯片吸盘': bool(IO.get(AIR + 'WT_ChipVac', {}).get('base')),
        '蓝膜台': bool(IO.get(AIR + 'WT_AirfootCompVac', {}).get('base')),
    }
    emo = any(IO.get(p, {}).get('base') for p in IO if 'EMO' in p and p.startswith('GVL_IO.i'))
    doors = all(IO.get('GVL_IO.i%sLock_DoorStatus_%d' % (s, c), {}).get('base', True)
                for s in ('Front', 'Left', 'Right') for c in (1, 2))
    return {
        't': t,
        'vacuum': vacuum,
        'vacuum_detail': {cup: cup_detail(c) for cup, c in VIZ_CUP_CIRCUITS.items()},
        'wafer': dict(state['wafer']),
        'dies': dies,
        'interlock': {'emo': bool(emo), 'doors_ok': bool(doors),
                      'masked': not state['interlock_enabled']},
        'tower': {'red': bool(IO.get('GVL_IO.oSignalTower_RedLight', {}).get('base')),
                  'yellow': bool(IO.get('GVL_IO.oSignalTower_YellowLight', {}).get('base')),
                  'green': bool(IO.get('GVL_IO.oSignalTower_GreenLight', {}).get('base'))},
    }


# 回零假数据：覆盖 4 种状态。homing 轴的 phase 随时间轮转
HOME_PHASES = ['fast_approach', 'retreat', 'latch']
HOME_STATE = {  # name → 固定状态（homing 的 phase 动态）
    'WS_LS_X': 'homing', 'E_EPIN_Z_1': 'fault', 'WS_Rz': 'never',
}


def home_fields(name, t):
    hs = HOME_STATE.get(name, 'done')   # 其余轴默认已回零
    if hs == 'homing':
        return hs, HOME_PHASES[int(t * 2) % len(HOME_PHASES)]
    if hs == 'done':
        return hs, 'done'
    return hs, ''


def axes_payload():
    t = time.time() - state['t0']
    out = []
    for i, (name, ctl, rotary, limits, amp) in enumerate(AXES):
        limits = list(state['limits_override'].get(name, limits))  # config 编辑后的回显
        center = (limits[0] + limits[1]) / 2
        value = center + amp * math.sin(2 * math.pi * 0.2 * t + i * 0.9)
        value = max(limits[0], min(limits[1], value))
        # 实际值 = 名义值 + 微小误差（直线 ~几十 nm，旋转 ~几 µrad）
        err = (30e-6 if not rotary else 3e-6) * math.sin(t * 1.3 + i)
        cfg = ({'vmax': 500, 'amax': 5000, 'jerk': 50000, 'repeat': 2e-6}
               if not rotary else
               {'vmax': 1.0, 'amax': 10, 'jerk': 100, 'repeat': 1e-6})
        cfg['profile'] = 'scurve'
        hs, hp = home_fields(name, t)
        out.append({
            'id': name.lower(), 'name': name, 'controller': ctl,
            'unit': 'rad' if rotary else 'mm',
            'value': value, 'actual': value + err,
            'moving': (i == 0 and (t % 10) < 5),      # 首轴周期性"运动中"
            'settled': not (i == 0 and (t % 10) < 5),
            'home_state': hs, 'home_phase': hp,
            'limits': limits,
            'config': cfg,
            'errormap': {'enabled': state['em_enabled'], 'coverage': limits},
        })
    return {'ok': True, 't': t, 'axes': out,
            'interlock_enabled': state['interlock_enabled'],
            'emo': viz_payload()['interlock']['emo'],
            'doors_ok': viz_payload()['interlock']['doors_ok']}


class Handler(BaseHTTPRequestHandler):
    def _json(self, obj, status=200):
        data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):  # 静音默认访问日志
        pass

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            try:
                with open(PANEL_HTML, 'rb') as f:
                    data = f.read()
            except OSError:
                return self._json({'ok': False, 'error': 'panel not found: %s' % PANEL_HTML}, 404)
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif path == '/api/axes':
            self._json(axes_payload())
        elif path == '/api/errormaps':
            self._json({'ok': True, 'enabled': state['em_enabled']})
        elif path == '/api/io':
            pts = io_snapshot()
            self._json({'ok': True, 'count': len(pts), 'points': pts})
        elif path == '/api/viz':
            self._json(viz_payload())
        elif path == '/api/camera/captures':
            self._json({'ok': True, 'captures': state['captures']})
        elif path == '/online':   # 托管联机版 3D，便于 headless 验证真空/工件可视化
            online = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  '..', '..', 'html_model', 'd2w_online.html')
            try:
                with open(online, 'rb') as f:
                    data = f.read()
            except OSError:
                return self._json({'ok': False, 'error': 'online html not found'}, 404)
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self._json({'ok': False, 'error': 'not found: %s' % self.path}, 404)

    def do_POST(self):
        n = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(n).decode('utf-8', 'replace')
        try:
            body = json.loads(raw) if raw else {}
        except ValueError:
            return self._json({'ok': False, 'error': 'bad json'}, 400)
        print('[mock] POST %s %s' % (self.path, json.dumps(body, ensure_ascii=False)), flush=True)
        if self.path == '/api/errormaps':
            state['em_enabled'] = bool(body.get('enabled'))
        elif self.path == '/api/speed':
            state['speed'] = body.get('scale')
        elif self.path == '/api/axis/config':
            # 软限位在线编辑：校验 lo<hi，持久化并在 /api/axes 回显
            lim = body.get('limits')
            if lim is not None:
                if (not isinstance(lim, list) or len(lim) != 2 or
                        not all(isinstance(x, (int, float)) for x in lim)):
                    return self._json({'ok': False, 'error': 'limits 格式应为 [lo,hi] 数值数组'})
                if not lim[0] < lim[1]:
                    return self._json({'ok': False, 'error': '软限位下限必须小于上限'})
                state['limits_override'][body.get('axis')] = list(lim)
            return self._json({'ok': True, 'persisted': True})
        elif self.path == '/api/io/command':
            applied = io_command(body)
            return self._json({'ok': True, 'applied': applied})
        elif self.path == '/api/workpiece/wafer':
            act = body.get('action')
            if act == 'load':
                state['wafer'] = {'present': True,
                                  'offset': body.get('offset') or [0, 0]}
            elif act == 'unload':
                state['wafer'] = {'present': False, 'offset': [0, 0]}
            else:
                return self._json({'ok': False, 'error': 'action 应为 load|unload'})
            return self._json({'ok': True, 'wafer': state['wafer']})
        elif self.path == '/api/workpiece/reset':
            state['wafer'] = {'present': False, 'offset': [0, 0]}
            return self._json({'ok': True})
        elif self.path == '/api/interlock':
            state['interlock_enabled'] = bool(body.get('enabled'))
            return self._json({'ok': True, 'interlock_enabled': state['interlock_enabled']})
        known = ('/api/axis/move', '/api/axis/jog', '/api/axis/home',
                 '/api/stop', '/api/estop', '/api/reset', '/api/speed', '/api/errormaps')
        if self.path in known:
            self._json({'ok': True})
        else:
            self._json({'ok': False, 'error': 'unknown path: %s' % self.path}, 404)


def main():
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print('[mock] 面板 mock 就绪: http://%s:%d/ （Ctrl+C 退出）' % (HOST, PORT), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == '__main__':
    main()
