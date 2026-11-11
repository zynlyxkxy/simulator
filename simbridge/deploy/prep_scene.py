# prep_scene.py - prepare 28-model test scene (EN-only)
#   wait 28 ready -> direct wafer load (no EGA) -> flip_pick -> flip_handoff
#   result: wafer on stage + one die on bond head (up/down cameras see marks)
# usage: python prep_scene.py [--host 127.0.0.1] [--port 9010]
#   calls 28 HTTP only; touches no 28/master/relay files.
import json
import sys
import time
import urllib.request

HOST = "127.0.0.1"
PORT = 9010
BASE = "http://%s:%d" % (HOST, PORT)


def http_json(path, body=None, timeout=30):
    url = BASE + path
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def health_ok():
    try:
        j = http_json("/api/health", timeout=3)
        return bool(j.get("ok")) and j.get("axes", 0) > 0
    except Exception:
        return False


def wait_health(seconds=60):
    t0 = time.time()
    while time.time() - t0 < seconds:
        if health_ok():
            print("[prep] 28 ready")
            return True
        time.sleep(1)
    print("[prep] ERROR: 28 not ready in %ds" % seconds)
    return False


def run_flow(name, path, body=None, poll=0.3, idle_timeout=300):
    print("[prep] start: %s (%s)" % (name, path))
    try:
        j = http_json(path, body)
    except Exception as e:
        print("[prep] ERROR call %s failed: %s" % (path, e))
        return False
    print("[prep]   submit -> %s" % json.dumps(j, ensure_ascii=False)[:200])
    t0 = time.time()
    while time.time() - t0 < idle_timeout:
        try:
            st = http_json("/api/flow/state")
        except Exception:
            time.sleep(poll)
            continue
        state = str(st.get("state", ""))
        flow = str(st.get("flow", ""))
        if flow and flow != name and state in ("running", "paused"):
            time.sleep(poll)
            continue
        if state in ("idle", "done", "aborted", "stopped", "failed"):
            steps = st.get("steps") or []
            fails = [s for s in steps if s.get("status") in ("fail", "aborted")]
            if fails:
                print("[prep] ERROR %s failed at step: %s" % (
                    name, json.dumps(fails[0], ensure_ascii=False)[:300]))
            else:
                print("[prep]   finished, state=%s flow=%s" % (state, flow))
            return not fails
        time.sleep(poll)
    print("[prep] ERROR %s idle timeout" % name)
    return False


def set_vision_fast():
    # vision global mode -> FAST (persist? no: runtime-only, but we never
    # restore REAL; prep + downstream flows stay FAST permanently)
    try:
        j = http_json("/api/vision/mode", {"mode": "fast"})
        print("[prep] vision mode -> %s" % j.get("mode"))
        return True
    except Exception as e:
        print("[prep] WARN set vision mode failed: %s" % e)
        return False


def set_speed(scale):
    try:
        j = http_json("/api/speed", {"scale": scale})
        print("[prep] speed_scale -> %s" % j.get("speed_scale"))
        return True
    except Exception as e:
        print("[prep] WARN set speed failed: %s" % e)
        return False


def scene_state():
    try:
        return http_json("/api/workpiece/map")
    except Exception as e:
        return {"error": str(e)}


def wafer_present():
    m = scene_state()
    if "error" in m:
        return None
    w = m.get("wafer") or {}
    return bool(w.get("present"))


def load_wafer_direct():
    # direct wafer load (load/unload API), no EGA
    try:
        j = http_json("/api/workpiece/wafer", {"action": "load"})
        print("[prep] wafer load -> %s" %
              json.dumps(j, ensure_ascii=False)[:200])
        return bool(j.get("ok"))
    except Exception as e:
        print("[prep] wafer load failed: %s" % e)
        return False


def main():
    if not wait_health():
        return 2
    rc = 0
    set_speed(8.0)                       # max speed for scene prep (28 caps at 8x)
    set_vision_fast()                    # vision global mode off (FAST), never restore
    try:
        # 0) direct wafer load, skip EGA; skip if already present
        wp = wafer_present()
        if wp is False:
            if not load_wafer_direct():
                rc = 6
        elif wp is None:
            print("[prep] WARN cannot read wafer state, continue anyway")
        else:
            print("[prep] wafer already present, skip load")
        # 1) flip pick: take one die to flip stage (die_id auto)
        if rc == 0 and not run_flow("flip_pick", "/api/process/flip_pick", {}):
            rc = 4
        # 2) flip handoff: die -> bond head
        if rc == 0 and not run_flow("flip_handoff",
                                    "/api/process/flip_handoff", {}):
            rc = 5
        if rc == 0:
            m = scene_state()
            print("[prep] scene map: %s" %
                  json.dumps(m, ensure_ascii=False)[:400])
            print("[prep] done: wafer on stage + die on bond head")
    finally:
        set_speed(1.0)                   # always restore 1x
    return rc


if __name__ == "__main__":
    sys.exit(main())
