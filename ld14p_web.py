#!/usr/bin/env python3
"""LDROBOT LD14P 2D LiDAR — live web radar for a headless Raspberry Pi.

Reads the LiDAR over UART and serves a live polar "radar" plot you can open
from any browser on your network — no desktop, monitor, or VNC on the Pi.

    pip install flask pyserial
    python3 ld14p_web.py                       # serial0, then open http://<pi-ip>:8080
    python3 ld14p_web.py /dev/ttyUSB0          # USB-to-serial adapter
    python3 ld14p_web.py --http-port 5000      # serve on a different port
    python3 ld14p_web.py --demo                # synthetic data, no LiDAR needed

This is Part 3 of the LD14P + Raspberry Pi series. It reuses the stream()
generator from Part 1, so keep ld14p_pi.py in the same folder.
"""

import sys
import math
import json
import argparse
import threading

from flask import Flask, Response

import ld14p_pi  # Part 1: stream() generator + DEFAULT_BAUD

DEFAULT_PORT = "/dev/serial0"
DEFAULT_HTTP_PORT = 8080
BINS = 360  # one distance bucket per whole degree


class ScanState:
    """Latest full 360° scan, updated point-by-point as packets arrive."""

    def __init__(self, bins=BINS):
        self.bins = bins
        self.dist = [0] * bins        # mm, 0 = no return
        self.intensity = [0] * bins   # 0-255
        self.rpm = 0.0
        self.error = None
        self.lock = threading.Lock()

    def update(self, angle_deg, dist_mm, inten):
        i = int(angle_deg) % self.bins
        with self.lock:
            self.dist[i] = dist_mm       # store 0 too, so vanished returns clear
            self.intensity[i] = inten

    def snapshot(self):
        with self.lock:
            points = [(a, self.dist[a], self.intensity[a])
                      for a in range(self.bins) if self.dist[a] > 0]
            return {"rpm": round(self.rpm, 1), "points": points, "error": self.error}


def reader(state, port, baud):
    """Background thread: feed the live scan from the real LiDAR."""
    try:
        for speed_dps, _start, _end, _ts, points in ld14p_pi.stream(port, baud):
            state.rpm = speed_dps / 6.0   # deg/s -> rev/min
            for angle, dist, inten in points:
                state.update(angle, dist, inten)
    except Exception as exc:  # surface serial/port problems in the browser HUD
        state.error = f"{type(exc).__name__}: {exc}"


def demo_reader(state):
    """Background thread: synthetic 4 m × 3 m room so you can try the UI
    without a LiDAR attached. Sweeps at ~5 rev/s like a real LD14P."""
    import time
    half_w, half_h = 2000.0, 1500.0          # room half-extents, mm
    angle = 0.0
    step = 0.8                                # degrees between points
    while True:
        pts = []
        # slowly drift the sensor around the room for a bit of life
        t = time.time()
        ox, oy = 600 * math.sin(t * 0.3), 400 * math.cos(t * 0.21)
        for _ in range(12):                   # 12 points per "packet", like the LD14P
            th = math.radians(angle)
            dx, dy = math.sin(th), -math.cos(th)
            ts = []
            if dx > 0:
                ts.append((half_w - ox) / dx)
            elif dx < 0:
                ts.append((-half_w - ox) / dx)
            if dy > 0:
                ts.append((half_h - oy) / dy)
            elif dy < 0:
                ts.append((-half_h - oy) / dy)
            dist = min(c for c in ts if c > 0)
            dist += 15 * math.sin(angle * 0.7)  # gentle wall texture
            state.update(angle % 360, int(dist), 200)
            angle = (angle + step) % 360
        state.rpm = 300.0                      # 5 rev/s
        time.sleep(0.005)


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LD14P LiDAR Radar</title>
<style>
 html,body{margin:0;height:100%;background:#0b0e14;color:#9aa4b2;
   font-family:system-ui,-apple-system,sans-serif;overflow:hidden}
 #hud{position:fixed;top:10px;left:14px;font-size:13px;line-height:1.6}
 #hud b{color:#cdd6e4} .err{color:#ff6b6b}
 canvas{display:block;margin:auto}
</style></head><body>
<div id="hud"><b>LD14P LiDAR</b><br>
 <span id="rpm">– rpm</span><br><span id="cnt">– points</span><br>
 <span id="rng">– m full scale</span><br><span id="err" class="err"></span></div>
<canvas id="c"></canvas>
<script>
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
function resize(){const s=Math.min(innerWidth,innerHeight);cv.width=s;cv.height=s;}
addEventListener('resize',resize);resize();
let maxRange=4000;
function draw(d){
 const W=cv.width,H=cv.height,cx=W/2,cy=H/2,R=Math.min(W,H)/2-12;
 ctx.clearRect(0,0,W,H);
 let mx=0; for(const p of d.points){ if(p[1]>mx) mx=p[1]; }
 if(mx>0){const target=Math.max(1000,Math.ceil(mx/1000)*1000);maxRange+=(target-maxRange)*0.1;}
 ctx.strokeStyle='#1b2230';ctx.fillStyle='#3a4658';ctx.font='11px system-ui';
 for(let k=1;k<=4;k++){const rr=R*k/4;ctx.beginPath();ctx.arc(cx,cy,rr,0,Math.PI*2);ctx.stroke();
   ctx.fillText((maxRange*k/4/1000).toFixed(1)+'m',cx+4,cy-rr+12);}
 ctx.beginPath();ctx.moveTo(cx-R,cy);ctx.lineTo(cx+R,cy);
 ctx.moveTo(cx,cy-R);ctx.lineTo(cx,cy+R);ctx.stroke();
 for(const p of d.points){
   const ang=p[0]*Math.PI/180,rr=Math.min(p[1]/maxRange,1)*R;
   const x=cx+rr*Math.sin(ang),y=cy-rr*Math.cos(ang);
   const g=Math.max(70,Math.min(255,p[2]));
   ctx.fillStyle='rgb('+(255-g)+','+g+',90)';
   ctx.fillRect(x-1.5,y-1.5,3,3);
 }
 ctx.fillStyle='#4da3ff';ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fill();
 document.getElementById('rng').textContent=(maxRange/1000).toFixed(1)+' m full scale';
}
async function tick(){
 try{
  const d=await (await fetch('/scan')).json();
  draw(d);
  document.getElementById('rpm').textContent=d.rpm.toFixed(1)+' rpm';
  document.getElementById('cnt').textContent=d.points.length+' points';
  document.getElementById('err').textContent=d.error||'';
 }catch(e){document.getElementById('err').textContent=e;}
 setTimeout(tick,100);
}
tick();
</script></body></html>"""


app = Flask(__name__)
STATE = ScanState()


@app.route("/")
def index():
    return Response(PAGE, mimetype="text/html")


@app.route("/scan")
def scan():
    return Response(json.dumps(STATE.snapshot()), mimetype="application/json")


def main():
    ap = argparse.ArgumentParser(description="LD14P live web radar")
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT,
                    help="serial port (default /dev/serial0)")
    ap.add_argument("--baud", type=int, default=ld14p_pi.DEFAULT_BAUD)
    ap.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT)
    ap.add_argument("--demo", action="store_true",
                    help="serve synthetic data, no LiDAR required")
    args = ap.parse_args()

    if args.demo:
        target, dargs = demo_reader, (STATE,)
        print("LD14P web radar: DEMO mode (synthetic data)", file=sys.stderr)
    else:
        target, dargs = reader, (STATE, args.port, args.baud)
        print(f"LD14P web radar: reading {args.port} @ {args.baud} baud",
              file=sys.stderr)
    threading.Thread(target=target, args=dargs, daemon=True).start()

    print(f"Open http://<pi-ip>:{args.http_port}/  (Ctrl-C to stop)", file=sys.stderr)
    app.run(host="0.0.0.0", port=args.http_port, threaded=True)


if __name__ == "__main__":
    main()
