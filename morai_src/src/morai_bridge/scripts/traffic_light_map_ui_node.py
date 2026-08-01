#!/usr/bin/env python3
"""Local browser UI for MORAI traffic-light control.

The map is parsed from the same Lanelet2 OSM file used by Autoware.  Each
``mgeo_signal_id`` is the MORAI Traffic Light Index accepted by /SetTrafficLight.
The existing morai_traffic_light_sender_node forwards that ROS topic to UDP 4000.
"""

import json
import threading
import time
import xml.etree.ElementTree as ET
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from morai_msgs.msg import SetTrafficLight
from rclpy.node import Node


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>MORAI Traffic Light Control</title>
<style>
* { box-sizing: border-box; } body { margin: 0; font-family: system-ui, sans-serif; background:#111827; color:#e5e7eb; }
#layout { display:grid; grid-template-columns: 1fr 320px; height:100vh; }
#map { width:100%; height:100%; display:block; background:#f8fafc; cursor:crosshair; }
#panel { padding:18px; background:#1f2937; overflow:auto; border-left:1px solid #374151; }
h1 { font-size:19px; margin:0 0 16px; } h2 { font-size:14px; margin:22px 0 8px; color:#cbd5e1; }
.selected { padding:10px; border-radius:7px; background:#111827; word-break:break-all; min-height:58px; }
.mode { width:100%; margin:4px 0; padding:10px; border:1px solid #475569; background:#334155; color:white; border-radius:6px; cursor:pointer; }
.mode.active { background:#1d4ed8; border-color:#60a5fa; }
.controls { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
button.status { padding:12px 5px; border:0; border-radius:6px; font-weight:700; cursor:pointer; color:#111827; }
.green{background:#4ade80}.red{background:#f87171}.yellow{background:#facc15}.left{background:#a78bfa}.redleft{background:#fb923c}.greenleft{background:#22d3ee}
#message { margin-top:15px; color:#93c5fd; font-size:13px; line-height:1.45; }
.hint { font-size:12px; color:#94a3b8; line-height:1.5; }
</style></head><body><div id="layout">
<canvas id="map"></canvas><aside id="panel">
<h1>MORAI 신호기 제어</h1>
<div class="hint">점을 클릭해 신호기를 선택한 후 색을 지정하세요.</div>
<h2>대상</h2><div id="selected" class="selected">신호기를 선택하세요</div>
<button id="all" class="mode">전체 신호기 제어 모드</button>
<button id="release-all" class="mode">전체 제어 해제 · 자동 주기 복귀</button>
<h2>상태 전송</h2><div class="controls">
<button class="status green" data-status="16">1 · 녹색</button><button class="status red" data-status="1">2 · 적색</button>
<button class="status yellow" data-status="4">3 · 황색</button><button class="status left" data-status="32">4 · 좌회전</button>
<button class="status redleft" data-status="33">5 · 적색 + 좌회전</button><button class="status greenleft" data-status="48">6 · 녹색 + 좌회전</button>
</div>
<p class="hint">좌클릭 드래그: 회전 · 우클릭 드래그: 이동 · 휠 또는 가운데 드래그: 확대/축소<br>신호기 우클릭: 해당 신호기 제어 해제 · 전체 제어 해제: 모든 신호기를 자동 주기로 복귀</p>
<div id="message"></div></aside></div>
<script>
let data, selected = null, hovered = null, allMode = false, view, markerState = {}, drag = null;
const canvas=document.querySelector('#map'), ctx=canvas.getContext('2d');
const label=document.querySelector('#selected'), allButton=document.querySelector('#all'), message=document.querySelector('#message');
const colors={1:'#dc2626',4:'#eab308',16:'#16a34a',32:'#7c3aed',33:'#f97316',48:'#0891b2'}, statusNames={1:'적색',4:'황색',16:'녹색',32:'좌회전',33:'적색 + 좌회전',48:'녹색 + 좌회전'};
function resize(){ canvas.width=canvas.clientWidth*devicePixelRatio; canvas.height=canvas.clientHeight*devicePixelRatio; ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0); if(data) fit(); }
function project(x,y){const ux=(x-view.cx)*view.scale*view.zoom,uy=-(y-view.cy)*view.scale*view.zoom,c=Math.cos(view.angle),s=Math.sin(view.angle);return [canvas.clientWidth/2+view.panX+ux*c-uy*s,canvas.clientHeight/2+view.panY+ux*s+uy*c];}
function unproject(x,y){const dx=x-(canvas.clientWidth/2+view.panX),dy=y-(canvas.clientHeight/2+view.panY),c=Math.cos(view.angle),s=Math.sin(view.angle),ux=(dx*c+dy*s)/(view.scale*view.zoom),uy=(-dx*s+dy*c)/(view.scale*view.zoom);return {x:view.cx+ux,y:view.cy-uy};}
function draw(){ if(!data) return; const w=canvas.clientWidth,h=canvas.clientHeight; ctx.clearRect(0,0,w,h); ctx.strokeStyle='#cbd5e1';ctx.lineWidth=.55;ctx.beginPath();
 for(const segment of data.lines){let p=project(segment[0][0],segment[0][1]);ctx.moveTo(...p);for(let i=1;i<segment.length;i++){p=project(segment[i][0],segment[i][1]);ctx.lineTo(...p)}}ctx.stroke();
 for(const s of data.signals){const [x,y]=project(s.x,s.y);ctx.beginPath();ctx.arc(x,y,s.id===selected?7:4,0,Math.PI*2);ctx.fillStyle=markerState[s.id]||'#64748b';ctx.fill();if(s.id===selected||s.id===hovered){ctx.strokeStyle='#111827';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#111827';ctx.font='12px sans-serif';ctx.fillText(s.id,x+9,y-8)}} }
function fit(){ const w=canvas.clientWidth,h=canvas.clientHeight,pad=28,dx=data.bounds.maxX-data.bounds.minX,dy=data.bounds.maxY-data.bounds.minY;view={cx:(data.bounds.maxX+data.bounds.minX)/2,cy:(data.bounds.maxY+data.bounds.minY)/2,scale:Math.min((w-pad*2)/dx,(h-pad*2)/dy),zoom:view?.zoom||1,angle:view?.angle||0,panX:view?.panX||0,panY:view?.panY||0};draw(); }
function nearest(e){let best=null,dist=Infinity;for(const s of data.signals){const [x,y]=project(s.x,s.y),d=Math.hypot(x-e.offsetX,y-e.offsetY);if(d<dist){dist=d;best=s}}return {best,dist};}
function select(id){ selected=id;allMode=false;allButton.classList.remove('active');label.textContent=id;draw(); }
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('mousedown',e=>{if(!data)return;drag={button:e.button,x:e.offsetX,y:e.offsetY,moved:false};if(e.button===2)e.preventDefault();});
canvas.addEventListener('mousemove',e=>{if(!data)return;if(drag){const dx=e.offsetX-drag.x,dy=e.offsetY-drag.y;if(Math.abs(dx)+Math.abs(dy)>2)drag.moved=true;if(drag.button===0)view.angle+=dx*.008;else if(drag.button===2){view.panX+=dx;view.panY+=dy;}else if(drag.button===1)view.zoom=Math.max(.15,Math.min(30,view.zoom*Math.exp(-dy*.012)));drag.x=e.offsetX;drag.y=e.offsetY;draw();return;}const {best,dist}=nearest(e),next=dist<20?best.id:null;if(next!==hovered){hovered=next;canvas.style.cursor=next?'pointer':'crosshair';draw();}});
canvas.addEventListener('mouseup',e=>{if(!drag||!data)return;const ended=drag;drag=null;const {best,dist}=nearest(e);if(!ended.moved&&dist<20){if(ended.button===0)select(best.id);if(ended.button===2)release([best.id]).catch(err=>message.textContent='오류: '+err.message);}});
canvas.addEventListener('wheel',e=>{if(!data)return;e.preventDefault();const anchor=unproject(e.offsetX,e.offsetY);view.zoom=Math.max(.15,Math.min(30,view.zoom*Math.exp(-e.deltaY*.001)));const [afterX,afterY]=project(anchor.x,anchor.y);view.panX+=e.offsetX-afterX;view.panY+=e.offsetY-afterY;draw();},{passive:false});
allButton.addEventListener('click',()=>{allMode=!allMode;allButton.classList.toggle('active',allMode);label.textContent=allMode?`전체 ${data.signals.length}개 신호기`:selected||'신호기를 선택하세요';draw();});
async function send(status){if(!allMode&&!selected){message.textContent='먼저 지도에서 신호기를 클릭하세요.';return}const ids=allMode?data.signals.map(s=>s.id):[selected];message.textContent=`${ids.length}개 신호기를 ${statusNames[status]}으로 고정 중…`;const res=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ids,status})});const result=await res.json();if(!res.ok)throw new Error(result.error);ids.forEach(id=>markerState[id]=colors[status]);message.textContent=`${result.held}개 신호기를 ${result.name}으로 유지 중`;draw();}
async function release(ids){const res=await fetch('/api/release',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ids})});const result=await res.json();if(!res.ok)throw new Error(result.error);ids.forEach(id=>delete markerState[id]);message.textContent=`${result.released}개 신호기 제어 해제: MORAI 자동 주기로 복귀`;draw();}
document.querySelectorAll('[data-status]').forEach(b=>b.addEventListener('click',()=>send(Number(b.dataset.status)).catch(e=>message.textContent='오류: '+e.message)));
document.querySelector('#release-all').addEventListener('click',()=>release(data.signals.map(s=>s.id)).catch(e=>message.textContent='오류: '+e.message));
window.addEventListener('keydown',e=>{if(['INPUT','TEXTAREA'].includes(document.activeElement.tagName))return;const map={'1':16,'2':1,'3':4,'4':32,'5':33,'6':48};if(map[e.key])send(map[e.key]).catch(err=>message.textContent='오류: '+err.message)});
fetch('/api/map').then(r=>r.json()).then(result=>{data=result;resize();window.addEventListener('resize',resize);message.textContent=`지도 로드 완료: ${data.signals.length}개 신호기`;}).catch(e=>message.textContent='지도 로드 실패: '+e.message);
</script></body></html>"""

STATUS_NAMES = {
    1: "적색",
    4: "황색",
    16: "녹색",
    32: "좌회전 화살표",
    33: "적색 + 좌회전 화살표",
    48: "녹색 + 좌회전 화살표",
}
DEFAULT_STATUS = -1


def load_map(osm_path):
    nodes, lines, signals = {}, [], []
    for _, element in ET.iterparse(osm_path, events=("end",)):
        if element.tag == "node":
            nodes[element.attrib["id"]] = (
                float(element.attrib["lon"]), float(element.attrib["lat"])
            )
            element.clear()
        elif element.tag == "way":
            tags = {child.attrib.get("k"): child.attrib.get("v") for child in element if child.tag == "tag"}
            points = [nodes[child.attrib["ref"]] for child in element if child.tag == "nd" and child.attrib["ref"] in nodes]
            signal_id = tags.get("mgeo_signal_id")
            if signal_id and points:
                x_values, y_values = zip(*points)
                signals.append({"id": signal_id, "x": sum(x_values) / len(points), "y": sum(y_values) / len(points)})
            elif len(points) >= 2 and tags.get("type") not in {"traffic_light", "light_bulbs"}:
                lines.append(points)
            element.clear()
    if not signals:
        raise RuntimeError(f"mgeo_signal_id traffic lights not found in {osm_path}")
    coordinates = [point for line in lines for point in line]
    x_values, y_values = zip(*coordinates)
    return {"lines": lines, "signals": signals, "bounds": {"minX": min(x_values), "maxX": max(x_values), "minY": min(y_values), "maxY": max(y_values)}}


class TrafficLightMapUi(Node):
    def __init__(self):
        super().__init__("traffic_light_map_ui")
        self.declare_parameter("map_path", "/home/sws/autoware_map/K_city/K_city_26.osm")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 18080)
        self.declare_parameter("output_topic", "/SetTrafficLight")
        self.map_data = load_map(self.get_parameter("map_path").value)
        self.publisher = self.create_publisher(SetTrafficLight, self.get_parameter("output_topic").value, 10)
        self.publish_lock = threading.Lock()
        self.overrides = {}
        self.overrides_lock = threading.Lock()
        self.create_timer(2.0, self.republish_overrides)

    def publish_states(self, ids, status, interval_sec=0.02):
        with self.publish_lock:
            for signal_id in ids:
                message = SetTrafficLight()
                message.traffic_light_index = signal_id
                message.traffic_light_status = status
                self.publisher.publish(message)
                time.sleep(interval_sec)

    def set_overrides(self, ids, status):
        with self.overrides_lock:
            self.overrides.update({signal_id: status for signal_id in ids})
        self.publish_states(ids, status)

    def release_overrides(self, ids):
        with self.overrides_lock:
            released = [signal_id for signal_id in ids if signal_id in self.overrides]
            for signal_id in ids:
                self.overrides.pop(signal_id, None)
        # -1 is MORAI's default state.  It releases the explicit override so
        # the simulator traffic-light plan can resume.
        self.publish_states(ids, DEFAULT_STATUS)
        return released

    def republish_overrides(self):
        with self.overrides_lock:
            grouped = {}
            for signal_id, status in self.overrides.items():
                grouped.setdefault(status, []).append(signal_id)
        for status, ids in grouped.items():
            self.publish_states(ids, status)


def make_handler(node):
    known_ids = {signal["id"] for signal in node.map_data["signals"]}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            node.get_logger().info("UI " + (fmt % args))

        def respond_json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                body = PAGE.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/map":
                self.respond_json(HTTPStatus.OK, node.map_data)
            else:
                self.respond_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self):
            if self.path == "/api/control":
                try:
                    content_length = int(self.headers.get("Content-Length", "0"))
                    request = json.loads(self.rfile.read(content_length))
                    ids = request["ids"]
                    status = int(request["status"])
                    if status not in STATUS_NAMES or not isinstance(ids, list) or not ids:
                        raise ValueError("invalid status or ids")
                    unknown_ids = set(ids) - known_ids
                    if unknown_ids:
                        raise ValueError("unknown signal id: " + ", ".join(sorted(unknown_ids)))
                    node.set_overrides(ids, status)
                    self.respond_json(HTTPStatus.OK, {"held": len(ids), "name": STATUS_NAMES[status]})
                except (ValueError, KeyError, json.JSONDecodeError) as error:
                    self.respond_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
            elif self.path == "/api/release":
                try:
                    content_length = int(self.headers.get("Content-Length", "0"))
                    request = json.loads(self.rfile.read(content_length))
                    ids = request["ids"]
                    if not isinstance(ids, list) or not ids:
                        raise ValueError("invalid ids")
                    unknown_ids = set(ids) - known_ids
                    if unknown_ids:
                        raise ValueError("unknown signal id: " + ", ".join(sorted(unknown_ids)))
                    node.release_overrides(ids)
                    self.respond_json(HTTPStatus.OK, {"released": len(ids)})
                except (ValueError, KeyError, json.JSONDecodeError) as error:
                    self.respond_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
            else:
                self.respond_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    return Handler


def main():
    rclpy.init()
    node = TrafficLightMapUi()
    host = node.get_parameter("host").value
    port = node.get_parameter("port").value
    server = ThreadingHTTPServer((host, port), make_handler(node))
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    node.get_logger().info(f"Traffic-light map UI: http://{host}:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
