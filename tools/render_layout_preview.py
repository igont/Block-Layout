"""Построить автономное HTML-превью физических тел результата Rust.

Новая раскладка читается только из physical_body_v1; этот инструмент не
рассчитывает кладку и не вычитает отверстия. Наблюдаемая Java-геометрия
преобразуется из сохранённых размеров прямоугольных блоков для сравнения.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path


BOX_FACES = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
             [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]


def read_json(path: Path) -> dict:
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as source:
            return json.load(source)
    return json.loads(path.read_text(encoding="utf-8"))


def validate_body(body: dict) -> dict:
    vertices, faces = body.get("vertices"), body.get("faces")
    if not isinstance(vertices, list) or len(vertices) < 4:
        raise ValueError("Физическое тело не содержит vertices")
    if any(not isinstance(v, list) or len(v) != 3 or
           any(not isinstance(x, (int, float)) or isinstance(x, bool) or
               not math.isfinite(x) for x in v) for v in vertices):
        raise ValueError("Некорректные координаты physical_body_v1")
    if not isinstance(faces, list) or not faces or any(
        not isinstance(f, list) or len(f) < 3 or any(
            type(i) is not int or not 0 <= i < len(vertices) for i in f)
        for f in faces
    ):
        raise ValueError("Некорректные индексы граней physical_body_v1")
    return {"vertices": vertices, "faces": faces}


def physical_blocks(result: dict) -> list[dict]:
    if result.get("format") != "physical_body_v1":
        raise ValueError("Требуется physical_body_v1; логическая раскладка не является физическим телом")
    blocks = result.get("blocks")
    if not isinstance(blocks, list) or not blocks:
        raise ValueError("Физический результат пуст")
    rendered = []
    for block in blocks:
        bodies = block.get("bodies")
        if not isinstance(bodies, list) or not bodies:
            raise ValueError(f"Нет физических тел у блока {block.get('id')}")
        if type(block.get("course_index")) is not int:
            raise ValueError("Не задан индекс ряда")
        rendered.append({"id": str(block["id"]), "kind": str(block["kind"]),
                         "course": block["course_index"],
                         "sources": block.get("source_ids", []),
                         "cut": bool(block.get("cut")),
                         "bodies": [validate_body(body) for body in bodies]})
    return rendered


def observed_blocks(observed: dict, z0: float) -> list[dict]:
    rendered = []
    for block in observed["observed_blocks"]:
        g = block["geometry"]
        c, s = math.cos(g["rotationRad"]), math.sin(g["rotationRad"])
        length, half_width, height = g["lengthMm"], g["widthMm"] / 2, g["heightMm"]
        # origin — начало оси длины, width центрирована относительно этой оси.
        local = [(0, -half_width, 0), (length, -half_width, 0),
                 (length, half_width, 0), (0, half_width, 0),
                 (0, -half_width, height), (length, -half_width, height),
                 (length, half_width, height), (0, half_width, height)]
        vertices = [[g["originXmm"] + x * c - y * s,
                     g["originYmm"] + x * s + y * c,
                     g["originZmm"] + z] for x, y, z in local]
        kind = "bridge" if block.get("isBridge") else "ordinary"
        rendered.append({"id": block["guid"], "kind": kind,
                         "course": round((g["originZmm"] - z0) / height),
                         "sources": [], "cut": bool(block.get("isDobor")),
                         "bodies": [{"vertices": vertices, "faces": BOX_FACES}]})
    return rendered


HTML = r'''<!doctype html>
<html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Баня · физическая раскладка Rust</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101820;color:#e6edf3;font:14px system-ui,sans-serif}
header{padding:16px 22px;border-bottom:1px solid #324150}h1{font-size:21px;margin:0 0 6px}
.muted{color:#aab8c7;font-size:12px}nav{display:flex;gap:15px;align-items:center;flex-wrap:wrap;padding:12px 22px}
label{display:flex;gap:6px;align-items:center}select,button,input{accent-color:#8acbc5}select,button{background:#233440;color:#edf4fa;border:1px solid #536877;padding:6px;border-radius:5px}
input[type=range]{width:125px}button{cursor:pointer}.legend{display:flex;gap:16px;padding:0 22px 10px;color:#bdcad5;font-size:12px}.dot{display:inline-block;width:10px;height:10px;margin-right:5px}
main{position:relative}canvas{display:block;width:100%;height:calc(100vh - 215px);min-height:350px;touch-action:none;cursor:grab}
#detail{position:absolute;left:20px;bottom:16px;white-space:pre-line;background:#101820df;padding:10px;border:1px solid #3b505f;max-width:600px;font-size:12px;pointer-events:none}
#stats{padding:9px 22px;border-top:1px solid #324150}.view-label{position:absolute;top:10px;left:20px;color:#aebfcd;pointer-events:none}
#warnings{margin:0 22px 12px;padding:9px 12px;border:1px solid #697750;background:#243126;font-size:12px;max-height:180px;overflow:auto}#warnings summary{cursor:pointer}#warnings ol{padding-left:22px}#warnings li{padding:5px 0;white-space:pre-line}
</style>
<header><h1 id="title"></h1><div class="muted" id="provenance"></div></header>
<nav>
<label>Раскладка <select id="dataset"><option value="new">Новая · тела с вырезами</option><option value="old">Существующая · номинальные тела</option><option value="compare">Сравнение рядом</option></select></label>
<label>Вид <select id="view"><option value="iso">Изометрия</option><option value="plan">План</option><option value="front">Фасад X / Z</option><option value="side">Фасад Y / Z</option></select></label>
<label>Ряды <input id="lo" type="range" min="0" value="0"><output id="loValue">0</output> — <input id="hi" type="range" min="0"><output id="hiValue"></output></label>
<label><input type="checkbox" id="edges" checked>Границы</label><button id="reset">Сбросить камеру</button><button id="export">Сохранить PNG</button>
</nav>
<details id="warnings"><summary id="warningSummary"></summary><div class="muted" id="geometryNote"></div><ol id="warningList"></ol></details>
<div class="legend"><span><i class="dot" style="background:#cba678"></i>Рядовой</span><span><i class="dot" style="background:#64b7bc"></i>Узел / узловой профиль</span><span><i class="dot" style="background:#be83c4"></i>Перемычка</span><span><i class="dot" style="background:#ed9861"></i>Подрезка</span><span>Перетаскивание: вращение · колесо: масштаб · щелчок: блок</span></div>
<main><canvas id="canvas"></canvas><div id="detail">Отображаются фактически рассчитанные тела. Проёмы образованы отсутствием материала.</div></main><div id="stats"></div>
<script id="data" type="application/json">__DATA__</script>
<script>
'use strict';
const data=JSON.parse(document.getElementById('data').textContent);
const canvas=document.getElementById('canvas'),ctx=canvas.getContext('2d');
const ui=Object.fromEntries(['dataset','view','lo','hi','loValue','hiValue','edges','stats','detail'].map(k=>[k,document.getElementById(k)]));
document.getElementById('title').textContent=data.name+' · физическая раскладка Rust';
document.getElementById('provenance').textContent='Снимок '+data.hash+' · единицы: мм · '+data.resultName;
document.getElementById('geometryNote').textContent=data.wallsOnly?'Первый этап: только стены и узловые профили, без проёмов и балок. Шипы не отображаются.':'Новая геометрия: номинальные тела, узловые профили и вырезы препятствий; шипы не отображаются. Существующая геометрия: номинальные тела.';
if(data.wallsOnly)ui.detail.textContent='Первый этап: готовые тела стен без проёмов и балок.';
if(!data.old.length)for(const option of ui.dataset.options)if(option.value!=='new')option.disabled=true;
document.getElementById('warningSummary').textContent=data.warnings.length?'Расчёт выполнен; замечания: '+data.warnings.length:'Расчёт выполнен; замечания не получены';
for(const warning of data.warnings){const li=document.createElement('li');let text=warning.message||warning.code||String(warning);
 if(warning.course_index!==undefined&&warning.course_index!==null)text+='\nРяд: '+warning.course_index;
 const sources=[...new Set([...(warning.source_ids||[]),...(warning.source_id?[warning.source_id]:[])])];if(sources.length)text+='\nИсходные объекты: '+sources.join(', ');
 if(warning.occurrences>1)text+='\nПовторений: '+warning.occurrences;
 li.textContent=text;document.getElementById('warningList').append(li);}
const all=[...data.new,...data.old];
let bounds=[[Infinity,Infinity,Infinity],[-Infinity,-Infinity,-Infinity]];
for(const b of all)for(const body of b.bodies)for(const v of body.vertices)for(let i=0;i<3;i++){bounds[0][i]=Math.min(bounds[0][i],v[i]);bounds[1][i]=Math.max(bounds[1][i],v[i]);}
const center=bounds[0].map((x,i)=>(x+bounds[1][i])/2),span=Math.max(...bounds[1].map((x,i)=>x-bounds[0][i]));
const maxCourse=Math.max(...all.map(b=>b.course));ui.lo.max=ui.hi.max=maxCourse;ui.hi.value=maxCourse;
let yaw=-.65,pitch=.6,zoom=1,drag=null,hitFaces=[],framePending=false;
function color(b){if(b.kind.includes('bridge')||b.kind.includes('lintel'))return '#be83c4';if(b.kind.includes('node'))return '#64b7bc';return b.cut||b.kind.includes('cut')?'#ed9861':'#cba678';}
function project(v){const x=v[0]-center[0],y=v[1]-center[1],z=v[2]-center[2];
 if(ui.view.value==='plan')return [x,-y,z];if(ui.view.value==='front')return [x,-z,-y];if(ui.view.value==='side')return [y,-z,x];
 const a=x*Math.cos(yaw)-y*Math.sin(yaw),b=x*Math.sin(yaw)+y*Math.cos(yaw);
 return [a,b*Math.sin(pitch)-z*Math.cos(pitch),b*Math.cos(pitch)+z*Math.sin(pitch)];}
function schedule(){if(!framePending){framePending=true;requestAnimationFrame(()=>{framePending=false;draw();});}}
function drawView(blocks,left,width,label){const height=canvas.clientHeight,scale=Math.min(width,height)*.72/span*zoom,faces=[];
 ctx.save();ctx.beginPath();ctx.rect(left,0,width,height);ctx.clip();
 let count=0,bodyCount=0;
 for(const b of blocks){if(b.course<+ui.lo.value||b.course>+ui.hi.value)continue;count++;bodyCount+=b.bodies.length;
  for(const body of b.bodies){const p=body.vertices.map(project);
   for(const f of body.faces){const points=f.map(i=>p[i]);
    const area=points.reduce((a,v,i)=>{const q=points[(i+1)%points.length];return a+v[0]*q[1]-q[0]*v[1];},0);
    if(Math.abs(area)<.0001)continue;
    faces.push({points:points.map(v=>[left+width/2+v[0]*scale,height/2+v[1]*scale]),depth:points.reduce((a,v)=>a+v[2],0)/points.length,b});
   }
  }
 }
 faces.sort((a,b)=>a.depth-b.depth);
 for(const f of faces){ctx.beginPath();f.points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.fillStyle=color(f.b);ctx.fill();if(ui.edges.checked){ctx.strokeStyle='#263641';ctx.lineWidth=.45;ctx.stroke();}}
 hitFaces.push(...faces);ctx.fillStyle='#e7edf2';ctx.font='14px system-ui';ctx.fillText(label,left+20,27);
 ctx.restore();return {count,bodyCount};
}
function draw(){const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
 if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
 ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#17242e';ctx.fillRect(0,0,w,h);hitFaces=[];
 if(+ui.lo.value>+ui.hi.value)ui.hi.value=ui.lo.value;ui.loValue.value=ui.lo.value;ui.hiValue.value=ui.hi.value;
 const mode=ui.dataset.value,counts=[];
 if(mode==='compare'){counts.push(drawView(data.new,0,w/2,'Новая · тела с вырезами'));counts.push(drawView(data.old,w/2,w/2,'Существующая · номинальные тела'));ctx.strokeStyle='#597080';ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(w/2,h);ctx.stroke();}
 else counts.push(drawView(mode==='new'?data.new:data.old,0,w,mode==='new'?'Новая · тела с вырезами':'Существующая · номинальные тела'));
 ui.stats.textContent='Rust: '+data.new.length+' блоков'+(data.old.length?' · Java: '+data.old.length+' блоков':'')+' · показано '+counts.map(c=>c.count+' блоков / '+c.bodyCount+' тел').join(' | ')+' · ряды '+ui.lo.value+'…'+ui.hi.value+' · габариты '+bounds[1].map((x,i)=>Math.round(x-bounds[0][i])).join(' × ')+' мм';
}
function inside(p,poly){let yes=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const a=poly[i],b=poly[j];if((a[1]>p[1])!==(b[1]>p[1])&&p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])yes=!yes;}return yes;}
canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,originX:e.clientX,originY:e.clientY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;if(ui.view.value==='iso'){yaw+=(e.clientX-drag.x)*.007;pitch=Math.max(-1.4,Math.min(1.4,pitch+(e.clientY-drag.y)*.007));}drag.x=e.clientX;drag.y=e.clientY;schedule();});
canvas.addEventListener('pointerup',e=>{if(drag&&Math.hypot(e.clientX-drag.originX,e.clientY-drag.originY)<4){const r=canvas.getBoundingClientRect(),p=[e.clientX-r.left,e.clientY-r.top];for(let i=hitFaces.length-1;i>=0;i--)if(inside(p,hitFaces[i].points)){const b=hitFaces[i].b;ui.detail.textContent=b.id+'\nТип: '+b.kind+' · ряд '+b.course+' · тел '+b.bodies.length+'\n'+b.sources.join(', ');break;}}drag=null;});
canvas.addEventListener('pointercancel',()=>{drag=null;});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(8,zoom*Math.exp(-e.deltaY*.001)));schedule();},{passive:false});
for(const key of ['dataset','view','lo','hi','edges'])ui[key].addEventListener('input',schedule);
document.getElementById('reset').onclick=()=>{yaw=-.65;pitch=.6;zoom=1;schedule();};
document.getElementById('export').onclick=()=>{const a=document.createElement('a');a.download='banya-physical-layout.png';a.href=canvas.toDataURL('image/png');a.click();};
new ResizeObserver(schedule).observe(canvas);draw();window.layoutPreviewReady=true;
</script></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True, type=Path)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--observed", type=Path)
    parser.add_argument("--title")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    request, result = (read_json(p) for p in (args.request, args.result))
    observed=read_json(args.observed) if args.observed else None
    sources=[("результат",result)]+([("наблюдение",observed)] if observed else [])
    for label, source in sources:
        if source.get("snapshot_hash") != request.get("snapshot_hash"):
            raise ValueError(f"Другой snapshot_hash: {label}")
    walls_only=request.get("format")=="fb-layout/1" and not request["model"].get("openings") and not request["model"].get("beams")
    z0=request.get("z0_mm",request.get("coordinate_system",{}).get("z0_mm"))
    payload = {"name": args.title or request.get("project_name") or result.get("project_name") or "Стены Бани",
               "hash": request["snapshot_hash"],"wallsOnly":walls_only,
               "resultName": args.result.name, "warnings": result.get("warnings", []),
               "new": physical_blocks(result),
               "old": observed_blocks(observed, z0) if observed else []}
    # Данные остаются JSON: закрытие script и HTML-метасимволы экранируются.
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    encoded = encoded.replace("&", "\\u0026").replace("<", "\\u003c").replace(">", "\\u003e")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(HTML.replace("__DATA__", encoded), encoding="utf-8")
    print(json.dumps({"output": str(args.output.resolve()), "new_blocks": len(payload["new"]),
                      "observed_blocks": len(payload["old"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
