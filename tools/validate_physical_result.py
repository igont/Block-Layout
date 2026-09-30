"""Независимо проверить готовые физические тела, балки и плоские проёмы.

Проверка не строит кладку. SAT применяется к выпуклым телам результата;
касание и проникновение не более 0,01 мм не считаются коллизией.
Наклонные проёмы отмечаются как непроверенные этим контуром.
"""

from __future__ import annotations

import argparse
import collections
import gzip
import json
import math
from pathlib import Path

EPS = 0.01
LIMIT = 20
THIN_HEIGHT_MM = 1.0


def read_json(path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as source:
            return json.load(source)
    return json.loads(path.read_text(encoding="utf-8"))


def normalize_request(request):
    if request.get("format") != "fb-layout/1":
        return request
    walls=[]
    for wall in request["model"]["wall_volumes"]:
        v=wall["volume"]
        walls.append({"guid":wall["id"],"startXmm":v["start_xy_mm"][0],
                      "startYmm":v["start_xy_mm"][1],"endXmm":v["end_xy_mm"][0],
                      "endYmm":v["end_xy_mm"][1],"startBottomZmm":v["bottom_start_mm"],
                      "endBottomZmm":v["bottom_end_mm"],"startTopZmm":v["top_start_mm"],
                      "endTopZmm":v["top_end_mm"],
                      "thicknessMm":v["left_thickness_mm"]+v["right_thickness_mm"]})
    if request["model"].get("openings") or request["model"].get("beams"):
        raise ValueError("Адаптер fb-layout/1 этой проверки поддерживает этап стен")
    return {"snapshot_hash":request["snapshot_hash"],"z0_mm":request["coordinate_system"]["z0_mm"],
            "wall_volumes":walls,"opening_volumes":[],"beams":[]}


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(v):
    norm = math.sqrt(dot(v, v))
    if norm <= 1e-12:
        return None
    return tuple(x / norm for x in v)


def unique_axes(vectors):
    found = {}
    for v in vectors:
        axis = unit(v)
        if axis is None:
            continue
        for component in axis:
            if abs(component) > 1e-9:
                if component < 0:
                    axis = tuple(-x for x in axis)
                break
        found.setdefault(tuple(round(x, 8) for x in axis), axis)
    return list(found.values())


def bounds(vertices):
    return (tuple(min(v[i] for v in vertices) for i in range(3)),
            tuple(max(v[i] for v in vertices) for i in range(3)))


def boxes_overlap(a, b):
    return all(min(a[1][i], b[1][i])-max(a[0][i], b[0][i]) > EPS for i in range(3))


def mesh_geometry(body):
    vertices, faces = body.get("vertices"), body.get("faces")
    if not isinstance(vertices, list) or len(vertices) < 4 or any(
        not isinstance(v, list) or len(v) != 3 or any(
            not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x)
            for x in v) for v in vertices
    ):
        raise ValueError("Некорректные или нечисловые вершины")
    if not isinstance(faces, list) or len(faces) < 4 or any(
        not isinstance(f, list) or len(f) < 3 or any(
            type(i) is not int or not 0 <= i < len(vertices) for i in f) for f in faces
    ):
        raise ValueError("Некорректные грани")
    edges = collections.Counter()
    directed = collections.Counter()
    edge_vectors, normals, planes = [], [], []
    origin, six_volume = vertices[0], 0.0
    for face in faces:
        normal = [0., 0., 0.]
        a = vertices[face[0]]
        for i in range(1, len(face)-1):
            tri_normal = cross(sub(vertices[face[i]], a), sub(vertices[face[i+1]], a))
            normal = [normal[j]+tri_normal[j] for j in range(3)]
            six_volume += dot(sub(a, origin), cross(sub(vertices[face[i]], origin),
                                                    sub(vertices[face[i+1]], origin)))
        n = unit(normal)
        if n is None:
            raise ValueError("Вырожденная грань")
        if any(dot(n, sub(v, a)) > EPS for v in vertices):
            raise ValueError("Невыпуклое тело или внутренняя нормаль грани")
        normals.append(n)
        planes.append((n,dot(n,a)))
        for i, start in enumerate(face):
            end = face[(i+1) % len(face)]
            if start == end:
                raise ValueError("Нулевая кромка")
            edges[tuple(sorted((start, end)))] += 1
            directed[(start, end)] += 1
            edge_vectors.append(sub(vertices[end], vertices[start]))
    if any(count != 2 for count in edges.values()):
        raise ValueError("Незамкнутая сетка: кромка принадлежит не двум граням")
    if any(directed[(a, b)] != 1 or directed[(b, a)] != 1 for a, b in edges):
        raise ValueError("Несогласованное направление соседних граней")
    volume = six_volume / 6
    if not math.isfinite(volume) or volume <= 1e-9:
        raise ValueError("Неположительный ориентированный объём")
    return {"vertices": vertices, "axes": unique_axes(normals),
            "edges": unique_axes(edge_vectors), "bounds": bounds(vertices), "volume": volume,
            "planes":planes}


def oriented_box(origin, axes, sizes):
    if any(not math.isfinite(x) or x <= 0 for x in sizes):
        raise ValueError("Некорректный размер препятствия")
    vertices = [tuple(origin[j] + sum(axes[i][j]*sizes[i]*bit[i] for i in range(3))
                      for j in range(3))
                for bit in ((0,0,0),(1,0,0),(1,1,0),(0,1,0),
                            (0,0,1),(1,0,1),(1,1,1),(0,1,1))]
    normals = unique_axes([cross(axes[1],axes[2]),cross(axes[2],axes[0]),cross(axes[0],axes[1])])
    return {"vertices": vertices, "axes": normals, "edges": axes, "bounds": bounds(vertices)}


def sat_penetration(mesh, box):
    if not boxes_overlap(mesh["bounds"], box["bounds"]):
        return None
    axes = unique_axes(mesh["axes"] + list(box["axes"]) +
                       [cross(a, b) for a in mesh["edges"] for b in box["edges"]])
    penetration = math.inf
    for axis in axes:
        a = [dot(axis, v) for v in mesh["vertices"]]
        b = [dot(axis, v) for v in box["vertices"]]
        depth = min(max(a), max(b)) - max(min(a), min(b))
        if depth <= EPS:
            return None
        penetration = min(penetration, depth)
    return penetration


def beam_box(beam):
    start = (beam["startXmm"], beam["startYmm"], beam["startZmm"])
    end = (beam["endXmm"], beam["endYmm"], beam["endZmm"])
    g = beam["geometry"]
    x = unit(sub(end, start))
    z = unit((g["heightDirectionX"], g["heightDirectionY"], g["heightDirectionZ"]))
    length = math.sqrt(dot(sub(end,start),sub(end,start)))
    if x is None or z is None or abs(dot(x, z))*max(length,g["heightMm"],g["widthMm"]) > EPS:
        raise ValueError("Некорректная система осей балки")
    y = unit(cross(z, x))
    origin = tuple(start[i] - y[i]*g["widthMm"]/2 for i in range(3))
    return oriented_box(origin, (x,y,z), (length,g["widthMm"],g["heightMm"]))


def attached(opening, wall):
    wx, wy = wall["endXmm"]-wall["startXmm"], wall["endYmm"]-wall["startYmm"]
    ox, oy = opening["endXmm"]-opening["startXmm"], opening["endYmm"]-opening["startYmm"]
    wl, ol = math.hypot(wx,wy), math.hypot(ox,oy)
    return wl > 0 and ol > 0 and abs(wx*oy-wy*ox)/(wl*ol) < 1e-7 and abs(
        (opening["startXmm"]-wall["startXmm"])*wy -
        (opening["startYmm"]-wall["startYmm"])*wx)/wl <= EPS


def opening_box(opening, width):
    dx, dy = opening["endXmm"]-opening["startXmm"], opening["endYmm"]-opening["startYmm"]
    length = math.hypot(dx,dy)
    x, y = (dx/length,dy/length,0.), (-dy/length,dx/length,0.)
    origin = (opening["startXmm"]-y[0]*width/2, opening["startYmm"]-y[1]*width/2,
              opening["startBottomZmm"])
    return oriented_box(origin, (x,y,(0.,0.,1.)),
                        (length,width,opening["startTopZmm"]-opening["startBottomZmm"]))


def cap_at(wall, vertex, top):
    dx, dy = wall["endXmm"]-wall["startXmm"], wall["endYmm"]-wall["startYmm"]
    t = ((vertex[0]-wall["startXmm"])*dx+(vertex[1]-wall["startYmm"])*dy)/(dx*dx+dy*dy)
    suffix = "TopZmm" if top else "BottomZmm"
    return wall["start"+suffix] + t*(wall["end"+suffix]-wall["start"+suffix])


def validate(request, result, candidate=None):
    request=normalize_request(request)
    errors, warnings = [], []
    error_counts = collections.Counter()
    def report(code, **details):
        error_counts[code] += 1
        if len(errors) < LIMIT:
            errors.append({"code": code, **details})
    if result.get("format") != "physical_body_v1":
        report("UNSUPPORTED_FORMAT")
    if result.get("snapshot_hash") != request.get("snapshot_hash"):
        report("SNAPSHOT_MISMATCH")
    blocks = result.get("blocks", [])
    if not isinstance(blocks, list) or not blocks:
        report("EMPTY_RESULT")
        blocks = []
    ids = [b.get("id") for b in blocks]
    if any(not isinstance(i, str) or not i for i in ids) or len(set(ids)) != len(ids):
        report("INVALID_BLOCK_IDS")
    if candidate is not None:
        expected = [b.get("id") for b in candidate.get("blocks", [])]
        if collections.Counter(expected) != collections.Counter(ids):
            report("CORE_PHYSICAL_IDS_MISMATCH", core_count=len(expected), physical_count=len(ids))
    walls = {w["guid"]:w for w in request["wall_volumes"]}
    beams = []
    for beam in request.get("beams", []):
        try:
            beams.append((beam["guid"],beam_box(beam)))
        except (ValueError,KeyError,ZeroDivisionError) as error:
            report("INVALID_BEAM", source_id=beam["guid"],message=str(error))
    flat, skipped = [], []
    for o in request.get("opening_volumes", []):
        if o.get("openingType") not in ("OPENING","CONSOLE"):
            continue
        if abs(o["startBottomZmm"]-o["endBottomZmm"])>EPS or abs(o["startTopZmm"]-o["endTopZmm"])>EPS:
            skipped.append(o["guid"])
        else:
            flat.append(o)
    if skipped:
        warnings.append({"code":"SLOPING_OPENINGS_NOT_CHECKED", "source_ids":skipped})
    total_bodies, total_volume, min_height, thin = 0, 0., math.inf, 0
    meshes=[]
    global_bounds = [None,None]
    for block in blocks:
        source_walls = [walls[s.removeprefix("wall:")] for s in block.get("source_ids", [])
                        if s.startswith("wall:") and s.removeprefix("wall:") in walls]
        if not source_walls:
            report("MISSING_SOURCE_WALL", block_id=block.get("id"))
        bodies = block.get("bodies")
        if not isinstance(bodies,list) or not bodies:
            report("MISSING_BODIES",block_id=block.get("id"))
            continue
        opening_boxes = []
        for o in flat:
            widths = {w["thicknessMm"] for w in source_walls if attached(o,w)}
            for width in widths:
                opening_boxes.append((o["guid"],opening_box(o,width)))
        for index, body in enumerate(bodies):
            total_bodies += 1
            detail = {"block_id":block.get("id"),"body_index":index,"course_index":block.get("course_index")}
            try:
                mesh = mesh_geometry(body)
            except (ValueError,TypeError,KeyError) as error:
                report("INVALID_MESH",**detail,message=str(error))
                continue
            low, high = mesh["bounds"]
            height = high[2]-low[2]
            min_height = min(min_height,height)
            thin += height < THIN_HEIGHT_MM
            total_volume += mesh["volume"]
            meshes.append((block,index,mesh))
            if global_bounds[0] is None:
                global_bounds=[list(low),list(high)]
            else:
                global_bounds=[[min(global_bounds[0][i],low[i]) for i in range(3)],
                               [max(global_bounds[1][i],high[i]) for i in range(3)]]
            if source_walls and any(v[2] > max(cap_at(w,v,True) for w in source_walls)+EPS or
                                    v[2] < min(cap_at(w,v,False) for w in source_walls)-EPS
                                    for v in mesh["vertices"]):
                report("WALL_CAP_VIOLATION",**detail)
            if type(block.get("course_index")) is not int:
                report("INVALID_COURSE",**detail)
            else:
                base=request["z0_mm"]+63*block["course_index"]
                if low[2]<base-EPS or high[2]>base+63+EPS:
                    report("COURSE_Z_VIOLATION",**detail,bounds_z_mm=[low[2],high[2]])
            for code, obstacles in (("BEAM_INTERSECTION",beams),("OPENING_INTERSECTION",opening_boxes)):
                for source_id, obstacle in obstacles:
                    depth = sat_penetration(mesh,obstacle)
                    if depth is not None:
                        report(code,**detail,source_id=source_id,min_axis_overlap_mm=depth)
    if thin:
        warnings.append({"code":"THIN_PHYSICAL_FRAGMENTS","count":thin,
                         "threshold_mm":THIN_HEIGHT_MM,"minimum_height_mm":min_height})
    wall_checks=walls_geometry_checks(request,meshes) if not request.get("beams") and not request.get("opening_volumes") else {}
    warnings.extend(wall_checks.pop("warnings",[]))
    return {"status":"failed" if error_counts else "passed", "epsilon_mm":EPS,
            "block_count":len(blocks),"body_count":total_bodies,"volume_mm3":total_volume,
            "bounds_mm":global_bounds,"min_body_height_mm":None if math.isinf(min_height) else min_height,
            "beam_count":len(beams),"flat_opening_count":len(flat),"sloping_opening_count":len(skipped),
            "error_counts":dict(error_counts),"errors":errors,"warnings":warnings,
            "walls_geometry":wall_checks,
            "checks":["core_ids" if candidate is not None else "core_ids_not_requested",
                      "closed_outward_convex_mesh","beam_sat","attached_flat_opening_sat","wall_z_caps"],
            "limitations":["Наклонные проёмы не проверяются SAT этого контура",
                           "Покрытие стен измеряется по осевой линии в сечении каждого ряда"]}


def walls_geometry_checks(request,meshes):
    grid=collections.defaultdict(list)
    signatures={}
    duplicates=[]
    for index,(block,body_index,mesh) in enumerate(meshes):
        low,high=mesh["bounds"]
        course=block["course_index"]
        signature=tuple(sorted(tuple(round(x,6) for x in v) for v in mesh["vertices"]))
        if signature in signatures and signatures[signature][0]!=block["id"]:
            duplicates.append({"block_ids":[signatures[signature][0],block["id"]],"course_index":course})
        signatures[signature]=(block["id"],body_index)
        for x in range(math.floor(low[0]/640),math.floor(high[0]/640)+1):
            for y in range(math.floor(low[1]/640),math.floor(high[1]/640)+1):
                grid[(course,x,y)].append(index)
    tested=set()
    collisions=[]
    collision_count=0
    collision_block_pairs=set()
    pair_counts=collections.Counter()
    for indices in grid.values():
        for i,a in enumerate(indices):
            ba,ia,ma=meshes[a]
            for b in indices[i+1:]:
                pair=tuple(sorted((a,b)))
                if pair in tested:continue
                tested.add(pair)
                bb,ib,mb=meshes[b]
                if ba["id"]==bb["id"]:continue
                depth=sat_penetration(ma,mb)
                if depth is None:continue
                collision_count+=1
                collision_block_pairs.add(tuple(sorted((ba["id"],bb["id"]))))
                key=tuple(sorted((ba["kind"],bb["kind"])))
                pair_counts[" / ".join(key)]+=1
                if len(collisions)<LIMIT:
                    collisions.append({"block_ids":[ba["id"],bb["id"]],"body_indices":[ia,ib],
                                       "course_index":ba["course_index"],"kinds":list(key),
                                       "min_axis_overlap_mm":depth})
    gaps=[]
    gap_count=0
    gap_lengths=[]
    expected_total_length=0.
    checked_sections=0
    wall_summary=[]
    def interval(mesh,start,direction):
        low,high=0.,math.inf
        for normal,offset in mesh["planes"]:
            slope=dot(normal,direction)
            rhs=offset-dot(normal,start)
            if abs(slope)<1e-10:
                if rhs < -EPS:return None
            elif slope>0:high=min(high,rhs/slope)
            else:low=max(low,rhs/slope)
        return (low,high) if high-low>EPS else None
    for wall in request["wall_volumes"]:
        dx,dy=wall["endXmm"]-wall["startXmm"],wall["endYmm"]-wall["startYmm"]
        length=math.hypot(dx,dy)
        direction=(dx/length,dy/length,0.)
        first=math.floor((min(wall["startBottomZmm"],wall["endBottomZmm"])-request["z0_mm"])/63)
        last=math.ceil((max(wall["startTopZmm"],wall["endTopZmm"])-request["z0_mm"])/63)-1
        wall_gap_count=0
        for course in range(first,last+1):
            base=request["z0_mm"]+63*course
            z=base+min(63,max(wall["startTopZmm"],wall["endTopZmm"])-base)/2
            expected_low,expected_high=0.,length
            for value_start,value_end in ((z-wall["startBottomZmm"],z-wall["endBottomZmm"]),
                                           (wall["startTopZmm"]-z,wall["endTopZmm"]-z)):
                slope=(value_end-value_start)/length
                if abs(slope)<1e-10:
                    if value_start < -EPS:expected_high=-1
                elif slope>0:expected_low=max(expected_low,-value_start/slope)
                else:expected_high=min(expected_high,-value_start/slope)
            if expected_high-expected_low<=EPS:continue
            checked_sections+=1
            expected_total_length+=expected_high-expected_low
            indices=set()
            for x in range(math.floor(min(wall["startXmm"],wall["endXmm"])/640),
                           math.floor(max(wall["startXmm"],wall["endXmm"])/640)+1):
                for y in range(math.floor(min(wall["startYmm"],wall["endYmm"])/640),
                               math.floor(max(wall["startYmm"],wall["endYmm"])/640)+1):
                    indices.update(grid.get((course,x,y),[]))
            intervals=[]
            start=(wall["startXmm"],wall["startYmm"],z)
            for index in indices:
                mesh=meshes[index][2]
                if not mesh["bounds"][0][2]-EPS<=z<=mesh["bounds"][1][2]+EPS:continue
                piece=interval(mesh,start,direction)
                if piece is not None:intervals.append(piece)
            cursor=expected_low
            for low,high in sorted(intervals):
                low,high=max(low,expected_low),min(high,expected_high)
                if high<=cursor+EPS:continue
                if low>cursor+EPS:
                    gap_count+=1;wall_gap_count+=1
                    gap_lengths.append(low-cursor)
                    if len(gaps)<LIMIT:gaps.append({"wall_id":wall["guid"],"course_index":course,
                                                   "z_mm":z,"start_u_mm":cursor,"end_u_mm":low,
                                                   "length_mm":low-cursor})
                cursor=max(cursor,high)
            if cursor<expected_high-EPS:
                gap_count+=1;wall_gap_count+=1
                gap_lengths.append(expected_high-cursor)
                if len(gaps)<LIMIT:gaps.append({"wall_id":wall["guid"],"course_index":course,
                                               "z_mm":z,"start_u_mm":cursor,"end_u_mm":expected_high,
                                               "length_mm":expected_high-cursor})
        wall_summary.append({"wall_id":wall["guid"],"first_course":first,"last_course":last,"gap_count":wall_gap_count})
    warnings=[]
    for code,count in (("INTER_BLOCK_INTERSECTIONS",collision_count),("AXIS_COVERAGE_GAPS",gap_count),
                       ("DUPLICATE_PHYSICAL_BODIES",len(duplicates))):
        if count:warnings.append({"code":code,"count":count})
    return {"walls_count":len(request["wall_volumes"]),"checked_course_sections":checked_sections,
            "collision_body_pair_count":collision_count,"collision_kind_counts":dict(pair_counts),
            "collision_block_pair_count":len(collision_block_pairs),
            "collision_block_pair_examples":[list(p) for p in sorted(collision_block_pairs)[:LIMIT]],
            "collision_examples":collisions,"duplicate_body_count":len(duplicates),
            "duplicate_examples":duplicates[:LIMIT],"axis_gap_count":gap_count,"axis_gap_examples":gaps,
            "max_axis_gap_mm":max(gap_lengths,default=0.),"total_axis_gaps_mm":sum(gap_lengths),
            "expected_axis_length_mm":expected_total_length,
            "axis_coverage_percent":100*(1-sum(gap_lengths)/expected_total_length) if expected_total_length else None,
            "axis_gap_over_10mm_count":sum(g>10 for g in gap_lengths),
            "wall_courses":wall_summary,"warnings":warnings}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request",required=True,type=Path)
    parser.add_argument("--result",required=True,type=Path)
    parser.add_argument("--candidate",type=Path)
    args = parser.parse_args()
    outcome = validate(read_json(args.request),read_json(args.result),
                       read_json(args.candidate) if args.candidate else None)
    print(json.dumps(outcome,ensure_ascii=False,allow_nan=False))
    raise SystemExit(0 if outcome["status"]=="passed" else 1)


if __name__ == "__main__":
    main()
