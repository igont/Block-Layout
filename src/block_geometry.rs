//! Общая геометрия заготовки и фактических торцов до выпуска изделия.
use crate::{api::{ApiFailure, LayoutRequest}, layout::Block, solid_geometry::{self as solid, Mesh, Plane}};

pub(crate) fn stock(block: &Block, width: f64) -> Result<Vec<Mesh>, ApiFailure> {
    if block.kind.starts_with("node_") || block.cuts.iter().any(|cut|cut.starts_with("Type")) {
        return crate::node_shapes::node_stock(block,width,63.0)
            .map_err(|m|ApiFailure::new("UNSUPPORTED_NODE_SHAPE",m,Some(block.id.clone())));
    }
    let a=(block.rotation_deg as f64).to_radians();
    let u=[a.cos(),a.sin(),0.0];let v=[-a.sin(),a.cos(),0.0];
    Ok(vec![solid::box_mesh([block.start.x as f64/100.0-v[0]*width/2.0,
        block.start.y as f64/100.0-v[1]*width/2.0,block.z_centimm as f64/100.0],
        [u,v,[0.0,0.0,1.0]],[block.length_centimm as f64/100.0,width,63.0])])
}

pub(crate) fn end_cuts(block: &Block) -> Vec<Plane> {
    let a=(block.rotation_deg as f64).to_radians();let u=[a.cos(),a.sin(),0.0];
    let origin=[block.start.x as f64/100.0,block.start.y as f64/100.0,block.z_centimm as f64/100.0];
    let length=block.length_centimm as f64/100.0;
    [(block.natural_end_left(),block.hide_spikes_left(),true),
     (block.natural_end_right(),block.hide_spikes_right(),false)].into_iter()
        .filter(|(natural,hidden,_)|*natural&&*hidden).map(|(_,_,left)| {
            let normal=if left {[-u[0],-u[1],0.0]}else{u};
            let shift=if left {5.0}else{length-5.0};
            Plane {normal,offset:normal[0]*(origin[0]+u[0]*shift)+normal[1]*(origin[1]+u[1]*shift)}
        }).collect()
}

pub(crate) fn subtract_rectangular_cuts(
    mut parts: Vec<Mesh>,
    block: &Block,
    width: f64,
) -> Vec<Mesh> {
    for cut in &block.rectangular_cuts {
        let planes = pocket_planes(block, width, cut);
        parts = parts.iter().flat_map(|part| solid::subtract(part, &planes)).collect();
    }
    parts
}

pub(crate) fn pocket_planes(
    block: &Block,
    width: f64,
    cut: &crate::rectangular_cut::RectangularCut,
) -> [Plane; 6] {
    use crate::rectangular_cut::Side;
    let angle = f64::from(block.rotation_deg).to_radians();
    let (sin, cos) = angle.sin_cos();
    let axes = [[cos, sin, 0.0], [-sin, cos, 0.0], [0.0, 0.0, 1.0]];
    let leading = crate::node_shapes::corner_extensions(block)[0] as f64 / 100.0;
    let origin = [block.start.x as f64 / 100.0 - cos * leading, block.start.y as f64 / 100.0 - sin * leading,
        block.z_centimm as f64 / 100.0];
    let depth = cut.depth_centimm as f64 / 100.0;
    let (low_y, high_y) = match cut.side {
        Side::Left => (width / 2.0 - depth, width / 2.0),
        Side::Right => (-width / 2.0, -width / 2.0 + depth),
    };
    let bounds = [
        (cut.offset_centimm as f64 / 100.0, (cut.offset_centimm + cut.width_centimm) as f64 / 100.0),
        (low_y, high_y),
        (cut.bottom_centimm as f64 / 100.0, (cut.bottom_centimm + cut.height_centimm) as f64 / 100.0),
    ];
    std::array::from_fn(|index| {
        let axis = axes[index / 2];
        let (low, high) = bounds[index / 2];
        let base: f64 = axis.iter().zip(origin).map(|(a, b)| a * b).sum();
        if index % 2 == 0 {
            Plane { normal: axis.map(|a| -a), offset: -(base + low) }
        } else {
            Plane { normal: axis, offset: base + high }
        }
    })
}

pub(crate) fn intersects_pocket(part: &Mesh, planes: &[Plane]) -> bool {
    let mut clipped = solid::clip(part, &planes[0]);
    for plane in &planes[1..] {
        clipped = clipped.and_then(|part| solid::clip(&part, plane));
    }
    clipped.is_some()
}

pub(crate) fn finished(request:&LayoutRequest,block:&Block)->Result<Vec<Mesh>,ApiFailure> {
    if !block.components.is_empty() {
        let mut parts=Vec::new();
        for child in &block.components {parts.extend(finished(request,child)?);}
        return Ok(parts);
    }
    let wall=request.wall_volumes.iter().find(|w|block.source_ids.contains(&format!("wall:{}",w.guid)))
        .ok_or_else(||ApiFailure::new("MISSING_WALL_GEOMETRY","Нет исходной стены детали",Some(block.id.clone())))?;
    let mut parts=stock(block,wall.thickness_mm)?;
    for plane in end_cuts(block) {parts=parts.into_iter().filter_map(|p|solid::clip(&p,&plane)).collect();}
    Ok(subtract_rectangular_cuts(parts, block, wall.thickness_mm))
}
