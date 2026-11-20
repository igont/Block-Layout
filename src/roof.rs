//! Исключение целой детали над верхом кровли минус 100 мм по нормали.
//! Частичные пересечения не меняют ни форму, ни размеры детали.
use serde::{Deserialize, Serialize};
use crate::{api::{ApiFailure,LayoutRequest},layout::Block,solid_geometry::{self as solid,Mesh,Plane}};

#[derive(Debug,Clone,Serialize,Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Roof {
    pub id:String,
    pub top_vertices_mm:Vec<[f64;3]>,
    pub top_triangles:Vec<[usize;3]>,
}
struct Cell {planes:Vec<Plane>,low:[f64;2],high:[f64;2]}
fn dot(a:[f64;3],b:[f64;3])->f64 {a[0]*b[0]+a[1]*b[1]+a[2]*b[2]}
fn sub(a:[f64;3],b:[f64;3])->[f64;3] {[a[0]-b[0],a[1]-b[1],a[2]-b[2]]}
fn cells(roofs:&[Roof])->Result<Vec<Cell>,ApiFailure> {
    let mut cells=Vec::new();
    for roof in roofs {
        let invalid=||ApiFailure::new("INVALID_ROOF_GEOMETRY","Не удалось определить верхнюю поверхность кровли",Some(roof.id.clone()));
        if roof.id.is_empty()||roof.top_triangles.is_empty()||roof.top_vertices_mm.iter().flatten().any(|v|!v.is_finite()) {return Err(invalid());}
        for face in &roof.top_triangles {
            if face.iter().any(|i|*i>=roof.top_vertices_mm.len()) {return Err(invalid());}
            let mut p=face.map(|i|roof.top_vertices_mm[i]);
            let u=sub(p[1],p[0]);let v=sub(p[2],p[0]);
            let mut n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
            let length=dot(n,n).sqrt();
            if !length.is_finite()||length==0.0||n[2]==0.0 {return Err(invalid());}
            if n[2]<0.0 {n=n.map(|x|-x);p.swap(1,2);}
            n=n.map(|x|x/length);
            let mut planes=vec![Plane {normal:n.map(|x|-x),offset:100.0-dot(n,p[0])}];
            for i in 0..3 {
                let d=sub(p[(i+1)%3],p[i]);let length=d[0].hypot(d[1]);
                let normal=[d[1]/length,-d[0]/length,0.0];
                planes.push(Plane {normal,offset:dot(normal,p[i])});
            }
            cells.push(Cell {planes,low:[p.iter().map(|v|v[0]).fold(f64::INFINITY,f64::min),p.iter().map(|v|v[1]).fold(f64::INFINITY,f64::min)],
                high:[p.iter().map(|v|v[0]).fold(f64::NEG_INFINITY,f64::max),p.iter().map(|v|v[1]).fold(f64::NEG_INFINITY,f64::max)]});
        }
    }
    Ok(cells)
}
fn fully_above(bodies:Vec<Mesh>,cells:&[Cell])->bool {
    if bodies.is_empty(){return false;}
    for body in bodies {
        let mut remaining=vec![body];
        for cell in cells {
            remaining=remaining.into_iter().flat_map(|part| {
                let (lo,hi)=solid::bounds(&part).expect("Validated block geometry");
                if (0..2).any(|i|hi[i]<cell.low[i]||lo[i]>cell.high[i])
                    ||part.vertices.iter().all(|p|dot(cell.planes[0].normal,*p)>cell.planes[0].offset) {return vec![part];}
                solid::subtract(&part,&cell.planes)
            }).collect();
            if remaining.is_empty(){break;}
        }
        if !remaining.is_empty(){return false;}
    }
    true
}
pub(crate) fn retain_material(request:&LayoutRequest,blocks:Vec<Block>)->Result<(Vec<Block>,Vec<String>),ApiFailure> {
    if request.roof_volumes.is_empty(){return Ok((blocks,Vec::new()));}
    let cells=cells(&request.roof_volumes)?;
    let mut kept=Vec::with_capacity(blocks.len());
    let mut removed=Vec::new();
    for block in blocks {
        if !fully_above(crate::block_geometry::finished(request,&block)?,&cells) {kept.push(block);} else {removed.push(block.id);}
    }
    Ok((kept,removed))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn roof(gx:f64,gy:f64,z:f64)->Roof {
        Roof {id:"r".into(),top_vertices_mm:vec![[0.,-100.,z-100.*gy],[100.,-100.,z+100.*gx-100.*gy],
            [100.,100.,z+100.*gx+100.*gy],[0.,100.,z+100.*gy]],top_triangles:vec![[0,1,2],[0,2,3]]}
    }
    fn body(x:f64,y:f64,z:f64,size:[f64;3])->Mesh {
        solid::box_mesh([x,y,z],[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],size)
    }
    #[test] fn complete_removal_partial_intersection_and_outside_footprint() {
        let cells=cells(&[roof(0.,0.,200.)]).unwrap();
        assert!(fully_above(vec![body(10.,-50.,101.,[80.,100.,63.])],&cells));
        assert!(!fully_above(vec![body(10.,-50.,90.,[80.,100.,63.])],&cells));
        assert!(!fully_above(vec![body(-10.,-50.,101.,[80.,100.,63.])],&cells));
        assert!(!fully_above(vec![body(10.,-50.,101.,[80.,100.,63.]),body(10.,-50.,0.,[80.,100.,63.])],&cells));
    }
    #[test] fn normal_distance_and_full_wall_thickness_are_used() {
        // z=y+300; shifted plane is z=y+300-100*sqrt(2), not z=y+200.
        let cells=cells(&[roof(0.,1.,300.)]).unwrap();
        assert!(fully_above(vec![body(10.,-1.,160.,[80.,2.,10.])],&cells));
        assert!(!fully_above(vec![body(10.,-90.,200.,[80.,180.,10.])],&cells));
    }
    #[test] fn adjacent_triangles_cover_a_body_without_a_false_seam() {
        let cells=cells(&[roof(0.,0.,200.)]).unwrap();
        assert!(fully_above(vec![body(0.,-100.,101.,[100.,200.,63.])],&cells));
    }
    #[test] fn malformed_top_is_reported_with_roof_identity() {
        let mut roof=roof(0.,0.,200.);roof.top_triangles[0][0]=99;
        assert_eq!(cells(&[roof]).err().unwrap().source_id.as_deref(),Some("r"));
    }
}
