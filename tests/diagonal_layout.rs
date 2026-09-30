use fb_layout::constraints::Constraints;
use fb_layout::domain::{RawBuilding, RawPoint, RawWall};
use fb_layout::grid::joint_residue;
use fb_layout::layout::{self, Profile};
use fb_layout::topology::{build_topology, normalize_building};

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/legacy-observed-study.json")).unwrap()
}

fn diagonal(end_mm: f64, bottom_mm: f64, top_mm: f64, reversed: bool) -> RawWall {
    let (start, end) = if reversed {
        (
            RawPoint {
                x_mm: end_mm,
                y_mm: end_mm,
            },
            RawPoint {
                x_mm: 0.0,
                y_mm: 0.0,
            },
        )
    } else {
        (
            RawPoint {
                x_mm: 0.0,
                y_mm: 0.0,
            },
            RawPoint {
                x_mm: end_mm,
                y_mm: end_mm,
            },
        )
    };
    RawWall {
        id: "diagonal".into(),
        start,
        end,
        bottom_start_mm: bottom_mm,
        bottom_end_mm: bottom_mm,
        top_start_mm: top_mm,
        top_end_mm: top_mm,
        thickness_mm: 193.0,
    }
}

fn build(wall: RawWall) -> fb_layout::domain::Topology {
    build_topology(
        normalize_building(RawBuilding {
            z0_mm: 0.0,
            walls: vec![wall],
        })
        .unwrap(),
    )
    .unwrap()
}

#[test]
fn diagonal_ordinary_joints_use_local_axis_and_keep_source() {
    let forward = build(diagonal(960.0, 0.0, 63.0, false));
    let reversed = build(diagonal(960.0, 0.0, 63.0, true));
    let run = &forward.courses[0].runs[0];
    assert_eq!(run.length, 135_765);
    assert_eq!(joint_residue(0, run).unwrap(), None);
    let a = layout::calculate(&forward, &Constraints::default(), &profile()).unwrap();
    let b = layout::calculate(&reversed, &Constraints::default(), &profile()).unwrap();
    assert_eq!(a, b);
    let ordinary = a
        .blocks
        .iter()
        .filter(|block| block.kind == "ordinary")
        .collect::<Vec<_>>();
    assert_eq!(
        ordinary
            .iter()
            .map(|block| block.length_centimm)
            .collect::<Vec<_>>(),
        vec![32_000, 64_000, 39_765]
    );
    assert_eq!(
        ordinary[0].length_centimm + ordinary[1].length_centimm,
        96_000
    );
    assert_eq!(ordinary[2].cuts, vec!["right"]);
    assert_eq!(ordinary[2].catalog_nominal_centimm, Some(64_000));
    assert!(ordinary
        .iter()
        .all(|block| block.course_index == 0 && block.z_centimm == 0));
    assert!(ordinary.iter().all(|block| block
        .source_ids
        .iter()
        .any(|source| source == "wall:diagonal")));
    assert_eq!(ordinary.last().unwrap().end, run.end);
}

#[test]
fn upper_diagonal_start_uses_global_course_and_blank_allowance() {
    let topology = build(diagonal(456.09, 63.0, 126.0, false));
    assert_eq!(topology.courses[0].index, 1);
    assert_eq!(topology.courses[0].z, 6_300);
    assert_eq!(
        joint_residue(1, &topology.courses[0].runs[0]).unwrap(),
        None
    );
    let result = layout::calculate(&topology, &Constraints::default(), &profile()).unwrap();
    assert_eq!(result.blocks.len(), 1);
    let blank = &result.blocks[0];
    assert_eq!(blank.kind, "ordinary");
    assert_eq!(blank.course_index, 1);
    assert_eq!(blank.z_centimm, 6_300);
    assert_eq!(blank.length_centimm, topology.courses[0].runs[0].length);
    assert!(blank.length_centimm > 64_000 && blank.length_centimm <= 65_000);
    assert_eq!(blank.cuts, vec!["right"]);
    assert_eq!(blank.end, topology.courses[0].runs[0].end);
}
