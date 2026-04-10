use vda5050_vehicle_simulator::config::MapNamePrefixes;
use vda5050_vehicle_simulator::map::parse_opentcs_model_xml;

const MINI: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model version="6.0.0" name="t">
  <point name="A" positionX="0" positionY="0" positionZ="0" type="HALT_POSITION"></point>
  <point name="B" positionX="1000" positionY="0" positionZ="0" type="HALT_POSITION"></point>
  <path name="P1" sourcePoint="A" destinationPoint="B" length="1000" maxVelocity="1000" locked="false"></path>
</model>
"#;

#[test]
fn parse_two_points_and_path() {
    let m = parse_opentcs_model_xml(MINI, 1.0, false).unwrap();
    assert_eq!(m.points.len(), 2);
    assert_eq!(m.paths.len(), 1);
    let p = m.path_for_edge("P1", "A", "B").unwrap();
    assert_eq!(p.polyline_world_m.len(), 2);
    assert!((p.polyline_world_m[0].0 - 0.0).abs() < 1e-6);
    assert!((p.polyline_world_m[1].0 - 1.0).abs() < 1e-6);
}

const PREFIX_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model version="6.0.0" name="t">
  <point name="Point_1" positionX="0" positionY="0" positionZ="0" type="HALT_POSITION"></point>
  <point name="Point_2" positionX="1000" positionY="0" positionZ="0" type="HALT_POSITION"></point>
  <path name="Path_1" sourcePoint="Point_1" destinationPoint="Point_2" length="1000" maxVelocity="1000" locked="false"></path>
</model>
"#;

#[test]
fn numeric_ids_resolve_with_apply_stripping_like_aos_backend() {
    let mut m = parse_opentcs_model_xml(PREFIX_XML, 1.0, false).unwrap();
    m.name_prefixes = MapNamePrefixes {
        apply_stripping: true,
        point_prefix: "Point_".into(),
        path_prefix: "Path_".into(),
    };
    assert!(m.point_world("1").is_some());
    assert!(m.point_world("Point_1").is_some());
    let p = m.path_for_edge("1", "1", "2").unwrap();
    assert_eq!(p.name, "Path_1");
    assert_eq!(p.source, "Point_1");
    assert_eq!(p.dest, "Point_2");
}
