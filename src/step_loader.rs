use anyhow::Context;
use rayon::prelude::*;
pub use ruststep::ast::Parameter;
use ruststep::parser::parse;
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::Table;

/// A 4x4 transformation matrix (column-major, like glam)
#[derive(Clone, Copy, Debug)]
pub struct Transform {
    /// Column-major storage: [col0, col1, col2, col3]
    pub cols: [[f64; 4]; 4],
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform {
    pub fn identity() -> Self {
        Self {
            cols: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a transform from AXIS2_PLACEMENT_3D components
    pub fn from_axis2_placement(location: [f64; 3], axis: [f64; 3], ref_dir: [f64; 3]) -> Self {
        // axis is Z direction, ref_dir is X direction
        // Y = Z cross X
        let z = normalize(axis);
        let x = normalize(ref_dir);
        let y = cross(z, x);

        Self {
            cols: [
                [x[0], x[1], x[2], 0.0],
                [y[0], y[1], y[2], 0.0],
                [z[0], z[1], z[2], 0.0],
                [location[0], location[1], location[2], 1.0],
            ],
        }
    }

    /// Multiply two transforms: self * other
    pub fn mul(&self, other: &Transform) -> Transform {
        let mut result = [[0.0; 4]; 4];
        for (i, result_col) in result.iter_mut().enumerate() {
            for (j, result_elem) in result_col.iter_mut().enumerate() {
                for k in 0..4 {
                    *result_elem += self.cols[k][j] * other.cols[i][k];
                }
            }
        }
        Transform { cols: result }
    }

    /// Compute the inverse transform
    pub fn inverse(&self) -> Transform {
        // For a rigid transform (rotation + translation), inverse is:
        // R^-1 = R^T, t^-1 = -R^T * t
        let r00 = self.cols[0][0];
        let r01 = self.cols[1][0];
        let r02 = self.cols[2][0];
        let r10 = self.cols[0][1];
        let r11 = self.cols[1][1];
        let r12 = self.cols[2][1];
        let r20 = self.cols[0][2];
        let r21 = self.cols[1][2];
        let r22 = self.cols[2][2];
        let tx = self.cols[3][0];
        let ty = self.cols[3][1];
        let tz = self.cols[3][2];

        // R^T
        let inv_tx = -(r00 * tx + r10 * ty + r20 * tz);
        let inv_ty = -(r01 * tx + r11 * ty + r21 * tz);
        let inv_tz = -(r02 * tx + r12 * ty + r22 * tz);

        Transform {
            cols: [
                [r00, r01, r02, 0.0],
                [r10, r11, r12, 0.0],
                [r20, r21, r22, 0.0],
                [inv_tx, inv_ty, inv_tz, 1.0],
            ],
        }
    }

    /// Transform a point
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        [
            self.cols[0][0] * p[0]
                + self.cols[1][0] * p[1]
                + self.cols[2][0] * p[2]
                + self.cols[3][0],
            self.cols[0][1] * p[0]
                + self.cols[1][1] * p[1]
                + self.cols[2][1] * p[2]
                + self.cols[3][1],
            self.cols[0][2] * p[0]
                + self.cols[1][2] * p[1]
                + self.cols[2][2] * p[2]
                + self.cols[3][2],
        ]
    }

    /// Transform a normal vector (rotation only, no translation)
    pub fn transform_normal(&self, n: [f64; 3]) -> [f64; 3] {
        [
            self.cols[0][0] * n[0] + self.cols[1][0] * n[1] + self.cols[2][0] * n[2],
            self.cols[0][1] * n[0] + self.cols[1][1] * n[1] + self.cols[2][1] * n[2],
            self.cols[0][2] * n[0] + self.cols[1][2] * n[1] + self.cols[2][2] * n[2],
        ]
    }
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-10 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Preprocess STEP file to join multi-line entities
fn preprocess_step_entities(raw: &str) -> String {
    // STEP entities can span multiple lines, ending with ;
    // Join lines to make parsing easier
    let mut result = String::with_capacity(raw.len());
    for line in raw.lines() {
        let line = line.trim();
        if !line.is_empty() {
            if !result.is_empty() && !result.ends_with(';') {
                result.push(' ');
            }
            result.push_str(line);
        }
    }
    result
}

/// Parse assembly transforms from raw STEP file content.
/// Returns a map from shell entity ID to world transform.
/// Uses foxtrot's approach: build parent->child graph, detect roots, traverse top-down.
fn parse_assembly_transforms(raw: &str) -> HashMap<u64, Transform> {
    // Parse basic geometric entities
    let mut cartesian_points: HashMap<u64, [f64; 3]> = HashMap::new();
    let mut directions: HashMap<u64, [f64; 3]> = HashMap::new();
    let mut placement_refs: HashMap<u64, (u64, u64, u64)> = HashMap::new();
    // ITEM_DEFINED_TRANSFORMATION: id -> (from_placement, to_placement)
    let mut item_transforms: HashMap<u64, (u64, u64)> = HashMap::new();
    // REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION: (rep_1, rep_2, transform_id)
    let mut rep_relationships: Vec<(u64, u64, u64)> = Vec::new();
    // MANIFOLD_SOLID_BREP: manifold_id -> shell_id
    let mut manifold_to_shell: HashMap<u64, u64> = HashMap::new();
    // ADVANCED_BREP_SHAPE_REPRESENTATION: absr_id -> Vec<all refs including manifolds>
    let mut absr_refs: HashMap<u64, Vec<u64>> = HashMap::new();
    // SHAPE_REPRESENTATION_RELATIONSHIP: (rep_1, rep_2) - links reps
    let mut shape_rep_relationships: Vec<(u64, u64)> = Vec::new();
    // SHAPE_REPRESENTATION: id -> Vec<item_refs>
    let mut _shape_reps: HashMap<u64, Vec<u64>> = HashMap::new();

    // Preprocess: join multi-line entities
    let joined = preprocess_step_entities(raw);

    // First pass: collect all entity definitions
    for entity in joined.split(';') {
        let entity = entity.trim();
        let Some(rest) = entity.strip_prefix('#') else {
            continue;
        };
        let Some((id_str, rest)) = rest.split_once('=') else {
            continue;
        };
        let Ok(id) = id_str.trim().parse::<u64>() else {
            continue;
        };
        let rest = rest.trim();

        if rest.starts_with("CARTESIAN_POINT") {
            if let Some(coords) = parse_point_coords(rest) {
                cartesian_points.insert(id, coords);
            }
        } else if rest.starts_with("DIRECTION") {
            if let Some(coords) = parse_point_coords(rest) {
                directions.insert(id, coords);
            }
        } else if rest.starts_with("AXIS2_PLACEMENT_3D") {
            let refs = parse_hash_refs(rest);
            if refs.len() >= 3 {
                placement_refs.insert(id, (refs[0], refs[1], refs[2]));
            }
        } else if rest.starts_with("ITEM_DEFINED_TRANSFORMATION") {
            let refs = parse_hash_refs(rest);
            if refs.len() >= 2 {
                item_transforms.insert(id, (refs[0], refs[1]));
            }
        } else if rest.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
            let refs = parse_hash_refs(rest);
            if refs.len() >= 3 {
                rep_relationships.push((refs[0], refs[1], refs[2]));
            }
        } else if rest.starts_with("ADVANCED_BREP_SHAPE_REPRESENTATION") {
            let refs = parse_hash_refs(rest);
            absr_refs.insert(id, refs);
        } else if rest.starts_with("MANIFOLD_SOLID_BREP") {
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                manifold_to_shell.insert(id, refs[0]);
            }
        } else if rest.starts_with("SHAPE_REPRESENTATION_RELATIONSHIP")
            && !rest.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        {
            let refs = parse_hash_refs(rest);
            if refs.len() >= 2 {
                shape_rep_relationships.push((refs[0], refs[1]));
            }
        } else if rest.starts_with("SHAPE_REPRESENTATION")
            && !rest.starts_with("SHAPE_REPRESENTATION_RELATIONSHIP")
        {
            let refs = parse_hash_refs(rest);
            _shape_reps.insert(id, refs);
        }
    }

    // Resolve AXIS2_PLACEMENT_3D
    let mut resolved_placements: HashMap<u64, Transform> = HashMap::new();
    for (&id, &(loc_id, axis_id, ref_id)) in &placement_refs {
        let location = cartesian_points
            .get(&loc_id)
            .copied()
            .unwrap_or([0.0, 0.0, 0.0]);
        let axis = directions.get(&axis_id).copied().unwrap_or([0.0, 0.0, 1.0]);
        let ref_dir = directions.get(&ref_id).copied().unwrap_or([1.0, 0.0, 0.0]);
        resolved_placements.insert(id, Transform::from_axis2_placement(location, axis, ref_dir));
    }

    log::info!("Assembly parsing:");
    log::info!("  {} cartesian points", cartesian_points.len());
    log::info!("  {} directions", directions.len());
    log::info!("  {} resolved placements", resolved_placements.len());
    log::info!("  {} item_defined_transforms", item_transforms.len());
    log::info!("  {} rep_relationships", rep_relationships.len());
    log::info!("  {} absr_refs entries", absr_refs.len());
    log::info!("  {} manifold_to_shell", manifold_to_shell.len());
    log::info!(
        "  {} shape_rep_relationships",
        shape_rep_relationships.len()
    );

    // ====== FOXTROT-STYLE TOP-DOWN TRANSFORM TRAVERSAL ======

    // Step 1: Build transform stack (parent -> [(child, transform)])
    // Try normal direction first
    let (transform_stack, flipped) =
        build_transform_stack(&rep_relationships, &item_transforms, &resolved_placements);

    log::info!(
        "Transform stack: {} parents, flipped={}",
        transform_stack.len(),
        flipped
    );

    // Step 2: Build shape_rep_relationship map for traversal
    let mut shape_rep_map: HashMap<u64, Vec<u64>> = HashMap::new();
    for &(r1, r2) in &shape_rep_relationships {
        shape_rep_map.entry(r1).or_default().push(r2);
        shape_rep_map.entry(r2).or_default().push(r1);
    }

    // Step 3: Find roots and traverse top-down
    let roots = find_transform_roots(&transform_stack);
    log::info!("Transform roots: {}", roots.len());

    // BFS traversal from roots, accumulating transforms
    let mut rep_transforms: HashMap<u64, Transform> = HashMap::new();
    let mut todo: Vec<(u64, Transform)> = roots
        .into_iter()
        .map(|r| (r, Transform::identity()))
        .collect();

    while let Some((rep_id, mat)) = todo.pop() {
        // Follow shape_rep_relationships (no transform change)
        if let Some(linked) = shape_rep_map.get(&rep_id) {
            for &child in linked {
                if !rep_transforms.contains_key(&child) {
                    todo.push((child, mat));
                }
            }
        }

        // Follow transform_stack (with transform)
        if let Some(children) = transform_stack.get(&rep_id) {
            for &(child, ref child_mat) in children {
                if !rep_transforms.contains_key(&child) {
                    let combined = mat.mul(child_mat);
                    todo.push((child, combined));
                }
            }
        }

        // Store this rep's transform if it's a leaf (ABSR or similar)
        rep_transforms.insert(rep_id, mat);
    }

    log::info!(
        "Computed transforms for {} representations",
        rep_transforms.len()
    );

    // Step 4: Map manifolds to their ABSR's transform
    let mut manifold_to_absr: HashMap<u64, u64> = HashMap::new();
    for (&absr_id, refs) in &absr_refs {
        for &ref_id in refs {
            if manifold_to_shell.contains_key(&ref_id) {
                manifold_to_absr.insert(ref_id, absr_id);
            }
        }
    }

    // Step 5: Build shell -> world transform
    let mut shell_transforms: HashMap<u64, Transform> = HashMap::new();
    for (&manifold_id, &shell_id) in &manifold_to_shell {
        if let Some(&absr_id) = manifold_to_absr.get(&manifold_id)
            && let Some(&transform) = rep_transforms.get(&absr_id)
        {
            if transform.cols[3][0].abs() > 0.001
                || transform.cols[3][1].abs() > 0.001
                || transform.cols[3][2].abs() > 0.001
            {
                log::info!(
                    "Shell #{} transform: translation=({:.2}, {:.2}, {:.2})",
                    shell_id,
                    transform.cols[3][0],
                    transform.cols[3][1],
                    transform.cols[3][2]
                );
            }
            shell_transforms.insert(shell_id, transform);
        }
    }

    // If no transforms found via hierarchy, shells get identity
    if shell_transforms.is_empty() {
        log::info!("No transform hierarchy found, using identity for all shells");
        for &shell_id in manifold_to_shell.values() {
            shell_transforms.insert(shell_id, Transform::identity());
        }
    }

    shell_transforms
}

/// Build transform stack: parent_rep -> [(child_rep, transform)]
/// Returns (stack, was_flipped)
fn build_transform_stack(
    rep_relationships: &[(u64, u64, u64)],
    item_transforms: &HashMap<u64, (u64, u64)>,
    placements: &HashMap<u64, Transform>,
) -> (HashMap<u64, Vec<(u64, Transform)>>, bool) {
    // Try normal direction first (rep_1 is parent, rep_2 is child)
    let stack =
        build_transform_stack_directed(rep_relationships, item_transforms, placements, false);
    let roots = find_transform_roots(&stack);

    // If multiple roots, flip direction (like foxtrot does)
    if roots.len() > 1 {
        log::info!(
            "Multiple roots ({}), flipping transform direction",
            roots.len()
        );
        let flipped_stack =
            build_transform_stack_directed(rep_relationships, item_transforms, placements, true);
        let flipped_roots = find_transform_roots(&flipped_stack);
        if flipped_roots.len() < roots.len() {
            return (flipped_stack, true);
        }
    }

    (stack, false)
}

fn build_transform_stack_directed(
    rep_relationships: &[(u64, u64, u64)],
    item_transforms: &HashMap<u64, (u64, u64)>,
    placements: &HashMap<u64, Transform>,
    flip: bool,
) -> HashMap<u64, Vec<(u64, Transform)>> {
    let mut stack: HashMap<u64, Vec<(u64, Transform)>> = HashMap::new();

    for &(rep_1, rep_2, transform_id) in rep_relationships {
        let (parent, child) = if flip { (rep_1, rep_2) } else { (rep_2, rep_1) };

        // Compute the transform from ITEM_DEFINED_TRANSFORMATION
        let mut mat = compute_item_transform(transform_id, item_transforms, placements);
        if flip {
            mat = mat.inverse();
        }

        stack.entry(parent).or_default().push((child, mat));
    }

    stack
}

/// Find roots: representations that are parents but never children
fn find_transform_roots(stack: &HashMap<u64, Vec<(u64, Transform)>>) -> Vec<u64> {
    let children: std::collections::HashSet<u64> = stack
        .values()
        .flat_map(|v| v.iter().map(|(c, _)| *c))
        .collect();

    stack
        .keys()
        .filter(|k| !children.contains(k))
        .copied()
        .collect()
}

/// Compute transform from ITEM_DEFINED_TRANSFORMATION
/// Formula: t2 * inverse(t1) where t1=transform_item_1, t2=transform_item_2
fn compute_item_transform(
    transform_id: u64,
    item_transforms: &HashMap<u64, (u64, u64)>,
    placements: &HashMap<u64, Transform>,
) -> Transform {
    let Some(&(place_1_id, place_2_id)) = item_transforms.get(&transform_id) else {
        return Transform::identity();
    };

    let t1 = placements.get(&place_1_id).copied().unwrap_or_default();
    let t2 = placements.get(&place_2_id).copied().unwrap_or_default();

    // t2 * inverse(t1): converts from local frame (t1) to target frame (t2)
    t2.mul(&t1.inverse())
}

/// Parse #id references from a STEP entity string
fn parse_hash_refs(s: &str) -> Vec<u64> {
    let mut refs = Vec::new();
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '#' {
            let mut num = String::new();
            while let Some(&d) = chars.peek() {
                if d.is_ascii_digit() {
                    num.push(d);
                    chars.next();
                } else {
                    break;
                }
            }
            if let Ok(id) = num.parse::<u64>() {
                refs.push(id);
            }
        }
    }
    refs
}

/// Parse coordinate values from CARTESIAN_POINT or DIRECTION
/// Format varies: CARTESIAN_POINT('',(x,y,z)) or CARTESIAN_POINT ( '', ( x, y, z ) )
fn parse_point_coords(s: &str) -> Option<[f64; 3]> {
    // Find the coordinate tuple - whitespace varies between files
    // Look for the comma after the name string, then find the opening paren
    let comma_pos = s.find(',')?;
    let after_comma = &s[comma_pos + 1..];
    let paren_pos = after_comma.find('(')?;
    let inner = &after_comma[paren_pos + 1..];
    let end = inner.find(')')?;
    let coords_str = &inner[..end];

    let parts: Vec<&str> = coords_str.split(',').collect();
    if parts.len() < 3 {
        return None;
    }

    let x = parse_step_float(parts[0])?;
    let y = parse_step_float(parts[1])?;
    let z = parse_step_float(parts[2])?;

    Some([x, y, z])
}

/// Parse a STEP float value (handles E notation like "0.E+000")
fn parse_step_float(s: &str) -> Option<f64> {
    let s = s.trim();
    // Handle STEP's weird notation like "0.E+000" (missing digit after decimal)
    let s = if s.contains(".E") {
        s.replace(".E", ".0E")
    } else {
        s.to_string()
    };
    s.parse().ok()
}

/// Apply a transform to all vertices and normals in a mesh
fn apply_transform_to_mesh(mesh: &mut PolygonMesh, transform: &Transform) {
    use truck_meshalgo::prelude::{Faces, Point3, StandardAttributes, Vector3};

    // Transform positions
    let positions: Vec<_> = mesh
        .positions()
        .iter()
        .map(|p| {
            let transformed = transform.transform_point([p.x, p.y, p.z]);
            Point3::new(transformed[0], transformed[1], transformed[2])
        })
        .collect();

    // Transform normals (rotation only)
    let normals: Vec<_> = mesh
        .normals()
        .iter()
        .map(|n| {
            let transformed = transform.transform_normal([n.x, n.y, n.z]);
            // Normalize the result
            let len = (transformed[0] * transformed[0]
                + transformed[1] * transformed[1]
                + transformed[2] * transformed[2])
                .sqrt();
            if len > 1e-10 {
                Vector3::new(
                    transformed[0] / len,
                    transformed[1] / len,
                    transformed[2] / len,
                )
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            }
        })
        .collect();

    // Create new mesh with transformed data
    let uv_coords: Vec<_> = mesh.uv_coords().to_vec();
    let tri_faces: Vec<_> = mesh.tri_faces().to_vec();

    *mesh = PolygonMesh::new(
        StandardAttributes {
            positions,
            uv_coords,
            normals,
        },
        Faces::from_tri_and_quad_faces(tri_faces, vec![]),
    );
}

/// Extract boundary edges from a tessellated polygon mesh.
/// Returns edges as pairs of 3D points. Boundary edges appear in only one triangle.
fn extract_mesh_edges(mesh: &PolygonMesh, transform: Option<&Transform>) -> Vec<([f64; 3], [f64; 3])> {
    use std::collections::HashMap;

    let positions = mesh.positions();
    let tri_faces = mesh.tri_faces();

    // Count how many times each edge appears (using sorted vertex indices as key)
    let mut edge_counts: HashMap<(usize, usize), Vec<(usize, usize)>> = HashMap::new();

    for tri in tri_faces {
        let indices = [
            tri[0].pos as usize,
            tri[1].pos as usize,
            tri[2].pos as usize,
        ];
        // Three edges per triangle
        for i in 0..3 {
            let a = indices[i];
            let b = indices[(i + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            edge_counts.entry(key).or_default().push((a, b));
        }
    }

    // Boundary edges appear exactly once
    let mut edges = Vec::new();
    for (key, occurrences) in edge_counts {
        if occurrences.len() == 1 {
            let (a, b) = key;
            let pa = positions[a];
            let pb = positions[b];

            let mut coord_a = [pa.x, pa.y, pa.z];
            let mut coord_b = [pb.x, pb.y, pb.z];

            if let Some(xform) = transform {
                coord_a = xform.transform_point(coord_a);
                coord_b = xform.transform_point(coord_b);
            }

            edges.push((coord_a, coord_b));
        }
    }

    edges
}

/// Parse colors from raw STEP file content.
/// Returns a map from shell entity ID to RGB color.
fn parse_step_colors(raw: &str) -> HashMap<u64, [f32; 3]> {
    let mut colours: HashMap<u64, [f32; 3]> = HashMap::new();
    let mut styled_items: Vec<(Vec<u64>, u64)> = Vec::new(); // (style_refs, target_id)
    let mut fill_area_style_colour_to_colour: HashMap<u64, u64> = HashMap::new();
    let mut fill_area_style_to_fasc: HashMap<u64, u64> = HashMap::new();
    let mut ssfa_to_fas: HashMap<u64, u64> = HashMap::new(); // SURFACE_STYLE_FILL_AREA -> FAS
    let mut sss_to_ssfa: HashMap<u64, Vec<u64>> = HashMap::new(); // SURFACE_SIDE_STYLE -> SSFA
    let mut ssu_to_sss: HashMap<u64, u64> = HashMap::new(); // SURFACE_STYLE_USAGE -> SSS
    let mut psa_to_styles: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut manifold_to_shell: HashMap<u64, u64> = HashMap::new();

    let joined = preprocess_step_entities(raw);

    for entity in joined.split(';') {
        let entity = entity.trim();
        let Some(rest) = entity.strip_prefix('#') else {
            continue;
        };
        let Some((id_str, rest)) = rest.split_once('=') else {
            continue;
        };
        let Ok(id) = id_str.trim().parse::<u64>() else {
            continue;
        };
        let rest = rest.trim();

        if rest.starts_with("COLOUR_RGB") {
            // COLOUR_RGB('',r,g,b) - note: no parens around RGB values
            if let Some(start) = rest.find('(') {
                let inner = &rest[start + 1..];
                if let Some(end) = inner.rfind(')') {
                    // Split by comma, skip the first element (the name string)
                    let params: Vec<&str> = inner[..end].split(',').collect();
                    if params.len() >= 4 {
                        // params[0] is the name (''), params[1..4] are r,g,b
                        if let (Some(r), Some(g), Some(b)) = (
                            parse_step_float(params[1]),
                            parse_step_float(params[2]),
                            parse_step_float(params[3]),
                        ) {
                            colours.insert(id, [r as f32, g as f32, b as f32]);
                        }
                    }
                }
            }
        } else if rest.starts_with("STYLED_ITEM") || rest.starts_with("OVER_RIDING_STYLED_ITEM") {
            // STYLED_ITEM('name',(#style_refs),#target)
            let refs = parse_hash_refs(rest);
            if refs.len() >= 2 {
                let target_id = refs[refs.len() - 1];
                let style_refs: Vec<u64> = refs[..refs.len() - 1].to_vec();
                styled_items.push((style_refs, target_id));
            }
        } else if rest.starts_with("FILL_AREA_STYLE_COLOUR") {
            // FILL_AREA_STYLE_COLOUR('',#colour_ref)
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                fill_area_style_colour_to_colour.insert(id, refs[0]);
            }
        } else if rest.starts_with("FILL_AREA_STYLE") && !rest.starts_with("FILL_AREA_STYLE_COLOUR")
        {
            // FILL_AREA_STYLE('',(#fasc_ref))
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                fill_area_style_to_fasc.insert(id, refs[0]);
            }
        } else if rest.starts_with("SURFACE_STYLE_FILL_AREA") {
            // SURFACE_STYLE_FILL_AREA(#fas_ref)
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                ssfa_to_fas.insert(id, refs[0]);
            }
        } else if rest.starts_with("SURFACE_SIDE_STYLE") {
            // SURFACE_SIDE_STYLE('',(#ssfa_refs))
            let refs = parse_hash_refs(rest);
            sss_to_ssfa.insert(id, refs);
        } else if rest.starts_with("SURFACE_STYLE_USAGE") {
            // SURFACE_STYLE_USAGE(.BOTH.,#sss_ref)
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                ssu_to_sss.insert(id, refs[0]);
            }
        } else if rest.starts_with("PRESENTATION_STYLE_ASSIGNMENT")
            || rest.starts_with("PRESENTATION_STYLE_BY_CONTEXT")
        {
            // PRESENTATION_STYLE_ASSIGNMENT((#style_refs))
            let refs = parse_hash_refs(rest);
            psa_to_styles.insert(id, refs);
        } else if rest.starts_with("MANIFOLD_SOLID_BREP") {
            // MANIFOLD_SOLID_BREP('',#shell)
            let refs = parse_hash_refs(rest);
            if !refs.is_empty() {
                manifold_to_shell.insert(id, refs[0]);
            }
        }
    }

    // Build color chain: follow references to find RGB values
    // FASC -> COLOUR_RGB
    let mut fasc_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&fasc_id, &colour_id) in &fill_area_style_colour_to_colour {
        if let Some(&rgb) = colours.get(&colour_id) {
            fasc_colors.insert(fasc_id, rgb);
        }
    }

    // FAS -> FASC -> COLOUR_RGB
    let mut fas_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&fas_id, &fasc_id) in &fill_area_style_to_fasc {
        if let Some(&rgb) = fasc_colors.get(&fasc_id) {
            fas_colors.insert(fas_id, rgb);
        }
    }

    // SSFA -> FAS -> ... -> COLOUR_RGB
    let mut ssfa_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&ssfa_id, &fas_id) in &ssfa_to_fas {
        if let Some(&rgb) = fas_colors.get(&fas_id) {
            ssfa_colors.insert(ssfa_id, rgb);
        }
    }

    // SSS -> SSFA -> ... -> COLOUR_RGB
    let mut sss_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&sss_id, ssfa_refs) in &sss_to_ssfa {
        for &ssfa_id in ssfa_refs {
            if let Some(&rgb) = ssfa_colors.get(&ssfa_id) {
                sss_colors.insert(sss_id, rgb);
                break;
            }
        }
    }

    // SSU -> SSS -> ... -> COLOUR_RGB
    let mut ssu_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&ssu_id, &sss_id) in &ssu_to_sss {
        if let Some(&rgb) = sss_colors.get(&sss_id) {
            ssu_colors.insert(ssu_id, rgb);
        }
    }

    // PSA -> SSU -> ... -> COLOUR_RGB
    let mut psa_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (&psa_id, style_refs) in &psa_to_styles {
        for &style_id in style_refs {
            if let Some(&rgb) = ssu_colors.get(&style_id) {
                psa_colors.insert(psa_id, rgb);
                break;
            }
        }
    }

    // STYLED_ITEM targets -> shell colors
    let mut shell_colors: HashMap<u64, [f32; 3]> = HashMap::new();
    for (style_refs, target_id) in &styled_items {
        // Find color through style refs
        let mut found_color: Option<[f32; 3]> = None;
        for &style_id in style_refs {
            if let Some(&rgb) = psa_colors.get(&style_id) {
                found_color = Some(rgb);
                break;
            }
        }

        if let Some(rgb) = found_color {
            // Target might be a manifold_brep, map to shell
            if let Some(&shell_id) = manifold_to_shell.get(target_id) {
                shell_colors.insert(shell_id, rgb);
            } else {
                // Target might be the shell directly
                shell_colors.insert(*target_id, rgb);
            }
        }
    }

    log::info!("Color parsing:");
    log::info!("  {} COLOUR_RGB entries", colours.len());
    log::info!("  {} STYLED_ITEM entries", styled_items.len());
    log::info!("  {} shell colors found", shell_colors.len());
    for (&id, &rgb) in &shell_colors {
        log::info!(
            "  Shell #{}: RGB({:.2}, {:.2}, {:.2})",
            id,
            rgb[0],
            rgb[1],
            rgb[2]
        );
    }

    shell_colors
}

/// A named header entry from the STEP file.
#[derive(Clone, Debug)]
pub struct HeaderEntry {
    pub name: String,
    pub parameter: Parameter,
}

/// Metadata pulled from a STEP file header.
#[derive(Clone, Debug, Default)]
pub struct StepMetadata {
    pub headers: Vec<HeaderEntry>,
    pub entity_count: usize,
}

/// A single STEP face (surface) with its tessellated mesh.
#[derive(Clone, Debug)]
pub struct StepFace {
    pub id: usize,
    pub name: String,
    pub mesh: PolygonMesh,
}

/// A STEP shell containing multiple faces.
#[derive(Clone, Debug)]
pub struct StepShell {
    pub id: usize,
    pub name: String,
    pub faces: Vec<StepFace>,
    /// RGB color from STEP file (if any)
    pub color: Option<[f32; 3]>,
    /// Assembly transform (world transform for this shell)
    pub transform: Option<Transform>,
    /// Tessellated boundary edges (each edge is a pair of 3D points)
    pub edges: Vec<([f64; 3], [f64; 3])>,
}

/// Full scene extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct StepScene {
    pub metadata: StepMetadata,
    pub shells: Vec<StepShell>,
}

/// Progress state for loading - stores (current, total) as packed u32s
#[derive(Clone, Default)]
pub struct LoadProgress {
    /// Packed as (current << 16) | total
    packed: Arc<AtomicU32>,
}

impl LoadProgress {
    pub fn new() -> Self {
        Self {
            packed: Arc::new(AtomicU32::new(0)),
        }
    }

    pub fn set(&self, current: u16, total: u16) {
        let packed = ((current as u32) << 16) | (total as u32);
        self.packed.store(packed, Ordering::Relaxed);
    }

    pub fn get(&self) -> (u16, u16) {
        let packed = self.packed.load(Ordering::Relaxed);
        ((packed >> 16) as u16, (packed & 0xFFFF) as u16)
    }

    pub fn fraction(&self) -> f32 {
        let (current, total) = self.get();
        if total == 0 {
            0.0
        } else {
            current as f32 / total as f32
        }
    }
}

/// Load and tessellate a STEP file into polygon meshes with progress reporting.
pub fn load_step_file_with_progress(
    path: &Path,
    progress: &LoadProgress,
) -> anyhow::Result<StepScene> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read STEP file {}", path.display()))?;

    let exchange = parse(&raw).context("Failed to parse STEP file")?;
    let table = Table::from_data_section(
        exchange
            .data
            .first()
            .context("STEP file has no data sections")?,
    );

    // Extract metadata
    let metadata = StepMetadata {
        headers: exchange
            .header
            .iter()
            .map(|r| HeaderEntry {
                name: r.name.clone(),
                parameter: r.parameter.clone(),
            })
            .collect(),
        entity_count: exchange
            .data
            .iter()
            .map(|section| section.entities.len())
            .sum(),
    };

    // Convert each shell into a triangulated mesh (in parallel)
    let mut shell_entries: Vec<_> = table.shell.iter().collect();
    shell_entries.sort_by_key(|(id, _)| *id);

    let total = shell_entries.len();
    progress.set(0, total as u16);
    let completed = AtomicUsize::new(0);

    let shells: Result<Vec<StepShell>, anyhow::Error> = shell_entries
        .into_par_iter()
        .enumerate()
        .map(|(local_idx, (_id, shell_holder))| {
            let compressed = table
                .to_compressed_shell(shell_holder)
                .map_err(|e| anyhow::anyhow!("Failed to convert STEP shell into topology: {e}"))?;

            // Use a two-pass tolerance to avoid degeneracies on large/small models.
            let coarse = compressed.robust_triangulation(0.01).to_polygon();
            let mut tol = coarse.bounding_box().diameter() * 0.001;
            if !tol.is_normal() {
                tol = 0.01;
            }

            let poly_shell = compressed.robust_triangulation(tol);

            // Extract individual faces and boundary edges from each face mesh
            let mut all_edges: Vec<([f64; 3], [f64; 3])> = Vec::new();
            let faces: Vec<StepFace> = poly_shell
                .faces
                .iter()
                .enumerate()
                .filter_map(|(face_idx, face)| {
                    face.surface.as_ref().map(|surface| {
                        let mesh = match face.orientation {
                            true => surface.clone(),
                            false => surface.inverse(),
                        };
                        // Extract boundary edges from this face's mesh
                        let face_edges = extract_mesh_edges(&mesh, None);
                        all_edges.extend(face_edges);

                        StepFace {
                            id: face_idx,
                            name: format!("Face {}", face_idx + 1),
                            mesh,
                        }
                    })
                })
                .collect();

            let done = completed.fetch_add(1, Ordering::Relaxed) + 1;
            progress.set(done as u16, total as u16);

            Ok(StepShell {
                id: local_idx,
                name: format!("Shell {}", local_idx + 1),
                faces,
                color: None,     // Non-streaming loader doesn't parse colors
                transform: None, // Non-streaming loader doesn't parse assembly transforms
                edges: all_edges,
            })
        })
        .collect();

    let mut shells = shells?;
    // Sort by id to maintain consistent ordering after parallel processing
    shells.sort_by_key(|s| s.id);

    if shells.is_empty() {
        anyhow::bail!("No shells found in STEP file");
    }

    Ok(StepScene { metadata, shells })
}

/// Load and tessellate a STEP file into polygon meshes.
pub fn load_step_file(path: &Path) -> anyhow::Result<StepScene> {
    load_step_file_with_progress(path, &LoadProgress::new())
}

/// Message sent from background loader to main thread
pub enum LoadMessage {
    /// Metadata parsed from STEP header
    Metadata(StepMetadata),
    /// Total number of shells to process
    TotalShells(usize),
    /// Progress update during tessellation (completed, total)
    Progress(usize, usize),
    /// A completed shell
    Shell(StepShell),
    /// Loading finished successfully
    Done,
    /// An error occurred
    Error(String),
}

/// Start loading a STEP file in a background thread, streaming results via channel
/// `tolerance_factor` controls tessellation density (smaller = more triangles, default 0.005)
pub fn load_step_file_streaming(
    path: std::path::PathBuf,
    tolerance_factor: f64,
) -> Receiver<LoadMessage> {
    let (tx, rx) = mpsc::channel();

    std::thread::spawn(move || {
        if let Err(e) = load_step_streaming_inner(&path, &tx, tolerance_factor) {
            let _ = tx.send(LoadMessage::Error(e.to_string()));
        }
    });

    rx
}

fn load_step_streaming_inner(
    path: &Path,
    tx: &Sender<LoadMessage>,
    tolerance_factor: f64,
) -> anyhow::Result<()> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read STEP file {}", path.display()))?;

    // Parse colors from raw STEP content
    let entity_colors = parse_step_colors(&raw);
    log::info!(
        "Parsed {} entity colors from STEP file",
        entity_colors.len()
    );
    for (id, rgb) in &entity_colors {
        log::info!(
            "  Entity #{}: RGB({:.2}, {:.2}, {:.2})",
            id,
            rgb[0],
            rgb[1],
            rgb[2]
        );
    }

    // Parse assembly transforms
    let assembly_transforms = parse_assembly_transforms(&raw);
    log::info!(
        "Parsed {} assembly transforms from STEP file",
        assembly_transforms.len()
    );

    let exchange = parse(&raw).context("Failed to parse STEP file")?;
    let table = Table::from_data_section(
        exchange
            .data
            .first()
            .context("STEP file has no data sections")?,
    );

    // Extract and send metadata
    let metadata = StepMetadata {
        headers: exchange
            .header
            .iter()
            .map(|r| HeaderEntry {
                name: r.name.clone(),
                parameter: r.parameter.clone(),
            })
            .collect(),
        entity_count: exchange
            .data
            .iter()
            .map(|section| section.entities.len())
            .sum(),
    };
    tx.send(LoadMessage::Metadata(metadata))?;

    // Convert each shell into a triangulated mesh (in parallel)
    let mut shell_entries: Vec<_> = table.shell.iter().collect();
    shell_entries.sort_by_key(|(id, _)| *id);

    let total = shell_entries.len();
    tx.send(LoadMessage::TotalShells(total))?;

    // Track progress with atomic counter
    let completed = Arc::new(AtomicUsize::new(0));

    // Process shells in parallel, sending each as it completes (true streaming)
    let error: Arc<std::sync::Mutex<Option<String>>> = Arc::new(std::sync::Mutex::new(None));

    shell_entries
        .into_par_iter()
        .enumerate()
        .for_each(|(local_idx, (shell_id, shell_holder))| {
            // Skip if we already encountered an error
            if error.lock().unwrap().is_some() {
                return;
            }

            // Look up color for this shell's entity ID
            let color = entity_colors.get(shell_id).copied();
            // Look up assembly transform for this shell
            let transform = assembly_transforms.get(shell_id).copied();
            log::info!(
                "Shell {} (entity #{}): color={:?}, transform={:?}",
                local_idx,
                shell_id,
                color,
                transform.map(|t| [t.cols[3][0], t.cols[3][1], t.cols[3][2]])
            );

            let compressed = match table.to_compressed_shell(shell_holder) {
                Ok(c) => c,
                Err(e) => {
                    *error.lock().unwrap() =
                        Some(format!("Failed to convert STEP shell into topology: {e}"));
                    return;
                }
            };

            // Use a two-pass tolerance to avoid degeneracies on large/small models.
            // tolerance_factor controls mesh density (smaller = finer mesh)
            let coarse = compressed.robust_triangulation(0.01).to_polygon();
            let mut tol = coarse.bounding_box().diameter() * tolerance_factor;
            if !tol.is_normal() {
                tol = 0.01;
            }

            let poly_shell = compressed.robust_triangulation(tol);

            // Extract individual faces and apply transform to mesh vertices
            // Also extract boundary edges from each face mesh
            let mut all_edges: Vec<([f64; 3], [f64; 3])> = Vec::new();
            let faces: Vec<StepFace> = poly_shell
                .faces
                .iter()
                .enumerate()
                .filter_map(|(face_idx, face)| {
                    face.surface.as_ref().map(|surface| {
                        let mut mesh = match face.orientation {
                            true => surface.clone(),
                            false => surface.inverse(),
                        };

                        // Extract boundary edges from this face's mesh (before transform is applied to mesh)
                        // Pass transform to extract_mesh_edges so edges are in world coords
                        let face_edges = extract_mesh_edges(&mesh, transform.as_ref());
                        all_edges.extend(face_edges);

                        // Apply assembly transform to mesh vertices and normals
                        if let Some(xform) = transform {
                            apply_transform_to_mesh(&mut mesh, &xform);
                        }

                        StepFace {
                            id: face_idx,
                            name: format!("Face {}", face_idx + 1),
                            mesh,
                        }
                    })
                })
                .collect();

            log::info!("Shell {}: extracted {} boundary edges from {} faces",
                local_idx, all_edges.len(), faces.len());

            let shell = StepShell {
                id: local_idx,
                name: format!("Shell {}", local_idx + 1),
                faces,
                color,
                transform,
                edges: all_edges,
            };

            // Send shell immediately (true streaming)
            let _ = tx.send(LoadMessage::Shell(shell));

            // Update and report progress
            let done = completed.fetch_add(1, Ordering::Relaxed) + 1;
            let _ = tx.send(LoadMessage::Progress(done, total));
        });

    // Check for errors
    if let Some(err) = error.lock().unwrap().take() {
        return Err(anyhow::anyhow!(err));
    }

    tx.send(LoadMessage::Done)?;
    Ok(())
}
