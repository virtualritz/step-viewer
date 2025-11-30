use bevy::{
    app::AppExit,
    asset::RenderAssetUsages,
    camera::{Viewport, visibility::RenderLayers},
    log::LogPlugin,
    prelude::MessageWriter,
    prelude::*,
    render::render_resource::PrimitiveTopology,
    window::{PresentMode, WindowTheme},
    winit::WinitSettings,
};
use bevy_egui::{
    EguiContexts, EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass, PrimaryEguiContext, egui,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use std::sync::Mutex;
use std::sync::mpsc::Receiver;
use step_viewer::{
    LoadMessage, Parameter, StepMetadata, StepScene, StepShell, load_step_file_streaming,
};
use truck_meshalgo::prelude::PolygonMesh;

#[derive(Resource)]
struct ViewerState {
    pending_path: Option<std::path::PathBuf>,
    loaded_path: Option<std::path::PathBuf>,
    metadata: Option<StepMetadata>,
    shells: Vec<ShellRecord>,
    faces: Vec<FaceRecord>,
    error: Option<String>,
    loading_job: Option<LoadJob>,
    pending_bounds: Option<Bounds>,
    panel_width: f32,
    // Viewport overlay toggles
    show_random_colors: bool,
    show_bounding_box: bool,
    show_wireframe: bool,
    scene_data: Option<StepScene>,
    needs_mesh_rebuild: bool,
    current_bounds: Option<Bounds>,
    /// Tessellation density factor (smaller = more triangles). Range: 0.0005 to 0.02
    tessellation_factor: f64,
    /// Flag to trigger visibility update (avoids costly is_changed() checks)
    visibility_changed: bool,
    /// Scene normalization: original center (for wireframe rendering)
    scene_center: Vec3,
    /// Scene normalization: scale factor (for wireframe rendering)
    scene_scale: f32,
}

impl Default for ViewerState {
    fn default() -> Self {
        Self {
            pending_path: None,
            loaded_path: None,
            metadata: None,
            shells: Vec::new(),
            faces: Vec::new(),
            error: None,
            loading_job: None,
            pending_bounds: None,
            panel_width: 340.0,
            show_random_colors: false,
            show_bounding_box: false,
            show_wireframe: false,
            scene_data: None,
            needs_mesh_rebuild: false,
            current_bounds: None,
            tessellation_factor: 0.001, // Default: matches original hardcoded value
            visibility_changed: false,
            scene_center: Vec3::ZERO,
            scene_scale: 1.0,
        }
    }
}

struct FaceRecord {
    id: usize,
    shell_id: usize,
    name: String,
    triangles: usize,
    visible: bool,
    ui_color: [f32; 3],
    mesh_handle: Handle<Mesh>,
}

struct ShellRecord {
    id: usize,
    name: String,
    expanded: bool,
    face_ids: Vec<usize>, // indices into ViewerState.faces
}

#[derive(Component)]
struct FaceMesh {
    face_id: usize,
}

struct LoadJob {
    path: std::path::PathBuf,
    receiver: Mutex<Receiver<LoadMessage>>,
    current_shell: usize,
    total_shells: usize,
}

#[derive(Clone, Copy, Debug, Default)]
struct Bounds {
    center: Vec3,
    min: Vec3,
    max: Vec3,
}

#[derive(Component)]
struct MainCamera;

/// Render a STEP Parameter as a collapsible tree in egui.
fn parameter_ui(ui: &mut egui::Ui, param: &Parameter, label: &str, depth: usize) {
    match param {
        Parameter::List(items) if !items.is_empty() => {
            egui::CollapsingHeader::new(format!("{} ({})", label, items.len()))
                .id_salt(format!("{}_{}", label, depth))
                .default_open(depth < 1)
                .show(ui, |ui| {
                    for (i, item) in items.iter().enumerate() {
                        parameter_ui(ui, item, &format!("[{}]", i), depth + 1);
                    }
                });
        }
        Parameter::List(_) => {
            ui.label(format!("{}: []", label));
        }
        Parameter::String(s) if s.is_empty() => {
            ui.label(format!("{}: (empty)", label));
        }
        Parameter::String(s) => {
            ui.horizontal_wrapped(|ui| {
                ui.label(format!("{}:", label));
                ui.add(egui::Label::new(s.as_str()).wrap());
            });
        }
        Parameter::Integer(n) => {
            ui.label(format!("{}: {}", label, n));
        }
        Parameter::Real(x) => {
            ui.label(format!("{}: {}", label, x));
        }
        Parameter::Enumeration(e) => {
            ui.label(format!("{}: .{}.", label, e));
        }
        Parameter::Typed { keyword, parameter } => {
            egui::CollapsingHeader::new(format!("{}: {}", label, keyword))
                .id_salt(format!("{}_typed_{}", label, depth))
                .default_open(depth < 2)
                .show(ui, |ui| {
                    parameter_ui(ui, parameter, "value", depth + 1);
                });
        }
        Parameter::Ref(name) => {
            ui.label(format!("{}: {:?}", label, name));
        }
        Parameter::NotProvided => {
            ui.label(format!("{}: $", label));
        }
        Parameter::Omitted => {
            ui.label(format!("{}: *", label));
        }
    }
}

fn main() {
    let cli_path = std::env::args().nth(1).map(std::path::PathBuf::from);

    App::new()
        .insert_resource(ViewerState {
            pending_path: cli_path.clone(),
            ..Default::default()
        })
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "STEP Viewer (Bevy + egui)".into(),
                        present_mode: PresentMode::AutoVsync,
                        fit_canvas_to_parent: true,
                        prevent_default_event_handling: false,
                        window_theme: Some(WindowTheme::Dark),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
                .set(LogPlugin {
                    filter: "info,wgpu_core=warn,wgpu_hal=warn".into(),
                    level: bevy::log::Level::INFO,
                    ..Default::default()
                }),
        )
        .add_plugins(EguiPlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .insert_resource(WinitSettings::desktop_app())
        .add_systems(Startup, setup_scene)
        .add_systems(Update, process_load_requests)
        .add_systems(Update, rebuild_meshes_on_toggle)
        .add_systems(EguiPrimaryContextPass, ui_system)
        .add_systems(Update, normalize_scene_and_setup_camera)
        .add_systems(Update, apply_face_visibility)
        .add_systems(Update, disable_camera_when_egui_wants_input)
        .add_systems(Update, draw_gizmos)
        .run();
}

fn setup_scene(mut commands: Commands, mut egui_global_settings: ResMut<EguiGlobalSettings>) {
    // Disable auto egui context - we create our own camera for it
    egui_global_settings.auto_create_primary_context = false;

    // Ambient light - higher brightness for SSAO scenes (SSAO dims ambient occlusion areas)
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 500.0,
        affects_lightmapped_meshes: false,
    });

    // Main 3D camera with lights as children (so lights move with camera)
    // Camera at ~2 units from origin for viewing unit-sized normalized scene
    commands
        .spawn((
            MainCamera,
            Camera3d::default(),
            Transform::from_xyz(1.5, 1.0, 1.5).looking_at(Vec3::ZERO, Vec3::Y),
            PanOrbitCamera {
                focus: Vec3::ZERO,
                radius: Some(2.0),
                ..Default::default()
            },
        ))
        .with_children(|parent| {
            // Key light - main directional light from upper right (relative to camera)
            parent.spawn((
                DirectionalLight {
                    illuminance: 10000.0,
                    shadows_enabled: true,
                    ..Default::default()
                },
                Transform::from_rotation(Quat::from_euler(
                    EulerRot::ZYX,
                    0.0,
                    std::f32::consts::PI * -0.15,
                    std::f32::consts::PI * -0.15,
                )),
            ));

            // Fill light - softer from the opposite side (relative to camera)
            parent.spawn((
                DirectionalLight {
                    illuminance: 3000.0,
                    shadows_enabled: false,
                    ..Default::default()
                },
                Transform::from_rotation(Quat::from_euler(
                    EulerRot::ZYX,
                    0.0,
                    std::f32::consts::PI * 0.6,
                    std::f32::consts::PI * -0.1,
                )),
            ));
        });

    // Egui-only camera for UI overlay
    commands.spawn((
        PrimaryEguiContext,
        Camera3d::default(),
        RenderLayers::none(),
        Camera {
            order: 1,
            ..Default::default()
        },
    ));
}

fn process_load_requests(
    mut commands: Commands,
    mut state: ResMut<ViewerState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    existing_meshes: Query<Entity, With<FaceMesh>>,
) {
    // Start a new load if requested
    if let Some(path) = state.pending_path.take() {
        for entity in existing_meshes.iter() {
            commands.entity(entity).despawn();
        }
        state.shells.clear();
        state.faces.clear();
        state.metadata = None;
        state.loaded_path = None;
        state.error = None;
        state.scene_data = None;

        let receiver = load_step_file_streaming(path.clone(), state.tessellation_factor);
        state.loading_job = Some(LoadJob {
            path,
            receiver: Mutex::new(receiver),
            current_shell: 0,
            total_shells: 0,
        });
        info!("Started loading STEP file");
    }

    // Poll the loading job for new messages
    let Some(job) = state.loading_job.as_mut() else {
        return;
    };

    // Collect all available messages first (to avoid borrow issues)
    let messages: Vec<_> = {
        let receiver = job.receiver.lock().unwrap();
        receiver.try_iter().collect()
    };

    // Process collected messages
    for msg in messages {
        // Re-borrow job mutably for each message
        let Some(job) = state.loading_job.as_mut() else {
            return;
        };

        match msg {
            LoadMessage::Metadata(meta) => {
                state.metadata = Some(meta);
            }
            LoadMessage::TotalShells(total) => {
                job.total_shells = total;
            }
            LoadMessage::Progress(current, _total) => {
                job.current_shell = current;
            }
            LoadMessage::Shell(shell) => {
                // Store shell in scene_data - don't spawn meshes yet (need bounds first)
                if let Some(scene) = state.scene_data.as_mut() {
                    scene.shells.push(shell);
                } else {
                    state.scene_data = Some(StepScene {
                        metadata: state.metadata.clone().unwrap_or_default(),
                        shells: vec![shell],
                    });
                }
            }
            LoadMessage::Done => {
                let path = job.path.clone();
                state.loaded_path = Some(path);
                state.loading_job = None;

                // Compute bounds for ENTIRE scene
                let bounds = state.scene_data.as_ref().and_then(compute_bounds);

                if let Some(bounds) = bounds {
                    let size = bounds.max - bounds.min;
                    let max_dim = size.x.max(size.y).max(size.z);
                    let scale = if max_dim > 0.0 { 1.0 / max_dim } else { 1.0 };

                    // Store normalization params for wireframe rendering
                    state.scene_center = bounds.center;
                    state.scene_scale = scale;

                    info!(
                        "Scene bounds: center=({:.2}, {:.2}, {:.2}), max_dim={:.2}, scale={:.4}",
                        bounds.center.x, bounds.center.y, bounds.center.z, max_dim, scale
                    );

                    // Now spawn all meshes with normalization applied
                    // Take scene_data temporarily to avoid borrow conflict
                    if let Some(scene) = state.scene_data.take() {
                        for shell in &scene.shells {
                            spawn_shell_faces_normalized(
                                shell,
                                &mut commands,
                                &mut meshes,
                                &mut materials,
                                &mut state,
                                bounds.center,
                                scale,
                            );
                        }
                        state.scene_data = Some(scene);
                    }
                    state.current_bounds = Some(Bounds {
                        center: Vec3::ZERO,
                        min: (bounds.min - bounds.center) * scale,
                        max: (bounds.max - bounds.center) * scale,
                    });
                }

                info!(
                    "Finished loading {} shells, {} faces",
                    state.shells.len(),
                    state.faces.len()
                );
                return;
            }
            LoadMessage::Error(err) => {
                state.error = Some(err);
                state.loading_job = None;
                return;
            }
        }
    }
}

/// Spawn faces for a single shell with normalization applied
fn spawn_shell_faces_normalized(
    shell: &StepShell,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    state: &mut ResMut<ViewerState>,
    scene_center: Vec3,
    scale: f32,
) {
    let use_random_colors = state.show_random_colors;
    let base_face_id = state.faces.len();
    let mut face_ids = Vec::new();

    // Shell color from STEP file (if defined)
    let step_color = shell.color;

    for (idx, face) in shell.faces.iter().enumerate() {
        let global_face_id = base_face_id + idx;

        // For random colors: each face gets its own color based on global_face_id
        // For STEP colors: all faces in shell use the STEP-defined color
        // Otherwise: neutral gray (handled in mesh function)
        let ui_rgb = if let Some(color) = step_color {
            color
        } else {
            let (_, rgb) = color_for_index(global_face_id);
            rgb
        };

        let (mesh, tri_count) = bevy_mesh_from_polygon_normalized(
            &face.mesh,
            ui_rgb,
            use_random_colors || step_color.is_some(),
            scene_center,
            scale,
        );
        let mesh_handle = meshes.add(mesh);

        // Use white base color to show vertex colors
        let material = materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.4,
            metallic: 0.0,
            ..Default::default()
        });

        commands.spawn((
            FaceMesh {
                face_id: global_face_id,
            },
            Mesh3d(mesh_handle.clone()),
            MeshMaterial3d(material),
            Transform::default(),
            Visibility::Visible,
        ));

        state.faces.push(FaceRecord {
            id: global_face_id,
            shell_id: shell.id,
            name: face.name.clone(),
            triangles: tri_count,
            visible: true,
            ui_color: ui_rgb,
            mesh_handle,
        });

        face_ids.push(global_face_id);
    }

    state.shells.push(ShellRecord {
        id: shell.id,
        name: shell.name.clone(),
        expanded: true,
        face_ids,
    });
}

fn compute_bounds(scene: &StepScene) -> Option<Bounds> {
    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    let mut has_points = false;

    for shell in &scene.shells {
        for face in &shell.faces {
            for p in face.mesh.positions() {
                let pos = Vec3::new(p.x as f32, p.y as f32, p.z as f32);
                min = min.min(pos);
                max = max.max(pos);
                has_points = true;
            }
        }
    }

    if !has_points {
        return None;
    }

    let center = (min + max) * 0.5;
    let size = max - min;
    log::info!(
        "Scene bounds: min=({:.2}, {:.2}, {:.2}), max=({:.2}, {:.2}, {:.2}), size=({:.2}, {:.2}, {:.2})",
        min.x,
        min.y,
        min.z,
        max.x,
        max.y,
        max.z,
        size.x,
        size.y,
        size.z
    );
    Some(Bounds { center, min, max })
}

fn bevy_mesh_from_polygon_normalized(
    mesh: &PolygonMesh,
    shell_color: [f32; 3],
    use_random_colors: bool,
    scene_center: Vec3,
    scale: f32,
) -> (Mesh, usize) {
    // Apply normalization: (pos - center) * scale
    let positions: Vec<[f32; 3]> = mesh
        .positions()
        .iter()
        .map(|p| {
            let pos = Vec3::new(p.x as f32, p.y as f32, p.z as f32);
            let normalized = (pos - scene_center) * scale;
            [normalized.x, normalized.y, normalized.z]
        })
        .collect();

    let normals: Vec<[f32; 3]> = mesh
        .normals()
        .iter()
        .map(|n| [n.x as f32, n.y as f32, n.z as f32])
        .collect();

    // Collect vertices as (pos_idx, nor_idx) tuples
    let mut vertices: Vec<(usize, Option<usize>)> = Vec::new();

    for tri in mesh.tri_faces() {
        vertices.extend([
            (tri[0].pos, tri[0].nor),
            (tri[1].pos, tri[1].nor),
            (tri[2].pos, tri[2].nor),
        ]);
    }

    for quad in mesh.quad_faces() {
        vertices.extend([
            (quad[0].pos, quad[0].nor),
            (quad[1].pos, quad[1].nor),
            (quad[2].pos, quad[2].nor),
            (quad[0].pos, quad[0].nor),
            (quad[2].pos, quad[2].nor),
            (quad[3].pos, quad[3].nor),
        ]);
    }

    for face in mesh.other_faces() {
        if face.len() < 3 {
            continue;
        }
        let first = (face[0].pos, face[0].nor);
        face.windows(2).skip(1).for_each(|w| {
            vertices.extend([first, (w[0].pos, w[0].nor), (w[1].pos, w[1].nor)]);
        });
    }

    // Expand indexed geometry to flat arrays
    let (flat_positions, flat_normals): (Vec<[f32; 3]>, Vec<[f32; 3]>) = vertices
        .iter()
        .map(|(pos_idx, nor_idx)| {
            let pos = positions[*pos_idx];
            let nor = nor_idx.map(|ni| normals[ni]).unwrap_or([0.0, 0.0, 1.0]); // Fallback normal
            (pos, nor)
        })
        .unzip();

    // Uniform color per shell: distinct color if random colors enabled, gray otherwise
    let color = if use_random_colors {
        [shell_color[0], shell_color[1], shell_color[2], 1.0]
    } else {
        [0.7, 0.7, 0.7, 1.0]
    };
    let colors: Vec<[f32; 4]> = vec![color; flat_positions.len()];

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, flat_positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, flat_normals);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);

    (bevy_mesh, vertices.len() / 3)
}

fn ui_system(
    mut contexts: EguiContexts,
    mut state: ResMut<ViewerState>,
    mut exit: MessageWriter<AppExit>,
    windows: Query<&Window>,
    mut camera_query: Query<&mut Camera, With<MainCamera>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::TopBottomPanel::top("menu").show(ctx, |ui| {
        ui.horizontal(|ui| {
            if ui.button("Open STEP file…").clicked() {
                #[cfg(not(target_arch = "wasm32"))]
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp", "step"])
                    .pick_file()
                {
                    state.pending_path = Some(path);
                }

                #[cfg(target_arch = "wasm32")]
                {
                    state.error = Some("File open dialog is not supported on wasm".to_string());
                }
            }

            ui.separator();

            // Quality slider (logarithmic scale for better UX)
            // Higher quality = finer mesh (smaller tessellation factor)
            // Quality maps to -log10(tessellation_factor): 1.5 (low) to 4.0 (ultra)
            let mut quality = -state.tessellation_factor.log10();
            ui.label("Quality:");
            let slider = ui.add(
                egui::Slider::new(&mut quality, 1.5_f64..=4.0_f64)
                    .show_value(false)
                    .custom_formatter(|v, _| {
                        if v > 3.5 {
                            "Ultra".to_string()
                        } else if v > 2.8 {
                            "High".to_string()
                        } else if v > 2.2 {
                            "Medium".to_string()
                        } else {
                            "Low".to_string()
                        }
                    }),
            );
            if slider.drag_stopped() && state.loaded_path.is_some() && state.loading_job.is_none() {
                let new_factor = 10_f64.powf(-quality);
                if (new_factor - state.tessellation_factor).abs() > 1e-10 {
                    state.tessellation_factor = new_factor;
                    state.pending_path = state.loaded_path.clone();
                }
            } else if slider.changed() {
                state.tessellation_factor = 10_f64.powf(-quality);
            }
            slider.on_hover_text("Tessellation quality (releases to apply)");

            ui.separator();

            if let Some(path) = &state.loaded_path {
                ui.label(format!("Loaded: {}", path.display()));
            } else {
                ui.label("No file loaded");
            }
        });
    });

    let panel_response = egui::SidePanel::left("entities")
        .default_width(340.0)
        .resizable(true)
        .show(ctx, |ui| {
            ui.heading("Model Hierarchy");
            ui.separator();

            if state.shells.is_empty() && state.loading_job.is_none() {
                ui.label("Load a STEP file to see hierarchy");
            } else if state.shells.is_empty() {
                ui.label("Loading...");
            } else {
                // Track if any visibility checkbox was toggled
                let vis_changed = std::cell::Cell::new(false);

                egui::ScrollArea::vertical().show(ui, |ui| {
                    // We need to collect shell data first to avoid borrow issues
                    let shell_data: Vec<_> = state
                        .shells
                        .iter()
                        .map(|s| (s.id, s.name.clone(), s.expanded, s.face_ids.clone()))
                        .collect();

                    for (shell_id, shell_name, expanded, face_ids) in shell_data {
                        let face_count = face_ids.len();
                        let header = egui::CollapsingHeader::new(format!(
                            "{} ({} faces)",
                            shell_name, face_count
                        ))
                        .id_salt(format!("shell_{}", shell_id))
                        .default_open(expanded);

                        header.show(ui, |ui| {
                            for &face_id in &face_ids {
                                if let Some(face) = state.faces.iter_mut().find(|f| f.id == face_id)
                                {
                                    let color = egui::Color32::from_rgb(
                                        (face.ui_color[0] * 255.0) as u8,
                                        (face.ui_color[1] * 255.0) as u8,
                                        (face.ui_color[2] * 255.0) as u8,
                                    );
                                    ui.horizontal(|ui| {
                                        let prev_visible = face.visible;
                                        ui.checkbox(&mut face.visible, "");
                                        if face.visible != prev_visible {
                                            vis_changed.set(true);
                                        }
                                        ui.colored_label(color, "■");
                                        ui.label(format!(
                                            "{} ({} tris)",
                                            face.name, face.triangles
                                        ));
                                    });
                                }
                            }
                        });
                    }
                });

                // Set visibility_changed flag if any checkbox was toggled
                if vis_changed.get() {
                    state.visibility_changed = true;
                }
            }
        });

    // Track left panel width
    let left_panel_width = panel_response.response.rect.width();

    let right_panel_response = egui::SidePanel::right("metadata")
        .resizable(true)
        .default_width(380.0)
        .show(ctx, |ui| {
            ui.heading("File Information");
            ui.separator();
            if let Some(meta) = &state.metadata {
                ui.label(format!("Entity Count: {}", meta.entity_count));
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for entry in &meta.headers {
                        egui::CollapsingHeader::new(&entry.name)
                            .id_salt(&entry.name)
                            .default_open(entry.name == "FILE_NAME" || entry.name == "FILE_SCHEMA")
                            .show(ui, |ui| {
                                parameter_ui(ui, &entry.parameter, "value", 0);
                            });
                    }
                });
            } else {
                ui.label("No metadata available");
            }
        });

    // Track right panel width
    let right_panel_width = right_panel_response.response.rect.width();
    state.panel_width = left_panel_width;

    // Get window info for viewport overlay positioning
    let window_info = windows.single().ok().map(|w| (w.width(), w.height()));

    // Update camera viewport to account for UI panels
    if let Ok(mut camera) = camera_query.single_mut()
        && let Ok(window) = windows.single()
    {
        let scale_factor = window.scale_factor();
        let left_panel_physical = (left_panel_width * scale_factor) as u32;
        let right_panel_physical = (right_panel_width * scale_factor) as u32;
        let window_width_physical = window.physical_width();
        let window_height_physical = window.physical_height();

        let viewport_width = window_width_physical
            .saturating_sub(left_panel_physical)
            .saturating_sub(right_panel_physical);

        camera.viewport = Some(Viewport {
            physical_position: UVec2::new(left_panel_physical, 0),
            physical_size: UVec2::new(viewport_width, window_height_physical),
            ..Default::default()
        });
    }

    // Show viewport toolbar and overlays
    if let Some((window_width, window_height)) = window_info {
        let viewport_x = left_panel_width;
        let viewport_width = window_width - left_panel_width - right_panel_width;

        // Viewport toolbar (top-right of 3D viewport, not main window)
        if state.scene_data.is_some() {
            let toolbar_margin = 8.0;
            // Position relative to right edge of viewport (before the right panel)
            let toolbar_x = left_panel_width + viewport_width - toolbar_margin;
            let toolbar_y = toolbar_margin + 24.0; // Below menu bar

            egui::Area::new(egui::Id::new("viewport_toolbar"))
                .anchor(egui::Align2::RIGHT_TOP, egui::vec2(0.0, 0.0))
                .fixed_pos(egui::pos2(toolbar_x, toolbar_y))
                .show(ctx, |ui| {
                    ui.visuals_mut().widgets.inactive.weak_bg_fill =
                        egui::Color32::from_rgba_unmultiplied(40, 40, 40, 220);
                    ui.visuals_mut().widgets.hovered.weak_bg_fill =
                        egui::Color32::from_rgba_unmultiplied(60, 60, 60, 230);
                    ui.visuals_mut().widgets.active.weak_bg_fill =
                        egui::Color32::from_rgba_unmultiplied(80, 80, 80, 240);

                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);

                        // Random colors toggle (dice icon: 🎲)
                        let colors_btn = ui.selectable_label(
                            state.show_random_colors,
                            egui::RichText::new("🎲").size(18.0),
                        );
                        if colors_btn.clicked() {
                            state.show_random_colors = !state.show_random_colors;
                            state.needs_mesh_rebuild = true;
                        }
                        colors_btn.on_hover_text("Random colors");

                        // Bounding box toggle (box icon: ⬜)
                        let bbox_btn = ui.selectable_label(
                            state.show_bounding_box,
                            egui::RichText::new("⬜").size(18.0),
                        );
                        if bbox_btn.clicked() {
                            state.show_bounding_box = !state.show_bounding_box;
                        }
                        bbox_btn.on_hover_text("Bounding box");

                        // Wireframe toggle (grid icon: ▦)
                        let wire_btn = ui.selectable_label(
                            state.show_wireframe,
                            egui::RichText::new("▦").size(18.0),
                        );
                        if wire_btn.clicked() {
                            state.show_wireframe = !state.show_wireframe;
                        }
                        wire_btn.on_hover_text("Wireframe edges");
                    });
                });
        }

        if let Some(err) = &state.error {
            // Error overlay
            egui::Area::new(egui::Id::new("error_overlay"))
                .fixed_pos(egui::pos2(viewport_x + 10.0, window_height - 40.0))
                .show(ctx, |ui| {
                    ui.colored_label(egui::Color32::RED, err);
                });
        } else if let Some(job) = &state.loading_job {
            // Progress bar at bottom of viewport
            let bar_height = 24.0;
            let bar_y = window_height - bar_height - 10.0;

            let current = job.current_shell;
            let total = job.total_shells;
            let fraction = if total > 0 {
                current as f32 / total as f32
            } else {
                0.0
            };

            egui::Area::new(egui::Id::new("progress_overlay"))
                .fixed_pos(egui::pos2(viewport_x, bar_y))
                .show(ctx, |ui| {
                    let rect = egui::Rect::from_min_size(
                        ui.cursor().min,
                        egui::vec2(viewport_width, bar_height),
                    );

                    // Background
                    ui.painter().rect_filled(
                        rect,
                        4.0,
                        egui::Color32::from_rgba_unmultiplied(0, 0, 0, 200),
                    );

                    // Progress bar fill
                    if fraction > 0.0 {
                        let progress_rect = egui::Rect::from_min_size(
                            rect.min,
                            egui::vec2(viewport_width * fraction, bar_height),
                        );
                        ui.painter().rect_filled(
                            progress_rect,
                            4.0,
                            egui::Color32::from_rgb(100, 149, 237),
                        );
                    }

                    // Text
                    let text = if total > 0 {
                        format!(
                            "Tessellating shell {}/{} ({:.0}%)",
                            current,
                            total,
                            fraction * 100.0
                        )
                    } else {
                        "Parsing STEP file...".to_string()
                    };

                    ui.painter().text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        text,
                        egui::FontId::proportional(14.0),
                        egui::Color32::WHITE,
                    );
                });

            // Request repaint to update progress
            ctx.request_repaint();
        }
    }

    // Allow escape to quit quickly on desktop
    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        exit.write(AppExit::Success);
    }
}

fn apply_face_visibility(
    mut state: ResMut<ViewerState>,
    mut query: Query<(&FaceMesh, &mut Visibility)>,
) {
    if !state.visibility_changed {
        return;
    }
    state.visibility_changed = false;

    for (mesh, mut visibility) in query.iter_mut() {
        if let Some(record) = state.faces.iter().find(|f| f.id == mesh.face_id) {
            *visibility = if record.visible {
                Visibility::Visible
            } else {
                Visibility::Hidden
            };
        }
    }
}

fn normalize_scene_and_setup_camera(
    mut state: ResMut<ViewerState>,
    mut camera_query: Query<(&mut Transform, &mut PanOrbitCamera), With<MainCamera>>,
    mesh_query: Query<&Transform, (With<FaceMesh>, Without<MainCamera>)>,
) {
    let Some(bounds) = state.pending_bounds else {
        return;
    };

    // Wait until meshes are actually available in the query (ECS delay)
    let mesh_count = mesh_query.iter().count();
    let expected_faces = state.faces.len();
    if mesh_count < expected_faces {
        // Meshes not ready yet, try again next frame
        return;
    }

    // Now we can consume pending_bounds
    state.pending_bounds = None;

    // Calculate scene dimensions
    let size = bounds.max - bounds.min;
    let max_dim = size.x.max(size.y).max(size.z);

    // Store bounds for bounding box gizmo
    state.current_bounds = Some(bounds);

    log::info!(
        "DEBUG: About to setup camera. Bounds center=({:.2}, {:.2}, {:.2}), max_dim={:.2}",
        bounds.center.x,
        bounds.center.y,
        bounds.center.z,
        max_dim
    );

    // Set up camera to view the scene from appropriate distance
    // Use ~1.5x the max dimension for good framing
    let camera_distance = max_dim * 1.5;
    if let Ok((mut transform, mut pan_orbit)) = camera_query.single_mut() {
        log::info!("DEBUG: Found camera, updating PanOrbitCamera");
        pan_orbit.focus = bounds.center;
        pan_orbit.radius = Some(camera_distance);
        pan_orbit.yaw = Some(std::f32::consts::FRAC_PI_4); // 45 degrees
        pan_orbit.pitch = Some(std::f32::consts::FRAC_PI_6); // 30 degrees
        pan_orbit.force_update = true;
        pan_orbit.initialized = false; // Force re-initialization

        // Set initial transform position
        let yaw = std::f32::consts::FRAC_PI_4;
        let pitch = std::f32::consts::FRAC_PI_6;
        let offset = Vec3::new(
            camera_distance * yaw.cos() * pitch.cos(),
            camera_distance * pitch.sin(),
            camera_distance * yaw.sin() * pitch.cos(),
        );
        transform.translation = bounds.center + offset;
        *transform = transform.looking_at(bounds.center, Vec3::Y);

        log::info!(
            "Camera setup: focus=({:.2}, {:.2}, {:.2}), distance={:.2}",
            bounds.center.x,
            bounds.center.y,
            bounds.center.z,
            camera_distance
        );
    } else {
        state.pending_bounds = Some(bounds);
    }
}

fn rebuild_meshes_on_toggle(mut state: ResMut<ViewerState>, mut meshes: ResMut<Assets<Mesh>>) {
    if !state.needs_mesh_rebuild {
        return;
    }
    state.needs_mesh_rebuild = false;

    let Some(scene) = &state.scene_data else {
        return;
    };

    let use_random_colors = state.show_random_colors;

    // Update vertex colors in-place on existing meshes (no despawn/respawn)
    // Iterate through all faces in all shells
    for shell in &scene.shells {
        // STEP-defined colors always show; random colors only when toggle is on
        let apply_colors = use_random_colors || shell.color.is_some();

        for step_face in &shell.faces {
            // Find the corresponding FaceRecord
            if let Some(face_record) = state
                .faces
                .iter()
                .find(|f| f.shell_id == shell.id && f.name == step_face.name)
                && let Some(mesh) = meshes.get_mut(&face_record.mesh_handle)
            {
                let colors =
                    recompute_colors_for_mesh(&step_face.mesh, face_record.ui_color, apply_colors);
                mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
            }
        }
    }
}

fn color_for_index(idx: usize) -> (Color, [f32; 3]) {
    use bevy::color::Hsva;
    // Use golden ratio for hue spread (in degrees for Hsva)
    let hue = (idx as f32 * 0.618_034 * 360.0) % 360.0;
    // Vary saturation and value to distinguish similar hues
    let s = 0.5 + 0.4 * ((idx as f32 * 0.317) % 1.0); // 0.5-0.9
    let v = 0.7 + 0.25 * ((idx as f32 * 0.513) % 1.0); // 0.7-0.95
    let hsva = Hsva::new(hue, s, v, 1.0);
    let color = Color::from(hsva);
    let srgba = color.to_srgba();
    (color, [srgba.red, srgba.green, srgba.blue])
}

/// Recompute vertex colors for a mesh without rebuilding geometry.
/// Returns colors in the same vertex order as bevy_mesh_from_polygon.
fn recompute_colors_for_mesh(
    mesh: &PolygonMesh,
    shell_color: [f32; 3],
    use_random_colors: bool,
) -> Vec<[f32; 4]> {
    // Count total vertices
    let mut vertex_count = 0usize;
    vertex_count += mesh.tri_faces().len() * 3;
    vertex_count += mesh.quad_faces().len() * 6; // 2 triangles per quad
    for face in mesh.other_faces() {
        if face.len() >= 3 {
            vertex_count += (face.len() - 2) * 3;
        }
    }

    // Use shell's distinct color if random colors enabled, otherwise neutral gray
    let color = if use_random_colors {
        [shell_color[0], shell_color[1], shell_color[2], 1.0]
    } else {
        [0.7, 0.7, 0.7, 1.0] // Neutral gray
    };

    vec![color; vertex_count]
}

/// Disable PanOrbitCamera when egui wants pointer input (e.g., during panel resize).
fn disable_camera_when_egui_wants_input(
    mut contexts: EguiContexts,
    mut camera_query: Query<&mut PanOrbitCamera, With<MainCamera>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let egui_wants_input = ctx.wants_pointer_input() || ctx.is_pointer_over_area();

    if let Ok(mut pan_orbit) = camera_query.single_mut() {
        pan_orbit.enabled = !egui_wants_input;
    }
}

/// Draw bounding box and wireframe gizmos when enabled.
fn draw_gizmos(state: Res<ViewerState>, mut gizmos: Gizmos) {
    // Draw wireframe edges (STEP geometry boundary edges stored in scene_data)
    if state.show_wireframe {
        if let Some(scene) = &state.scene_data {
            let color = Color::srgba(0.0, 0.0, 0.0, 0.7);
            let center = state.scene_center;
            let scale = state.scene_scale;

            for shell in &scene.shells {
                for (p0_arr, p1_arr) in &shell.edges {
                    // Apply same normalization as mesh vertices: (pos - center) * scale
                    let p0_raw = Vec3::new(p0_arr[0] as f32, p0_arr[1] as f32, p0_arr[2] as f32);
                    let p1_raw = Vec3::new(p1_arr[0] as f32, p1_arr[1] as f32, p1_arr[2] as f32);
                    let p0 = (p0_raw - center) * scale;
                    let p1 = (p1_raw - center) * scale;
                    gizmos.line(p0, p1, color);
                }
            }
        }
    }

    // Draw bounding box
    if state.show_bounding_box
        && let Some(bounds) = state.current_bounds
    {
        let min = bounds.min;
        let max = bounds.max;
        let color = Color::srgb(0.0, 1.0, 0.0); // Green

        // 12 edges of the bounding box
        // Bottom face
        gizmos.line(
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(max.x, min.y, min.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(max.x, min.y, max.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, min.y, max.z),
            Vec3::new(min.x, min.y, max.z),
            color,
        );
        gizmos.line(
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(min.x, min.y, min.z),
            color,
        );
        // Top face
        gizmos.line(
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(max.x, max.y, min.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, max.y, min.z),
            Vec3::new(max.x, max.y, max.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, max.y, max.z),
            Vec3::new(min.x, max.y, max.z),
            color,
        );
        gizmos.line(
            Vec3::new(min.x, max.y, max.z),
            Vec3::new(min.x, max.y, min.z),
            color,
        );
        // Vertical edges
        gizmos.line(
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(min.x, max.y, min.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(max.x, max.y, min.z),
            color,
        );
        gizmos.line(
            Vec3::new(max.x, min.y, max.z),
            Vec3::new(max.x, max.y, max.z),
            color,
        );
        gizmos.line(
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(min.x, max.y, max.z),
            color,
        );
    }
}
