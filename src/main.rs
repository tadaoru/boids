use bevy::{math::Vec3, prelude::*};
use bevy_egui::{egui, EguiContext, EguiPlugin, EguiSettings};
use rand::Rng;
use std::{f32::consts::PI, fmt};

const THR: usize = 16;
const BOIDS_COUNT: usize = 1024;
const BOIDS_RADIUS: f32 = 0.01;

fn restrict_vector_length(vector: Vec3, min: f32, max: f32) -> Vec3 {
    let len = vector.length();
    if len < min {
        vector * min / len
    } else if len > max {
        vector * max / len
    } else {
        vector
    }
}

fn partial_max(a: f32, b: f32, c: f32) -> f32 {
    if a > b {
        if a > c {
            a
        } else {
            c
        }
    } else if b > c {
        b
    } else {
        c
    }
}

struct UIState {
    selected_property_idx: i32,
}

impl UIState {
    const DEFAULT_PROPERTY_IDX: usize = 0;
    const PROPERTY_STR: [&str; 6] = ["Git", "Book 1", "Book 2", "Book 3", "Book 4", "Book 5"];
}

impl fmt::Display for UIState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            Self::PROPERTY_STR[self.selected_property_idx as usize]
        )
    }
}

impl Default for UIState {
    fn default() -> Self {
        Self {
            selected_property_idx: Self::DEFAULT_PROPERTY_IDX as i32,
        }
    }
}

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Boids".to_string(),
            width: 1200.,
            height: 800.,
            ..default()
        })
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(BoidProperty::default())
        .insert_resource(UIState::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(EguiPlugin)
        .add_startup_system(setup_scene)
        .add_startup_system(generate_boids)
        .add_system(update_ui_scale_factor)
        .add_system(treat_ui)
        .add_system(interact_boids)
        .add_system(affect_from_env)
        .add_system(integrate)
        .run();
}

#[derive(Clone, Copy, Debug)]
struct BoidPropertyUI {
    coh_force: f32,
    sep_force: f32,
    ali_force: f32,
    coh_dist: f32,
    sep_dist: f32,
    ali_dist: f32,
    coh_angle: f32,
    sep_angle: f32,
    ali_angle: f32,
    min_velocity: f32,
    max_velocity: f32,
}

impl BoidPropertyUI {
    const PREDEFINED: [Self; 6] = [
        Self {
            coh_force: 0.008,
            sep_force: 0.5,
            ali_force: 0.05,
            coh_dist: 0.2,
            sep_dist: 0.04,
            ali_dist: 0.3,
            coh_angle: 2.0,
            sep_angle: 2.0,
            ali_angle: 2.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
        Self {
            coh_force: 0.008,
            sep_force: 0.4,
            ali_force: 0.06,
            coh_dist: 0.5,
            sep_dist: 0.05,
            ali_dist: 0.1,
            coh_angle: 2.0,
            sep_angle: 2.0,
            ali_angle: 3.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
        Self {
            coh_force: 0.008,
            sep_force: 0.4,
            ali_force: 0.06,
            coh_dist: 0.5,
            sep_dist: 0.05,
            ali_dist: 0.1,
            coh_angle: 2.0,
            sep_angle: 2.0,
            ali_angle: 3.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
        Self {
            coh_force: 0.2,
            sep_force: 0.1,
            ali_force: 0.03,
            coh_dist: 0.5,
            sep_dist: 0.08,
            ali_dist: 0.1,
            coh_angle: 2.0,
            sep_angle: 2.0,
            ali_angle: 2.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
        Self {
            coh_force: 0.005,
            sep_force: 0.5,
            ali_force: 0.01,
            coh_dist: 0.8,
            sep_dist: 0.03,
            ali_dist: 0.5,
            coh_angle: 2.0,
            sep_angle: 2.0,
            ali_angle: 2.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
        Self {
            coh_force: 0.008,
            sep_force: 0.5,
            ali_force: 0.05,
            coh_dist: 0.2,
            sep_dist: 0.04,
            ali_dist: 0.3,
            coh_angle: 1.0,
            sep_angle: 2.0,
            ali_angle: 2.0,
            min_velocity: 0.005,
            max_velocity: 0.03,
        },
    ];
}

#[derive(Clone, Copy, Debug)]
struct BoidProperty {
    pub coh_force: f32,
    pub sep_force: f32,
    pub ali_force: f32,
    // (distance threshould) ^ 2
    pub coh_dist_sq: f32,
    pub sep_dist_sq: f32,
    pub ali_dist_sq: f32,
    // angle threshould
    pub coh_angle_cos: f32,
    pub sep_angle_cos: f32,
    pub ali_angle_cos: f32,
    // velocity range
    pub min_velocity: f32,
    pub max_velocity: f32,
    // boundary
    pub boundary_dist_sq: f32,
    pub boundary_force: f32,
}

impl Default for BoidProperty {
    fn default() -> Self {
        Self::from_ui(&BoidPropertyUI::PREDEFINED[UIState::DEFAULT_PROPERTY_IDX])
    }
}

impl BoidProperty {
    fn from_ui(ui: &BoidPropertyUI) -> Self {
        Self {
            coh_force: ui.coh_force,
            sep_force: ui.sep_force,
            ali_force: ui.ali_force,
            coh_dist_sq: ui.coh_dist * ui.coh_dist,
            sep_dist_sq: ui.sep_dist * ui.sep_dist,
            ali_dist_sq: ui.ali_dist * ui.ali_dist,
            coh_angle_cos: (PI / ui.coh_angle).cos(),
            sep_angle_cos: (PI / ui.sep_angle).cos(),
            ali_angle_cos: (PI / ui.ali_angle).cos(),
            min_velocity: ui.min_velocity,
            max_velocity: ui.max_velocity,
            boundary_dist_sq: 1.0,
            boundary_force: 0.001,
        }
    }
}

#[derive(Component)]
struct Position(Vec3);

#[derive(Component)]
struct Velocity(Vec3);

#[derive(Component)]
struct CorrFactor {
    coh: (usize, Vec3),
    sep: (usize, Vec3),
    ali: (usize, Vec3),
}

impl CorrFactor {
    fn zero() -> Self {
        Self {
            coh: (0, Vec3::ZERO),
            sep: (0, Vec3::ZERO),
            ali: (0, Vec3::ZERO),
        }
    }
}

#[derive(Component)]
struct EnvForce {
    boundary: Vec3,
}

impl EnvForce {
    fn zero() -> Self {
        Self {
            boundary: Vec3::ZERO,
        }
    }
}

#[derive(Bundle)]
pub struct Boid {
    #[bundle]
    pbr: PbrBundle,
    position: Position,
    velocity: Velocity,
    corr_factor: CorrFactor,
    env_force: EnvForce,
}

fn generate_boids(
    mut commands: Commands,
    prop: Res<BoidProperty>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut rng = rand::thread_rng();
    for _ in 0..BOIDS_COUNT {
        let position = Position(Vec3::new(
            rng.gen_range(-1.0..=1.0),
            rng.gen_range(-1.0..=1.0),
            rng.gen_range(-1.0..=1.0),
        ));
        let velocity = Velocity(restrict_vector_length(
            Vec3::new(
                rng.gen_range(-1.0..=1.0),
                rng.gen_range(-1.0..=1.0),
                rng.gen_range(-1.0..=1.0),
            ),
            prop.min_velocity,
            prop.max_velocity,
        ));
        let corr_factor = CorrFactor::zero();
        let env_force = EnvForce::zero();
        let pbr = PbrBundle {
            transform: Transform::from_translation(position.0),
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: BOIDS_RADIUS,
                ..default()
            })),
            material: materials.add(StandardMaterial {
                base_color: Color::SILVER,
                ..default()
            }),
            ..default()
        };

        commands.spawn_bundle(Boid {
            pbr,
            position,
            velocity,
            corr_factor,
            env_force,
        });
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            color: Color::WHITE,
            intensity: 5000.0,
            range: 20.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(1.0, 10.0, 1.0),
        ..default()
    });

    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(shape::Plane { size: 6.0 }.into()),
        material: materials.add(Color::DARK_GRAY.into()),
        transform: Transform::from_xyz(0.0, -3.0, 0.0),
        ..default()
    });

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(4.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),

        ..default()
    });
}

fn interact_boids(
    prop: Res<BoidProperty>,
    mut query: Query<(&Position, &Velocity, &mut CorrFactor)>,
) {
    let mut iter = query.iter_combinations_mut();
    while let Some(
        [(Position(p1), Velocity(v1), mut corrf1), (Position(p2), Velocity(v2), mut corrf2)],
    ) = iter.fetch_next()
    {
        let delta1 = *p2 - *p1;
        let dist_sq = delta1.length_squared();
        if dist_sq >= partial_max(prop.coh_dist_sq, prop.sep_dist_sq, prop.ali_dist_sq) {
            continue;
        }
        let delta2 = -delta1;

        let angle1_cos = v1.dot(delta1) / (v1.length() * delta1.length());
        let angle2_cos = v2.dot(delta2) / (v2.length() * delta2.length());

        if dist_sq < prop.coh_dist_sq {
            if angle1_cos > prop.coh_angle_cos {
                corrf1.coh.0 += 1;
                corrf1.coh.1 += delta1;
            }
            if angle2_cos > prop.coh_angle_cos {
                corrf2.coh.0 += 1;
                corrf2.coh.1 += delta2;
            }
        }
        if dist_sq < prop.sep_dist_sq {
            if angle1_cos > prop.sep_angle_cos {
                corrf1.coh.0 += 1;
                corrf1.coh.1 += -delta1;
            }
            if angle2_cos > prop.sep_angle_cos {
                corrf2.coh.0 += 1;
                corrf2.coh.1 += -delta2;
            }
        }
        if dist_sq < prop.ali_dist_sq {
            if angle1_cos > prop.ali_angle_cos {
                corrf1.coh.0 += 1;
                corrf1.coh.1 += *v2 - *v1;
            }
            if angle2_cos > prop.ali_angle_cos {
                corrf2.coh.0 += 1;
                corrf2.coh.1 += *v1 - *v2
            }
        }
    }
}

fn affect_from_env(prop: Res<BoidProperty>, mut query: Query<(&Position, &mut EnvForce)>) {
    query.par_for_each_mut(THR, |(Position(pos), mut envf)| {
        let len_sq = pos.length_squared();
        envf.boundary = if len_sq > prop.boundary_dist_sq {
            *pos * (prop.boundary_force * (1.0 - len_sq.sqrt()) / len_sq.sqrt())
        } else {
            Vec3::ZERO
        }
    });
}

fn integrate(
    prop: Res<BoidProperty>,
    mut query: Query<(
        &mut Position,
        &mut Velocity,
        &mut CorrFactor,
        &mut EnvForce,
        &mut Transform,
    )>,
) {
    query.par_for_each_mut(THR, |(mut pos, mut vel, mut corrf, mut envf, mut trans)| {
        let mut force_sum = envf.boundary;
        if corrf.coh.0 > 0 {
            force_sum += corrf.coh.1 / corrf.coh.0 as f32 * prop.coh_force;
        }
        if corrf.sep.0 > 0 {
            force_sum += corrf.sep.1 * prop.sep_force;
        }
        if corrf.ali.0 > 0 {
            force_sum += corrf.ali.1 / corrf.ali.0 as f32 * prop.ali_force;
        }
        vel.0 = restrict_vector_length(vel.0 + force_sum, prop.min_velocity, prop.max_velocity);
        pos.0 += vel.0;

        trans.translation = pos.0;

        *corrf = CorrFactor::zero();
        *envf = EnvForce::zero();
    });
}

fn treat_ui(
    mut egui_ctx: ResMut<EguiContext>,
    mut prop: ResMut<BoidProperty>,
    mut ui_state: ResMut<UIState>,
) {
    let bpui = &BoidPropertyUI::PREDEFINED[ui_state.selected_property_idx as usize];
    egui::Window::new("Boid property")
        .vscroll(true)
        .show(egui_ctx.ctx_mut(), |ui| {
            egui::ComboBox::from_label("Configuration")
                .selected_text(format!("{}", *ui_state))
                .show_ui(ui, |ui| {
                    for (i, s) in UIState::PROPERTY_STR.into_iter().enumerate() {
                        ui.selectable_value(&mut ui_state.selected_property_idx, i as i32, s);
                    }
                });

            ui.separator();
            ui.label("Forces");
            ui.label(format!(" cohesion: {}", bpui.coh_force));
            ui.label(format!(" separation: {}", bpui.sep_force));
            ui.label(format!(" alignment: {}", bpui.ali_force));
        });

    *prop = BoidProperty::from_ui(bpui);
}

fn update_ui_scale_factor(
    keyboard_input: Res<Input<KeyCode>>,
    mut toggle_scale_factor: Local<Option<bool>>,
    mut egui_settings: ResMut<EguiSettings>,
    windows: Res<Windows>,
) {
    if keyboard_input.just_pressed(KeyCode::Slash) || toggle_scale_factor.is_none() {
        *toggle_scale_factor = Some(!toggle_scale_factor.unwrap_or(true));

        if let Some(window) = windows.get_primary() {
            let scale_factor = if toggle_scale_factor.unwrap() {
                1.0
            } else {
                1.0 / window.scale_factor()
            };
            egui_settings.scale_factor = scale_factor;
        }
    }
}
