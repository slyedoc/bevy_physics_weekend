mod helper;
use bevy::prelude::*;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use bevy_physics_weekend::{
    colliders::{ColliderBox, ColliderSphere},
    debug::PhysicsDebugPlugin,
    primitives::*,
    PhysicsPlugin,
};

use bevy_rapier3d::{
    physics::{ColliderBundle, ColliderPositionSync, NoUserData, RigidBodyBundle},
    prelude::{ColliderShape, RapierPhysicsPlugin, RigidBodyVelocity},
};

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum AppState {
    Setup,
    Play,
    Reset,
}

#[derive(Component)]
pub struct Reset;

// This is setup to use differet physics systems on reset
fn main() {
    App::new()
        .add_state(AppState::Setup)
        .insert_resource(WindowDescriptor {
            title: "Physics Ball Stack".to_string(),
            vsync: false, // just for testing
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(helper::HelperPlugin)
        // our plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(PhysicsDebugPlugin)
        // Rapier
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        //.add_plugin(RapierRenderPlugin)
        // Custom helpers
        .add_plugin(InspectorPlugin::<StackConfig>::new())
        .add_system_set(SystemSet::on_enter(AppState::Setup).with_system(enter_setup_system))
        .add_system_set(SystemSet::on_enter(AppState::Play).with_system(enter_play_system))
        .add_system_set(SystemSet::on_update(AppState::Play).with_system(hotkey_system))
        .add_system_set(SystemSet::on_enter(AppState::Reset).with_system(enter_reset_system))
        .run();
}

fn enter_play_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<StackConfig>,
) {
    info!("Reset");

    // Create the ground
    match config.engine {
        Engine::Crate => {
            let ground_radius = 1000.0;
            commands
                .spawn_bundle(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::UVSphere {
                        radius: ground_radius,
                        sectors: 60,
                        stacks: 60,
                    })),
                    transform: Transform::from_xyz(0.0, -ground_radius - 1.0, 0.0),
                    material: materials.add(StandardMaterial {
                        base_color: Color::GREEN,
                        ..Default::default()
                    }),
                    ..Default::default()
                })
                .insert(Body {
                    inv_mass: 0.0,
                    friction: 0.2,
                    elasticity: 1.0,
                    ..Default::default()
                })
                .insert(ColliderSphere::new(ground_radius))
                .insert(Reset)
                .insert(Name::new("Ground"));
        }
        Engine::Rapier => {
            let ground_size = 200.0;
            let ground_height = 10.0;
            let ground_pos = Vec3::new(0.0, -ground_height, 0.0);
            commands
                .spawn()
                .insert_bundle(PbrBundle {
                    transform: Transform::from_translation(ground_pos),
                    mesh: meshes.add(Mesh::from(shape::Box {
                        min_x: -ground_size,
                        max_x: ground_size,
                        min_y: -ground_height,
                        max_y: ground_height,
                        min_z: -ground_size,
                        max_z: ground_size,
                    })),
                    material: materials.add(StandardMaterial {
                        base_color: Color::GREEN,
                        ..Default::default()
                    }),
                    ..Default::default()
                })
                .insert_bundle(ColliderBundle {
                    shape: ColliderShape::cuboid(ground_size, ground_height, ground_size).into(),
                    position: ground_pos.into(),
                    ..ColliderBundle::default()
                })
                //.insert(ColliderDebugRender::default())
                .insert(ColliderPositionSync::Discrete)
                .insert(Reset)
                .insert(Name::new("Ground"));
        }
    }

    let mesh = match config.shape {
        Shape::Sphere => meshes.add(Mesh::from(shape::UVSphere {
            radius: 0.5,
            sectors: 16,
            stacks: 16,
        })),
        Shape::Box => meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0))),
    };

    let b2 = config.base_size * config.base_size;
    let mut pos = Vec3::new(0.0, config.start_height, 0.0);
    for i in 0..config.count {
        if i % config.base_size == 0 {
            pos.x = 0.0;
            pos.z += config.grid_offset;
        } else {
            pos.x += config.grid_offset;
        }
        if i % b2 == 0 {
            pos.x = 0.0;
            pos.z = 0.0;
            pos.y += config.grid_offset;
        }

        let item = commands
            .spawn_bundle(PbrBundle {
                transform: Transform::from_translation(pos),
                mesh: mesh.clone(),
                material: config.ball_material.clone(),
                ..Default::default()
            })
            .insert(Name::new("Stack Item"))
            .insert(Reset)
            .id();

        match config.engine {
            Engine::Crate => {
                commands.entity(item).insert(Body {
                    inv_mass: 1.0,
                    elasticity: 1.0,
                    friction: 0.9,
                    ..Default::default()
                });
                match config.shape {
                    Shape::Sphere => {
                        commands.entity(item).insert(ColliderSphere::new(0.5));
                    }
                    Shape::Box => {
                        commands
                            .entity(item)
                            .insert(ColliderBox::new_xyz(1.0, 1.0, 1.0));
                    }
                }
            }
            Engine::Rapier => {
                commands
                    .entity(item)
                    .insert_bundle(RigidBodyBundle {
                        position: pos.into(),
                        // TODO: Remove this, a small velocity so its not a stable stack
                        velocity: RigidBodyVelocity {
                            linvel: Vec3::new(0.1, 0.0, 0.0).into(),
                            angvel: Vec3::new(0.1, 0.0, 0.0).into(),
                        }
                        .into(),
                        ..Default::default()
                    })
                    .insert_bundle(ColliderBundle {
                        shape: match config.shape {
                            Shape::Sphere => ColliderShape::ball(0.5).into(),
                            Shape::Box => ColliderShape::cuboid(0.5, 0.5, 0.5).into(),
                        },
                        ..ColliderBundle::default()
                    })
                    .insert(ColliderPositionSync::Discrete);
            }
        }
    }
}

fn enter_reset_system(
    mut app_state: ResMut<State<AppState>>,

    q: Query<Entity, With<Reset>>,
    mut commands: Commands,
) {
    for e in q.iter() {
        commands.entity(e).despawn_recursive();
    }
    app_state.set(AppState::Play).unwrap();
}

/// set up a simple 3D scene
fn enter_setup_system(mut commands: Commands, mut app_state: ResMut<State<AppState>>) {
    // camera
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(60.0, 60.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        })
        .insert(helper::CameraController::default())
        .insert(Name::new("Camera"));

    // light
    helper::spawn_light(&mut commands);

    app_state.set(AppState::Play).unwrap();
}

#[derive(Inspectable)]
pub struct StackConfig {
    pub count: usize,
    pub base_size: usize,
    #[inspectable(min = 1.0, max = 10.0)]
    pub grid_offset: f32,
    pub start_height: f32,

    pub shape: Shape,
    pub engine: Engine,

    #[inspectable(ignore)]
    pub ball_material: Handle<StandardMaterial>,
}

impl FromWorld for StackConfig {
    fn from_world(world: &mut World) -> Self {
        let world = world.cell();
        let mut materials = world
            .get_resource_mut::<Assets<StandardMaterial>>()
            .unwrap();

        let asset_server = world.get_resource_mut::<AssetServer>().unwrap();

        Self {
            shape: Shape::Sphere,
            count: 2000,
            base_size: 10,
            grid_offset: 1.0,
            ball_material: materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("checker_red.png")),
                ..Default::default()
            }),
            start_height: 5.0,
            engine: Engine::Crate,
        }
    }
}

#[derive(Inspectable)]
pub enum Engine {
    Crate,
    Rapier,
}

#[derive(Inspectable)]
pub enum Shape {
    Sphere,
    Box,
}

pub fn hotkey_system(
    mut input: ResMut<Input<KeyCode>>,
    mut config: ResMut<StackConfig>,
    mut app_state: ResMut<State<AppState>>,
) {
    if input.just_pressed(KeyCode::Equals) {
        config.count += 1000;
    }

    if input.just_pressed(KeyCode::Minus) {
        config.count = (config.count - 1000).clamp(0, 50000);
    }

    if input.just_pressed(KeyCode::R) {
        app_state.set(AppState::Reset).unwrap();
        input.reset(KeyCode::R);
    }
}
