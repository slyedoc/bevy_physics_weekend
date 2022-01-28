use bevy::prelude::*;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use bevy_physics_weekend::primitives::{ColliderBox, ColliderSphere, Body};
use bevy_rapier3d::{physics::{ColliderBundle, RigidBodyBundle, ColliderPositionSync}, prelude::{ColliderShape, RigidBodyVelocity}};

use super::Reset;

pub struct StackConfigPlugin;

impl Plugin for StackConfigPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(InspectorPlugin::<StackConfig>::new())
            .add_system(input_system);
    }
}


#[derive(Inspectable)]
pub struct StackConfig {
    pub count: usize,
    pub base_size: usize,
    pub grid_offset: f32,
    pub start_height: f32,

    pub shape: Shape,

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
            count: 5000,
            base_size: 10,
            grid_offset: 3.0,
            ball_material: materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("checker_red.png")),
                ..Default::default()
            }),
            start_height: 5.0,
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

impl StackConfig {
    #[allow(dead_code)]
    pub fn spawn(&self, commands: &mut Commands, meshes: &mut ResMut<Assets<Mesh>>, engine: Engine) {
        let mesh = match self.shape {
            Shape::Sphere => meshes.add(Mesh::from(shape::UVSphere {
                radius: 1.0,
                sectors: 16,
                stacks: 16,
            })),
            Shape::Box => meshes.add(Mesh::from(shape::Box::new(
                1.0, 1.0, 1.0,
            ))),
        };

        let b2 = self.base_size * self.base_size;
        let mut pos = Vec3::new(0.0, self.start_height, 0.0);
        for i in 0..self.count {
            if i % self.base_size == 0 {
                pos.x = 0.0;
                pos.z += self.grid_offset;
            } else {
                pos.x += self.grid_offset;
            }
            if i % b2 == 0 {
                pos.x = 0.0;
                pos.z = 0.0;
                pos.y += self.grid_offset;
            }

            let item =commands
                .spawn_bundle(PbrBundle {
                    transform: Transform::from_translation(pos),
                    mesh: mesh.clone(),
                    material: self.ball_material.clone(),
                    ..Default::default()
                })
                .insert(Name::new("Stack Item"))
                .insert(Reset)
                .id();

            match engine {
                Engine::Crate => {
                    commands.entity(item).insert(Body {
                        inv_mass: 1.0,
                        elasticity: 0.9,
                        friction: 0.9,
                        ..Default::default()
                    });

                    match self.shape {
                        Shape::Sphere => {
                            commands.entity(item).insert(ColliderSphere::new(1.0));
                        }
                        Shape::Box => {
                            commands.entity(item).insert(ColliderBox::new_xyz(1.0, 1.0, 1.0));
                        }
                    }
                },
                Engine::Rapier => {

                    commands.entity(item).insert_bundle(RigidBodyBundle {
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
                        shape: match self.shape {
                            Shape::Sphere  => ColliderShape::ball(1.0).into(),
                            Shape::Box => {
                                ColliderShape::cuboid(0.5, 0.5, 0.5).into()
                            }
                        },
                        ..ColliderBundle::default()
                    })
                    .insert(ColliderPositionSync::Discrete);
                },
            }
        }
    }
}



#[allow(dead_code)]
pub fn input_system(input: Res<Input<KeyCode>>, mut config: ResMut<StackConfig>) {
    if input.just_pressed(KeyCode::Equals) {
        config.count += 1000;
    }

    if input.just_pressed(KeyCode::Minus) {
        config.count = (config.count - 1000).clamp(0, 50000);
    }
}
