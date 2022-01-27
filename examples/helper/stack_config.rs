use bevy::prelude::*;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};

pub struct StackConfigPlugin;

impl Plugin for StackConfigPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(InspectorPlugin::<StackConfig>::new())
            .add_system(input_system);
    }
}

#[derive(Inspectable)]
pub enum StackItem {
    Sphere {
        radius: f32,
        sectors: usize,
        stacks: usize,
    },
    Box {
        size: Vec3,
    },
}

#[derive(Inspectable)]
pub struct StackConfig {
    pub count: usize,
    pub base_size: usize,
    pub grid_offset: f32,
    pub start_height: f32,

    pub stack_item: StackItem,

    #[inspectable(ignore)]
    pub ball_material: Handle<StandardMaterial>,
}

impl StackConfig {
    pub fn get_mesh(&self, meshes: &mut ResMut<Assets<Mesh>>) -> Handle<Mesh> {
        match self.stack_item {
            StackItem::Sphere {
                radius,
                sectors,
                stacks,
            } => meshes.add(Mesh::from(bevy::render::mesh::shape::UVSphere {
                radius,
                sectors,
                stacks,
            })),
            StackItem::Box { size } => meshes.add(Mesh::from(bevy::render::mesh::shape::Box::new(
                size.x, size.y, size.z,
            ))),
        }
    }
}
impl FromWorld for StackConfig {
    fn from_world(world: &mut World) -> Self {
        let world = world.cell();
        let mut materials = world
            .get_resource_mut::<Assets<StandardMaterial>>()
            .unwrap();

        let asset_server = world.get_resource_mut::<AssetServer>().unwrap();

        Self {
            count: 10000,
            base_size: 10,
            grid_offset: 3.0,

            stack_item: StackItem::Sphere {
                radius: 1.0,
                sectors: 8,
                stacks: 8,
            },

            ball_material: materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("checker_red.png")),
                ..Default::default()
            }),
            start_height: 5.0,
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
