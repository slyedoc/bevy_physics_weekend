pub mod camera_controller;
pub mod editor;
pub mod fps;
pub mod reset;
pub mod ball_stack;

use bevy::{prelude::*, app::PluginGroupBuilder};
use bevy_inspector_egui::InspectorPlugin;
use bevy_physics_weekend::prelude::PhysicsReport;
pub use camera_controller::*;
pub use editor::*;
pub use fps::*;
pub use reset::*;
pub use ball_stack::*;

pub struct HelperPlugins;

impl PluginGroup for HelperPlugins {
    fn build(&mut self, group: &mut PluginGroupBuilder) {
        group.add(EditorPlugin);
        group.add(CameraControllerPlugin);
        group.add(ResetPlugin);
        group.add(FPSPlugin);
        group.add(InspectorPlugin::<PhysicsReport>::new());
    }
}


pub fn spawn_light(commands: &mut Commands) {
    const HALF_SIZE: f32 = 100.0;
    commands
        .spawn_bundle(DirectionalLightBundle {
            directional_light: DirectionalLight {
                illuminance: 10000.0,
                // Configure the projection to better fit the scene
                shadow_projection: OrthographicProjection {
                    left: -HALF_SIZE,
                    right: HALF_SIZE,
                    bottom: -HALF_SIZE,
                    top: HALF_SIZE,
                    near: -10.0 * HALF_SIZE,
                    far: 100.0 * HALF_SIZE,
                    ..Default::default()
                },
                shadows_enabled: true,
                ..Default::default()
            },
            transform: Transform {
                translation: Vec3::new(10.0, 2.0, 10.0),
                rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_4),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("Light"));
}

