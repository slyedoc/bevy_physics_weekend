pub mod camera_controller;
pub mod editor;
pub mod fps;
pub mod reset;
pub mod stack_config;

use bevy::{prelude::*, app::AppExit};
use bevy_inspector_egui::InspectorPlugin;

use bevy_physics_weekend::debug_render::PhysicsReport;
pub use camera_controller::*;
pub use editor::*;
pub use fps::*;
pub use reset::*;
pub use stack_config::*;

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(EditorPlugin)
        .add_plugin(CameraControllerPlugin)
        .add_plugin(ResetPlugin)
        .add_plugin(FPSPlugin)
        .add_plugin(InspectorPlugin::<PhysicsReport>::new());

        #[cfg(feature = "timeout")]
        app.add_system(timeout_system);
    }
}


pub fn timeout_system(time: Res<Time>, mut quit: EventWriter<AppExit>) {
    if time.time_since_startup().as_secs_f32() > 5.0 {
        quit.send(AppExit);
    }
}


pub fn spawn_light(commands: &mut Commands) {
    const HALF_SIZE: f32 = 50.0;
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

