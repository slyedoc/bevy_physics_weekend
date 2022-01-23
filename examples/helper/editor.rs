use bevy::prelude::*;
use bevy_inspector_egui::{WorldInspectorPlugin, WorldInspectorParams};
use bevy_physics_weekend::PhysicsConfig;
pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldInspectorParams {
            enabled: false,
            ..Default::default()
        })
        .add_plugin(WorldInspectorPlugin::default())
        .add_system(toggle_editor)
        .add_system(toggle_physics)
        .add_startup_system(setup);
    }
}

fn setup() {
    info!("Press F12 - toggle World Inspector");
    info!("Press Space - toggle Physics");
}

fn toggle_physics(
    input: Res<Input<KeyCode>>,
    mut config: ResMut<PhysicsConfig>,
) {
    if input.just_pressed(KeyCode::Space) {
        config.enabled = !config.enabled;
    }
}

fn toggle_editor(input: Res<Input<KeyCode>>, mut params: ResMut<WorldInspectorParams>) {
    if input.just_pressed(KeyCode::F12) {
        params.enabled = !params.enabled;
    }
}

