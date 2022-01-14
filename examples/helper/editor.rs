use bevy::prelude::*;
use bevy_inspector_egui::{WorldInspectorPlugin, WorldInspectorParams};
pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldInspectorParams {
            enabled: false,
            ..Default::default()
        })
        .add_plugin(WorldInspectorPlugin::default())
        .add_system(toggle_editor)
        .add_startup_system(setup);
    }
}

fn setup() {
    info!("Press F12 to toggle World Inspector");
}

fn toggle_editor(input: Res<Input<KeyCode>>, mut params: ResMut<WorldInspectorParams>) {
    if input.just_pressed(KeyCode::F12) {
        params.enabled = !params.enabled;
    }
}

