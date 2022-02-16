use bevy::prelude::*;
use bevy_inspector_egui::{WorldInspectorPlugin, WorldInspectorParams};
use bevy_physics_weekend::{PhysicsConfig, StepOnceEvent};
pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldInspectorParams {
            enabled: false,
            ..Default::default()
        })
        .add_plugin(WorldInspectorPlugin::default())
        .add_system(toggle_editor)
        .add_system_to_stage(
            CoreStage::PreUpdate,
            toggle_physics)
        .add_startup_system(setup);

        // TODO: Remove, only added so rapier tests dont error
        app.init_resource::<PhysicsConfig>();
    }
}

fn setup() {
    info!("Press F12 - toggle World Inspector");
    info!("Press Space - toggle Physics");
    info!("Press 1 - step ahead one frame Physics");
}

fn toggle_physics(
    input: Res<Input<KeyCode>>,
    mut config: ResMut<PhysicsConfig>,
    mut step_ev : EventWriter<StepOnceEvent>
) {
    if input.just_pressed(KeyCode::Space) {
        config.enabled = !config.enabled;
    }

    if !config.enabled && input.just_pressed(KeyCode::Key1) {
        info!("send");
        step_ev.send(StepOnceEvent);
    }
}

fn toggle_editor(input: Res<Input<KeyCode>>, mut params: ResMut<WorldInspectorParams>) {
    if input.just_pressed(KeyCode::F12) {
        params.enabled = !params.enabled;
    }
}

