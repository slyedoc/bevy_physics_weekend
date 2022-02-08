use bevy::prelude::*;
use bevy_physics_weekend::Physics;

pub struct ResetPlugin;

pub struct ResetEvent;

#[derive(Component)]
pub struct Reset;

impl Plugin for ResetPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ResetEvent>()
            .add_startup_system(setup)
            .add_system_to_stage(
                CoreStage::PostUpdate,
                reset_level.before(Physics::PreUpdate),
            );
    }
}

fn setup(mut ev_reset: EventWriter<ResetEvent>) {
    // Fire at startup
    ev_reset.send(ResetEvent);
    info!("Press `R` to reset level");
}

fn reset_level(
    input: Res<Input<KeyCode>>,
    q: Query<Entity, With<Reset>>,
    mut commands: Commands,
    mut ev_reset: EventWriter<ResetEvent>,
) {
    if input.just_pressed(KeyCode::R) {
        for e in q.iter() {
            commands.entity(e).despawn_recursive();
        }
        ev_reset.send(ResetEvent);
    }
}
