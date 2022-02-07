mod report;

use crate::{Physics, PhysicsConfig, DebugMode, bounds::{aabb::Aabb, debug::{DebugBounds, DebugBoundsMesh}}};
use bevy::{prelude::*, ecs::schedule::ShouldRun};
use bevy_polyline::*;
pub use report::*;

pub struct PhysicsDebugPlugin;
impl Plugin for PhysicsDebugPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_plugin(PolylinePlugin)
            //.init_resource::<PhysicsDebugRender>()

            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .label(Physics::PostUpdate)
                    .with_system(report_system),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .label(Physics::PostUpdate)
                    .with_run_criteria(run_debug_bound)
                    .with_system(setup_debug_system)
                )
                .add_system_set_to_stage(
                    CoreStage::PreUpdate,
                    SystemSet::new()
                        .label(Physics::PostUpdate)
                        .with_run_criteria(run_debug_off)
                        .with_system(remove_debug_system)
                    );
    }
}

fn run_debug_bound(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.debug_mode == DebugMode::Bounds {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn run_debug_off(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.debug_mode == DebugMode::Off {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

// #[derive(Inspectable)]
// pub struct PhysicsDebugRender {
//     #[inspectable(ignore)]
//     pub material: Handle<PolylineMaterial>,
// }

// impl FromWorld for PhysicsDebugRender {
//     fn from_world(world: &mut World) -> Self {
//         let mut materials = world
//             .get_resource_mut::<Assets<PolylineMaterial>>()
//             .unwrap();

//         Self {
//             material: materials.add(PolylineMaterial {
//                 width: 3.0,
//                 color: Color::RED,
//                 perspective: false,
//             }),
//         }
//     }
// }

pub fn setup_debug_system(
    mut commands: Commands,
    query: Query<Entity, (With<Aabb>, Without<DebugBounds>)>,
) {
    for e in query.iter() {
        commands.entity(e).insert(DebugBounds);
    }
}

pub fn remove_debug_system(
    mut commands: Commands,
    query: Query<Entity, (With<Aabb>, With<DebugBounds>)>,
    query1: Query<Entity,  With<DebugBoundsMesh>>,
) {
    for e in query.iter() {
        commands.entity(e).remove::<DebugBounds>();

    }
    for e in query1.iter() {
        commands.entity(e).despawn();
    }
}
