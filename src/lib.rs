#![allow(dead_code)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
#![feature(vec_retain_mut)]
#![feature(div_duration)]

pub mod colliders;
pub mod constraints;
pub mod debug;
pub mod intersect;
pub mod bounds;
mod math;
mod phase;
pub mod primitives;

use bounds::*;
use colliders::{Collider, ColliderBox, ColliderSphere};
use primitives::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::Inspectable;
use phase::*;

#[derive(Inspectable, PartialEq, Eq)]
pub enum Mode {
    Static,
    Dynamic,
}

#[derive(Inspectable, PartialEq, Eq)]
pub enum DebugMode {
    Off,
    Bounds,
}
#[derive(Component, Inspectable)]
pub struct PhysicsConfig {
    pub enabled: bool,
    pub mode: Mode,
    #[inspectable(min = -10.0, max = 10.0)]
    pub time_dilation: f32,
    pub gravity: Vec3,
    pub constrain_max_iter: usize,
    pub debug_mode: DebugMode,
}

#[derive(Default)]
pub struct PhysicsTime {
    time: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gravity: Vec3::new(0.0, -9.8, 0.0),
            constrain_max_iter: 5,
            time_dilation: 1.0,
            mode: Mode::Static,
            debug_mode: DebugMode::Bounds,
        }
    }
}

#[derive(Clone, Hash, Debug, PartialEq, Eq, SystemLabel)]
pub enum Physics {
    PreUpdate,
    Update,
    PostUpdate,
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PreUpdate {
    First,
    Second,
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum Update {
    Dynamics,
    Broadphase,
    Narrowphase,
    Manifold,
    ConstraintsPreSolve,
    ConstraintsSolve,
    ResolveContact,
    Transform,
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(BoundingVolumePlugin::<aabb::Aabb>::default())
            .init_resource::<PhysicsConfig>()
            .init_resource::<PhysicsTime>()
            .add_event::<Contact>()
            .add_event::<BroadContact>()
            .add_event::<ManifoldContactEvent>()
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .label(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_system(update_time_system)
                    .with_system(added_collider::<ColliderSphere>.label(PreUpdate::First))
                    .with_system(added_collider::<ColliderBox>.label(PreUpdate::First))
                    // .with_system(
                    //     calucate_aabb::<ColliderSphere>
                    //         .label(PreUpdate::Second)
                    //         .after(PreUpdate::First),
                    // )
                    // .with_system(
                    //     calucate_aabb::<ColliderBox>
                    //         .label(PreUpdate::Second)
                    //         .after(PreUpdate::First),
                    // ),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .label(Physics::Update)
                    .after(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_system(dynamics::dynamics_gravity_system.label(Update::Dynamics))
                    .with_system(
                        broadphase::broadphase_system
                            .label(Update::Broadphase)
                            .after(Update::Dynamics),
                    ),
            )
            // Static Path
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .label(Physics::Update)
                    .after(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_run_criteria(run_static)
                    .with_system(
                        narrowphase::narrowphase_system_static
                            .label(Update::Narrowphase)
                            .after(Update::Broadphase),
                    )
                    .with_system(
                        resolve_contact::resolve_contact_system_static
                            .label(Update::ResolveContact)
                            .after(Update::Narrowphase),
                    )
                    .with_system(
                        transform::update_local_tranform
                            .label(Update::Transform)
                            .after(Update::ResolveContact),
                    ),
            );
        // // Dynamic Path
        // .add_system_set_to_stage(
        //     CoreStage::PreUpdate,
        //     SystemSet::new()
        //         .label(Physics::Update)
        //         .after(Physics::PreUpdate)
        //         .with_run_criteria(run_physics)
        //         .with_run_criteria(run_dynamic)
        //         .with_system(
        //             narrowphase::narrowphase_system_dynamic
        //                 .label(Update::Narrowphase)
        //                 .after(Update::Broadphase),
        //         )
        //         .with_system(
        //             resolve_contact::resolve_contact_system_dynamic
        //                 .label(Update::ResolveContact)
        //                 .after(Update::Narrowphase),
        //         ),
        // );
        // .with_system(
        //     constraints::constraint_penetration::pre_solve_system
        //         .label(Update::ConstraintsPreSolve)
        //         .after(Update::ResolveContact),
        // )
        // .with_system(
        //     constraints::constraint_penetration::solve_system
        //         .label(Update::ConstraintsPreSolve)
        //         .after(Update::ResolveContact),
        // ),
        // .add_system_set_to_stage(
        //     CoreStage::PreUpdate,
        //     SystemSet::new()
        //         .label(Physics::PostUpdate)
        //         .with_run_criteria(run_physics)
        //         .with_system(manifold_remove_expired_system),
        // );
    }
}

pub fn added_collider<T: Component + Collider>(
    mut commands: Commands,
    query: Query<(Entity, &T), Added<T>>,
) {
    for (e, collider) in query.iter() {
        commands
            .entity(e)
            .insert(collider.shape_type())
            .insert(Bounded::<aabb::Aabb>::default());
    }
}

// pub fn calucate_aabb<T: Component + Collider>(
//     mut query: Query<(&GlobalTransform, &Body, &T, &mut Aabb)>,
//     pt: Res<PhysicsTime>,
//     config: Res<PhysicsConfig>,
// ) {
//     #[allow(unused)]
//     for (trans, body, collider, mut aabb) in query.iter_mut() {
//         // TODO: remove this copy and use
//         let mut new_aabb = collider.aabb(trans);

//         // expand the bounds by the linear velocity
//         if config.mode == Mode::Dynamic {
//             new_aabb.expand_by_point(new_aabb.mins + body.linear_velocity * pt.time);
//             new_aabb.expand_by_point(new_aabb.maxs + body.linear_velocity * pt.time);
//         }

//         const BOUNDS_EPS: f32 = 0.01;
//         new_aabb.expand_by_point(new_aabb.mins - Vec3::splat(BOUNDS_EPS));
//         new_aabb.expand_by_point(new_aabb.maxs + Vec3::splat(BOUNDS_EPS));

//         aabb.mins = new_aabb.mins;
//         aabb.maxs = new_aabb.maxs;
//     }
// }

fn run_physics(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.time_dilation != 0.0 && config.enabled {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn run_static(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.mode == Mode::Static {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn run_dynamic(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.mode == Mode::Dynamic {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn update_time_system(time: Res<Time>, config: Res<PhysicsConfig>, mut pt: ResMut<PhysicsTime>) {
    // NOTE: I am avoiding using fixed time, thats because
    // we want to develop the hot path of the physics system
    pt.time = time.delta_seconds() * config.time_dilation;
}
