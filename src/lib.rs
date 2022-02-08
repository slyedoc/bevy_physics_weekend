#![allow(dead_code)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
#![feature(vec_retain_mut)]
#![feature(div_duration)]

pub mod bounds;
pub mod colliders;
pub mod constraints;
pub mod debug;
pub mod intersect;
mod math;
mod phase;
pub mod primitives;

use bounds::{*, aabb::Aabb};
use colliders::{Collider, ColliderBox, ColliderSphere};
use primitives::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*, transform::TransformSystem};
use bevy_inspector_egui::Inspectable;
use phase::*;

/// Continuous Collision etection
#[derive(Inspectable, PartialEq, Eq)]
pub enum CollisionDetection {
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
    pub collision_dection: CollisionDetection,
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
            collision_dection: CollisionDetection::Static,
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

/// The names of system labels for run order
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
        app
            .init_resource::<PhysicsConfig>()
            .init_resource::<PhysicsTime>()
            .add_event::<Contact>()
            .add_event::<BroadContact>()
            .add_event::<ManifoldContactEvent>()
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(Physics::PreUpdate)
                    .after(TransformSystem::TransformPropagate)
                    .with_run_criteria(run_physics)
                    .with_system(update_time_system)
                    .with_system(added_collider::<ColliderSphere>.label(PreUpdate::First))
                    .with_system(added_collider::<ColliderBox>.label(PreUpdate::First))
                    //.with_system(update_aabb.after(BoundingSystem::UpdateBounds)),
            )
            // TODO: right now this uses the mesh instead of the collider
            .add_plugin(BoundingVolumePlugin::<aabb::Aabb>::default())
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(Physics::Update)
                    .after(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_system(dynamics::dynamics_gravity_system.label(Update::Dynamics))
                    .with_system(
                        broad::broadphase_system
                            .label(Update::Broadphase)
                            .after(Update::Dynamics),
                    )
                    // Narrowphase Static and Dynamic collision detection would go here
                    // they part of diffferent set since they use different run_criteria
                    .with_system(
                        //resolve_contact::resolve_contact_system_ordered
                            resolve_contact::resolve_contact_system
                            .label(Update::ResolveContact)
                            .after(Update::Narrowphase),
                    )
                    .with_system(
                        constraints::constraint_penetration::pre_solve_system
                            .label(Update::ConstraintsPreSolve)
                            .after(Update::ResolveContact),
                    )
                    .with_system(
                        constraints::constraint_penetration::solve_system
                            .label(Update::ConstraintsSolve)
                            .after(Update::ConstraintsPreSolve),
                    )
                    .with_system(
                        transform::update_local_tranform
                            .label(Update::Transform)
                            .after(Update::ConstraintsSolve),
                    )
                    // .add_system_set_to_stage(
                    //     CoreStage::PreUpdate,
                    //     SystemSet::new()
                    //         .label(Physics::PostUpdate)
                    //         .with_run_criteria(run_physics)
                    //         .with_system(manifold_remove_expired_system),
                    // );

            )
            //TODO: Wish i could use run criteria on sub systems, but its not allowed
            // Static Collision Detection
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(Physics::Update)
                    .after(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_run_criteria(run_static)
                    .with_system(
                        narrow::narrowphase_system_static
                            .label(Update::Narrowphase)
                            .after(Update::Broadphase),
                    ),
            )
            // Dynamic Collision Detection
            .add_system_set_to_stage(
                CoreStage::PostUpdate,
                SystemSet::new()
                    .label(Physics::Update)
                    .after(Physics::PreUpdate)
                    .with_run_criteria(run_physics)
                    .with_run_criteria(run_dynamic)
                    .with_system(
                        narrow::narrowphase_system_dynamic
                            .label(Update::Narrowphase)
                            .after(Update::Broadphase),
                    ),
            );
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
            .insert(Bounded::<Aabb>::default());
    }
}

pub fn update_aabb(
     mut query: Query<(&Body, &mut Aabb)>,
     pt: Res<PhysicsTime>,
) {
    for (body, mut aabb) in query.iter_mut() {
        // expand the bounds by the linear velocity
        aabb.expand_velocity(body.linear_velocity * pt.time);
        const BOUNDS_EPS: f32 = 0.01;
        aabb.expand_velocity(Vec3::splat(BOUNDS_EPS));    
    }
}


fn run_physics(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.time_dilation != 0.0 && config.enabled {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn run_static(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.collision_dection == CollisionDetection::Static {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn run_dynamic(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.collision_dection == CollisionDetection::Dynamic {
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
