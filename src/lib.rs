#![allow(dead_code)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
#![feature(vec_retain_mut)]
#![feature(div_duration)]

pub mod colliders;
pub mod constraints;
pub mod debug_render;
pub mod intersect;
mod math;
mod phase;
pub mod primitives;

use colliders::{Collider, ColliderBox, ColliderSphere};
use primitives::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::Inspectable;
use phase::*;

use bevy_polyline::*;

#[derive(Inspectable, PartialEq, Eq)]
pub enum Mode {
    Static,
    Dynamic,
}
#[derive(Component, Inspectable)]
pub struct PhysicsConfig {
    pub enabled: bool,
    pub mode: Mode,
    #[inspectable(min = -10.0, max = 10.0)]
    pub time_dilation: f32,
    pub gravity: Vec3,
    pub constrain_max_iter: usize,
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
        }
    }
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    First,
    PreUpdate,
    Dynamics,
    Broadphase,
    NarrowphaseBreakout,
    Narrowphase,
    Manifold,
    ConstraintsPreSolve,
    ConstraintsSolve,
    ResolveContact,
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Contact>()
            .add_event::<BroadContact>()
            .add_event::<ManifoldContactEvent>()
            // NOTE: For now I am laying out the system very explicitly so that its easier to move things around
            .add_system_set_to_stage(
                CoreStage::First,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .with_system(timestep_system)
                    .with_system(collider_add_watch::<ColliderSphere>)
                    .with_system(collider_add_watch::<ColliderBox>),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::First)
                    .with_system(calucate_aabb::<ColliderSphere>)
                    .with_system(calucate_aabb::<ColliderBox>),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Dynamics)
                    .after(PhysicsSystem::First)
                    .with_system(dynamics::dynamics_gravity_system),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Broadphase)
                    .after(PhysicsSystem::Dynamics)
                    .with_system(broadphase::broadphase_system),
            );

            app.add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .with_run_criteria(run_static)
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::Broadphase)
                    .with_system(narrowphase::narrowphase_system_static),
            );
            app.add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .with_run_criteria(run_dynamic)
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::Broadphase)
                    .with_system(narrowphase::narrowphase_system_dynamic),
            );
            app.add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::ResolveContact)
                    .after(PhysicsSystem::Narrowphase)
                    .with_system(resolve_contact::resolve_contact_system_static),
            );

        //.with_system(manifold_remove_expired_system),

        // .add_system_set(
        //     SystemSet::new()
        //         .with_run_criteria(run_physics)
        //         .label(PhysicsSystem::ConstraintsPreSolve)
        //         .after(PhysicsSystem::Narrowphase)
        //         .with_system(constraints::constraint_penetration::pre_solve_system), //.with_system(constraints::post_solve_system::<ConstraintPenetration>)
        // )
        // .add_system_set(
        //     SystemSet::new()
        //         .with_run_criteria(run_physics)
        //         .label(PhysicsSystem::ConstraintsSolve)
        //         .after(PhysicsSystem::ConstraintsPreSolve)
        //         .with_system(constraints::constraint_penetration::solve_system),
        // );
        // // TODO: Make this optional
        // .add_system_set(
        //     SystemSet::new()
        //         .after(PhysicsSystem::ConstraintsSolve)
        //         .with_run_criteria(run_physics)
        //         .with_system(report_system),
        // )

        // Debug Render
        // TODO: Make this optional
        app.add_plugin(PolylinePlugin).add_system_set_to_stage(
            CoreStage::PreUpdate,
            SystemSet::new()
                .after(PhysicsSystem::ResolveContact)
                .with_system(debug_render::report_system),
        );

        // Add resources
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsTime>();
    }

    fn name(&self) -> &str {
        std::any::type_name::<Self>()
    }
}

pub fn collider_add_watch<T: Component + Collider>(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform, &T), Added<T>>,
) {
    for (e, trans, collider) in query.iter() {
        commands.entity(e).insert(collider.aabb(trans));
        commands.entity(e).insert(collider.shape_type());
    }
}

pub fn calucate_aabb<T: Component + Collider>(
    mut query: Query<(&GlobalTransform, &Body, &T, &mut Aabb)>,
    pt: Res<PhysicsTime>,
    config: Res<PhysicsConfig>,
) {
    #[allow(unused)]
    for (trans, body, collider, mut aabb) in query.iter_mut() {
        // TODO: remove this copy and use
        let mut new_aabb = collider.aabb(trans);

        // expand the bounds by the linear velocity
        if config.mode ==  Mode::Dynamic {
            new_aabb.expand_by_point(new_aabb.mins + body.linear_velocity * pt.time);
            new_aabb.expand_by_point(new_aabb.maxs + body.linear_velocity * pt.time);
        }

        const BOUNDS_EPS: f32 = 0.01;
        new_aabb.expand_by_point(new_aabb.mins - Vec3::splat(BOUNDS_EPS));
        new_aabb.expand_by_point(new_aabb.maxs + Vec3::splat(BOUNDS_EPS));

        aabb.mins = new_aabb.mins;
        aabb.maxs = new_aabb.maxs;
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
    if config.time_dilation != 0.0 && config.enabled {
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

fn timestep_system(time: Res<Time>, config: Res<PhysicsConfig>, mut pt: ResMut<PhysicsTime>) {
    // NOTE: I am avoiding using fixed time, thats because
    // we want to develop the hot path of the physics system
    pt.time = time.delta_seconds() * config.time_dilation;
}
