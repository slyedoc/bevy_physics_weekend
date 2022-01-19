#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(clippy::type_complexity)]
mod bounds;
mod colliders;
mod constraints;
mod contact;
mod intersect;
mod manifold;
mod math;
mod body;
mod gjk;

use std::borrow::BorrowMut;

use bevy::prelude::*;
use colliders::*;
use intersect::*;
use manifold::*;
use prelude::{ConstraintArena, ContactEvent};
use body::*;

pub mod prelude {
    pub use crate::{colliders::*, constraints::*, contact::*, body::*, PhysicsPlugin};
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    ApplyGravity,
    CollisionDetection,
}

#[derive(Component)]
pub struct PhysicsResource {
    gravity: Vec3,
    constrain_max_iter: usize, // solve constraints,
}

impl Default for PhysicsResource {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            constrain_max_iter: 5,
        }
    }
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_system(apply_gravity.label(PhysicsSystem::ApplyGravity));
        app.add_system(
            colision_detection
                .label(PhysicsSystem::CollisionDetection)
                .after(PhysicsSystem::ApplyGravity),
        );

        // incase user didn't supply a gravity
        app.init_resource::<PhysicsResource>();
        app.init_resource::<ManifoldCollector>();
        app.init_resource::<ConstraintArena>();
    }
}

fn apply_gravity(config: Res<PhysicsResource>, time: Res<Time>, mut query: Query<&mut Body>) {
    let dt = time.delta_seconds();
    for mut body in query.iter_mut() {
        if body.inv_mass != 0.0 {
            // gravity needs to be an impulse
            // I = dp, F = dp/dt => dp = F * dt => I = F * dt
            // F = mgs
            let mass = 1.0 / body.inv_mass;
            let impluse_gravity = config.gravity * mass * dt;
            body.apply_impulse_linear(impluse_gravity);
        }
    }
}

fn colision_detection(
    config: Res<PhysicsResource>,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Body, &mut Transform)>,
    mut manifolds: ResMut<ManifoldCollector>,
    mut constraints: ResMut<ConstraintArena>,
) {
    let dt = time.delta_seconds();

    manifolds.remove_expired(&query);

    // Broadphase (build potential collision pairs)

    // running sweep and prune in 1 direction to test for intersection
    // TODO: reuse sorted_bodies vec to avoid allocation, test with local
    let mut sorted_bodies = Vec::<PsuedoBody>::new();
    let axis = Vec3::ONE.normalize();
    for (entity, body, t) in query.iter_mut() {
        let mut bounds = body.collider.bounds(&t);

        // expand the bounds by the linear velocity
        bounds.expand_by_point(bounds.mins + body.linear_velocity * dt);
        bounds.expand_by_point(bounds.maxs + body.linear_velocity * dt);

        const BOUNDS_EPS: f32 = 0.01;
        bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        sorted_bodies.push(PsuedoBody {
            entity,
            value: axis.dot(bounds.mins),
            is_min: true,
        });
        sorted_bodies.push(PsuedoBody {
            entity,
            value: axis.dot(bounds.maxs),
            is_min: false,
        });
    }
    sorted_bodies.sort_unstable_by(compare_sat);
    let possable_collision_pairs = build_pairs(&sorted_bodies);

    // Narrowphase (perform actual collision detection)

    // test possable contacts collisions
    // TODO: reuse sorted_bodies vec to avoid allocation, test with local
    let mut contacts = Vec::new();
    for pair in possable_collision_pairs {
        // SAFETY: There is no way for a and b to the same entity
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (_, mut body_a, mut transform_a) = query.get_unchecked(pair.a).unwrap();
            let (_, mut body_b, mut transform_b) = query.get_unchecked(pair.b).unwrap();

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            // check for intersection
            if let Some(contact) = intersect_dynamic(
                pair.a,
                &mut body_a,
                &mut transform_a,
                pair.b,
                &mut body_b,
                &mut transform_b,
                dt,
            ) {
                if contact.time_of_impact == 0.0 {
                    // static contact
                    manifolds.add_contact(&query, contact);
                } else {
                    // ballistic contact
                    contacts.push(contact)
                }
            }
        }
    }

    // sort the times of impact from earliest to latest
    contacts.sort_unstable_by(|a, b| {
        if a.time_of_impact < b.time_of_impact {
            std::cmp::Ordering::Less
        } else if a.time_of_impact == b.time_of_impact {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    });

    // solve constraints
    constraints.pre_solve(&mut query, dt);
    manifolds.pre_solve(&mut query, dt);


    for _ in 0..config.constrain_max_iter {
        constraints.solve(&mut query);
        manifolds.solve(&mut query);
    }

    constraints.post_solve();
    manifolds.post_solve();

    // Apply Ballistic impulses
    let mut accumulated_time = 0.0;
    for contact in contacts {
        let contact_time = contact.time_of_impact - accumulated_time;

        // position update
        for (_, mut body, mut transform) in query.iter_mut() {
            body.update(&mut transform, contact_time)
        }

        // SAFETY: There is no way to safey access the query twice at the same time I am aware of
        // Should be safe since entity a and b can't be the same
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (_, mut body_a, mut transform_a) = query.get_unchecked(contact.entity_a).unwrap();
            let (_, mut body_b, mut transform_b) = query.get_unchecked(contact.entity_b).unwrap();
            resolve_contact(
                contact,
                &mut body_a,
                &mut transform_a,
                &mut body_b,
                &mut transform_b,
            );
        }
        accumulated_time += contact_time;
    }

    // update positions for the rest of this frame's time
    let time_remaining = dt - accumulated_time;
    if time_remaining > 0.0 {
        for (_, mut body, mut transform) in query.iter_mut() {
            body.update(&mut transform, time_remaining)
        }
    }
}

fn build_pairs(sorted_bodies: &[PsuedoBody]) -> Vec<CollisionPair> {
    let mut collision_pairs = Vec::new();

    // Now that the bodies are sorted, build the collision pairs
    for i in 0..sorted_bodies.len() {
        let a = &sorted_bodies[i];
        if !a.is_min {
            continue;
        }

        for b in sorted_bodies.iter().skip(i + 1) {
            // if we've hit the end of the a element then we're done creating pairs with a
            if b.entity == a.entity {
                break;
            }

            if !b.is_min {
                continue;
            }

            collision_pairs.push(CollisionPair {
                a: a.entity,
                b: b.entity,
            });
        }
    }

    collision_pairs
}

fn resolve_contact(
    contact: ContactEvent,
    body_a: &mut Body,
    transform_a: &mut Transform,
    body_b: &mut Body,
    transform_b: &mut Transform,
) {
    let elasticity = body_a.elasticity * body_b.elasticity;
    let total_inv_mass = body_a.inv_mass + body_b.inv_mass;

    let inv_inertia_world_a = body_a.inv_inertia_tensor_world(transform_a);
    let inv_inertia_world_b = body_b.inv_inertia_tensor_world(transform_b);

    let ra = contact.world_point_a - body_a.centre_of_mass_world(transform_a);
    let rb = contact.world_point_b - body_b.centre_of_mass_world(transform_b);

    let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
    let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
    let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

    // Get the world space velocity of the motion and rotation
    let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(ra);
    let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(rb);

    // Calculate the collion impulse
    let vab = vel_a - vel_b;
    let impluse_j =
        -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
    let impluse_vec_j = contact.normal * impluse_j;

    body_a.apply_impulse(contact.world_point_a, impluse_vec_j, transform_a);
    body_b.apply_impulse(contact.world_point_b, -impluse_vec_j, transform_b);

    // Calculate the friction impulse
    let friction = body_a.friction * body_b.friction;

    // Find the normal direction of the velocity with respoect to the normal of the collison
    let velocity_normal = contact.normal * contact.normal.dot(vab);
    let velocity_tangent = vab - velocity_normal;

    // Get the tangent velocities relative to the other body
    let relative_velocity_tangent = velocity_tangent.normalize();

    let inertia_a = (inv_inertia_world_a * ra.cross(relative_velocity_tangent)).cross(ra);
    let inertia_b = (inv_inertia_world_b * rb.cross(relative_velocity_tangent)).cross(rb);
    let inv_inertia = (inertia_a + inertia_b).dot(relative_velocity_tangent);

    // calculat the tangential impluse for friction
    let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
    let impluse_friction = velocity_tangent * (reduced_mass * friction);

    // TODO: Book didnt have this if check, but I was getitng velocity_tangent of zero leading to
    // a Vec3 Nan when normalized if perfectly lined up on ground
    if !impluse_friction.is_nan() {
        // apply kinetic friction
        body_a.apply_impulse(contact.world_point_a, -impluse_friction, transform_a);
        body_b.apply_impulse(contact.world_point_b, impluse_friction, transform_b);
    }

    // lets move the bodies part if intersecting
    if contact.time_of_impact == 0.0 {
        // Let's also move our colliding objects to just outside of each other
        let a_move_weight = body_a.inv_mass / total_inv_mass;
        let b_move_weight = body_b.inv_mass / total_inv_mass;

        let direction = contact.world_point_b - contact.world_point_a;

        transform_a.translation += direction * a_move_weight;
        transform_b.translation -= direction * b_move_weight;
    }
}

#[derive(Copy, Clone, Debug)]
struct PsuedoBody {
    entity: Entity,
    value: f32,
    is_min: bool,
}

fn compare_sat(a: &PsuedoBody, b: &PsuedoBody) -> std::cmp::Ordering {
    if a.value < b.value {
        std::cmp::Ordering::Less
    } else if a.value > b.value {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

#[derive(Copy, Clone, Debug)]
struct CollisionPair {
    a: Entity,
    b: Entity,
}

impl PartialEq for CollisionPair {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for CollisionPair {}
