#![allow(dead_code)]

mod body;
mod shapes;

use crate::shapes::PyhsicsShape;
use bevy::prelude::*;
use bevy_inspector_egui::RegisterInspectable;
use body::*;

pub mod prelude {
    pub use crate::{body::*, shapes::*, PhysicsPlugin};
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    Detect,
    Resolve,
    Apply,
}

#[derive(Component)]
pub struct Gravity(Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Gravity(Vec3::new(0.0, -9.8, 0.0))
    }
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ContactEvent>()
            .add_system(update_system)
            .register_inspectable::<Body>();

        // incase user didn't supply a gravity
        app.init_resource::<Gravity>();
    }
}

// TODO: Rename, using
fn update_system(
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Body, &mut Transform)>,
) {
    let dt = time.delta_seconds();
    // apply gravity
    for (_, mut body, _) in query.iter_mut() {
        if body.inv_mass != 0.0 {
            // gravity needs to be an impulse
            // I = dp, F = dp/dt => dp = F * dt => I = F * dt
            // F = mgs
            let mass = 1.0 / body.inv_mass;
            let impluse_gravity = gravity.0 * mass * dt;
            body.apply_impulse_linear(impluse_gravity);
        }
    }

    // find contacts collisions
    let mut contacts = Vec::new();
    let mut iter = query.iter_combinations_mut();
    while let Some([(a, mut body_a, mut transform_a), (b, mut body_b, mut transform_b)]) = iter.fetch_next() {
        if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
            continue;
        }

        if let Some(contact) = intersect_test(a, &mut body_a, &mut transform_a, b, &mut body_b, &mut transform_b, dt) {
            contacts.push(contact);
        }
    }

    // sort the times of impact from earliest to latest
    contacts.sort_by(|a, b| {
        if a.time_of_impact < b.time_of_impact {
            std::cmp::Ordering::Less
        } else if a.time_of_impact == b.time_of_impact {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    });

    let mut accumulated_time = 0.0;
    for contact in contacts {
        let contact_time = contact.time_of_impact - accumulated_time;

        // position update
        for (_, mut body, mut transform) in query.iter_mut() {
            body.update( &mut transform, contact_time)
        }

        // SAFETY: There is no way to safey access the query twice at the same time I am aware of
        // Should be safe since entity a and b can't be the same
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (_, mut body_a, mut transform_a) = query.get_unchecked(contact.entity_a).unwrap();
            let (_, mut body_b, mut transform_b) = query.get_unchecked(contact.entity_b).unwrap();
            resolve_contact(contact, &mut body_a, &mut transform_a, &mut body_b, &mut transform_b);
        }
        accumulated_time += contact_time;
    }

    // update positions for the rest of this frame's time
    let time_remaining = dt - accumulated_time;
    for (_, mut body, mut transform) in query.iter_mut() {
        body.update(&mut transform, time_remaining)
    }

    // let ab = transform_b.translation - transform_a.translation;
    // let radius_ab = radius_a + radius_b;
    // let radius_ab_sq = radius_ab * radius_ab;
    // let ab_len_sq = ab.length_squared();
    // if ab_len_sq <= radius_ab_sq {
    //     let ab_normal = ab.normalize();
    //     contact_events.send(ContactEvent {
    //         entity_a: a,
    //         entity_b: b,
    //         world_point_a: transform_a.translation + ab_normal * radius_a,
    //         world_point_b: transform_b.translation - ab_normal * radius_b,
    //         normal: ab_normal,
    //     });
    // }
}

#[derive(Copy, Clone, Debug)]
pub struct ContactEvent {
    entity_a: Entity,
    entity_b: Entity,
    world_point_a: Vec3,
    world_point_b: Vec3,
    local_point_a: Vec3,
    local_point_b: Vec3,
    normal: Vec3,
    separation_dist: f32,
    time_of_impact: f32,
}

fn ray_sphere_intersect(
    ray_start: Vec3,
    ray_direction: Vec3,
    sphere_center: Vec3,
    sphere_radius: f32,
) -> Option<(f32, f32)> {
    let m = sphere_center - ray_start;
    let a = ray_direction.dot(ray_direction);
    let b = m.dot(ray_direction);
    let c = m.dot(m) - sphere_radius * sphere_radius;

    let delta = b * b - a * c;

    if delta < 0.0 {
        None
    } else {
        let inv_a = 1.0 / a;
        let delta_root = delta.sqrt();
        let t1 = inv_a * (b - delta_root);
        let t2 = inv_a * (b + delta_root);
        Some((t1, t2))
    }
}

fn sphere_sphere_dynamic(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
    vel_a: Vec3,
    vel_b: Vec3,
    dt: f32,
) -> Option<(Vec3, Vec3, f32)> {
    let relative_velocity = vel_a - vel_b;

    let start_pt_a = pos_a;
    let end_pt_a = pos_a + relative_velocity * dt;
    let ray_dir = end_pt_a - start_pt_a;

    let mut t0 = 0.0;
    let mut t1 = 0.0;

    const EPSILON: f32 = 0.001;
    const EPSILON_SQ: f32 = EPSILON * EPSILON;
    if ray_dir.length_squared() < EPSILON_SQ {
        // ray is too short, just check if intersecting
        let ab = pos_b - pos_a;
        let radius = radius_a + radius_b + EPSILON;
        if ab.length_squared() > radius * radius {
            return None;
        }
    } else if let Some(toi) = ray_sphere_intersect(pos_a, ray_dir, pos_b, radius_a + radius_b) {
        t0 = toi.0;
        t1 = toi.1;
    } else {
        return None;
    }

    // Change from [0,1] range to [0,dt] range
    t0 *= dt;
    t1 *= dt;

    // If the collision is only in the past, then there's not future collision this frame
    if t1 < 0.0 {
        return None;
    }

    // Get the earliest positive time of impact
    let toi = if t0 < 0.0 { 0.0 } else { t0 };

    // If the earliest collision is too far in the future, then there's no collision this frame
    if toi > dt {
        return None;
    }

    // get the points on the respective points of collision
    let new_pos_a = pos_a + vel_a * toi;
    let new_pos_b = pos_b + vel_b * toi;
    let ab = (new_pos_b - new_pos_a).normalize();

    let pt_on_a = new_pos_a + ab * radius_a;
    let pt_on_b = new_pos_b - ab * radius_b;

    Some((pt_on_a, pt_on_b, toi))
}

pub fn intersect_test(
    a: Entity,
    body_a: &mut Body,
    transform_a:  &mut Transform,
    b: Entity,
    body_b:  &mut Body,
    transform_b:  &mut Transform,
    dt: f32,
) -> Option<ContactEvent> {
    // skip body pairs with infinite mass
    if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
        return None;
    }

    // test for intersections, fire contact event if necessary
    let shapes = (body_a.shape, body_b.shape);
    match shapes {
        (PyhsicsShape::Sphere { radius: radius_a }, PyhsicsShape::Sphere { radius: radius_b }) => {
            if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                radius_a,
                radius_b,
                transform_a.translation,
                transform_b.translation,
                body_a.linear_velocity,
                body_b.linear_velocity,
                dt,
            ) {

                // step bodies forward to get local space collision points
                body_a.update( transform_a, time_of_impact);
                body_b.update(transform_b, time_of_impact);

                // convert world space contacts to local space
                let local_point_a = body_a.world_to_local(transform_a, world_point_a);
                let local_point_b = body_b.world_to_local(transform_b, world_point_b);

                let normal = (transform_a.translation - transform_b.translation).normalize();

                // unwind time step
                body_a.update( transform_a, -time_of_impact);
                body_b.update( transform_b, -time_of_impact);

                // calculate the separation distance
                let ab = transform_a.translation - transform_b.translation;
                let separation_dist = ab.length() - (radius_a + radius_b);

                Some(ContactEvent {
                    world_point_a,
                    world_point_b,
                    local_point_a,
                    local_point_b,
                    normal,
                    separation_dist,
                    time_of_impact,
                    entity_a: a,
                    entity_b: b,
                })
            } else {
                None
            }
        }
    }
}

fn resolve_contact(
    contact: ContactEvent,
    body_a:  &mut Body,
    transform_a: &mut Transform,
    body_b: &mut Body,
    transform_b: &mut Transform,
) {
    let elasticity = body_a.elasticity * body_b.elasticity;
    let total_inv_mass = body_a.inv_mass + body_b.inv_mass;

    let inv_inertia_world_a = body_a.get_inv_inertia_tensor_world(transform_a);
    let inv_inertia_world_b = body_b.get_inv_inertia_tensor_world(transform_b);

    let ra = contact.world_point_a - body_a.get_centre_of_mass_world(transform_a);
    let rb = contact.world_point_b - body_b.get_centre_of_mass_world(transform_b);

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