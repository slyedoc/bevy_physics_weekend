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
            .add_system(apply_forces.label(PhysicsSystem::Detect))
            .add_system(
                resolve_contact
                    .label(PhysicsSystem::Resolve)
                    .after(PhysicsSystem::Detect),
            )
            .add_system(
                update_bodies
                    .label(PhysicsSystem::Apply)
                    .after(PhysicsSystem::Resolve),
            )
            .register_inspectable::<Body>();

        // incase user didn't supply a gravity
        app.init_resource::<Gravity>();
    }
}

fn apply_forces(
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Body, &mut Transform)>,
    mut contact_events: EventWriter<ContactEvent>,
) {
    let dt = time.delta_seconds();
    // apply gravity
    for (_, mut body, _) in query.iter_mut() {
        let mass = 1.0 / body.inv_mass;
        let impluse_gravity = gravity.0 * mass * dt;
        body.apply_impulse_linear(impluse_gravity);
    }

    // check for collisions
    let mut iter = query.iter_combinations_mut();
    while let Some([(a, a_body, a_transform), (b, b_body, b_transform)]) = iter.fetch_next() {
        // skip body pairs with infinite mass
        if a_body.inv_mass == 0.0 && b_body.inv_mass == 0.0 {
            continue;
        }

        // test for intersections, fire contact event if necessary
        let shapes = (a_body.shape, b_body.shape);
        match shapes {
            (
                PyhsicsShape::Sphere { radius: radius_a },
                PyhsicsShape::Sphere { radius: radius_b },
            ) => {
                let ab = b_transform.translation - a_transform.translation;
                let radius_ab = radius_a + radius_b;
                let radius_ab_sq = radius_ab * radius_ab;
                let ab_len_sq = ab.length_squared();
                if ab_len_sq <= radius_ab_sq {
                    let ab_normal = ab.normalize();
                    contact_events.send(ContactEvent {
                        entity_a: a,
                        entity_b: b,
                        world_point_a: a_transform.translation + ab_normal * radius_a,
                        world_point_b: b_transform.translation - ab_normal * radius_b,
                        normal: ab_normal,
                    });
                }
            }
        };
    }
}

fn resolve_contact(
    mut contact_events: EventReader<ContactEvent>,
    query: Query<(&mut Body, &mut Transform)>,
) {
    for contact in contact_events.iter() {
        // info!("contact {:?}", contact);
        // SAFETY: There is no way to safey access the query twice at the same time I am aware of
        // Should be safe since entity a and b can't be the same
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (mut a_body, mut a_transform) = query.get_unchecked(contact.entity_a).unwrap();
            let (mut b_body, mut b_transform) = query.get_unchecked(contact.entity_b).unwrap();

            let elasticity = a_body.elasticity * b_body.elasticity;
            let total_inv_mass = a_body.inv_mass + b_body.inv_mass;
            
            let inv_inertia_world_a = a_body.get_inv_inertia_tensor_world(*a_transform);
            let inv_inertia_world_b = b_body.get_inv_inertia_tensor_world(*b_transform);

            let ra = contact.world_point_a - a_body.get_centre_of_mass_world(*a_transform);
            let rb = contact.world_point_b - b_body.get_centre_of_mass_world(*b_transform);

            let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
            let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
            let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

            // Get the world space velocity of the motion and rotation
            let vel_a = a_body.linear_velocity + a_body.angular_velocity.cross(ra);
            let vel_b = b_body.linear_velocity + b_body.angular_velocity.cross(rb);

            // Calculate the collion impulse
            let vab = vel_a - vel_b;
            let impluse_j = -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
            let impluse_vec_j = contact.normal * impluse_j;

            a_body.apply_impulse(contact.world_point_a, impluse_vec_j, *a_transform);
            b_body.apply_impulse(contact.world_point_b, -impluse_vec_j, *b_transform);

            // Calculate the friction impulse
            let friction = a_body.friction * b_body.friction;

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

            // TODO: Book didnt have this check, but was getitng velocity_tangent of zero if sphere is lined up with ground perfectly
            if !impluse_friction.is_nan() {
                // apply kinetic friction
                a_body.apply_impulse(contact.world_point_a, -impluse_friction, *a_transform);
                b_body.apply_impulse(contact.world_point_b, impluse_friction, *b_transform);
            }

            // Let's also move our colliding objects to just outside of each other
            let a_move_weight = a_body.inv_mass / total_inv_mass;
            let b_move_weight = b_body.inv_mass / total_inv_mass;

            let direction = contact.world_point_b - contact.world_point_a;

            a_transform.translation += direction * a_move_weight;
            b_transform.translation -= direction * b_move_weight;
        }
    }
}

fn update_bodies(mut query: Query<(&mut Body, &mut Transform)>, time: Res<Time>) {

    let dt = time.delta_seconds();
    for ( mut body, mut t) in query.iter_mut() {
        // apply linear velocity
        t.translation += body.linear_velocity * dt;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model

        let position_com = body.get_centre_of_mass_world(*t);
        let com_to_position = t.translation - position_com;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(t.rotation);
        let inertia_tensor = orientation * body.shape.inertia_tensor() * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (body
                .angular_velocity
                .cross(inertia_tensor * body.angular_velocity));
                body.angular_velocity += alpha * dt;

        // update orientation
        let d_angle = body.angular_velocity * dt;
        let angle = d_angle.length();
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::IDENTITY
        };
        t.rotation = (dq * t.rotation).normalize();

        // now get the new body position
        t.translation = position_com + dq * com_to_position;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ContactEvent {
    entity_a: Entity,
    entity_b: Entity,

    world_point_a: Vec3,
    world_point_b: Vec3,
    // local_point_a: Vec3,
    // local_point_b: Vec3,
    normal: Vec3,
    // separation_dist: f32,
    // time_of_impact: f32,

    // body_a: &'a mut Body,
    // body_b: &'a mut Body,
}

