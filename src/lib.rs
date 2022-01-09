#![allow(dead_code)]

mod body;
mod shapes;

use bevy::prelude::*;
use body::*;
use shapes::PyhsicsShape;
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
            .add_system(update_bodies.label(PhysicsSystem::Detect))
            .add_system(
                resolve_contact
                    .label(PhysicsSystem::Resolve)
                    .after(PhysicsSystem::Detect),
            )
            .add_system(
                apply_linear_velocity
                    .label(PhysicsSystem::Apply)
                    .after(PhysicsSystem::Resolve),
            );

        app.init_resource::<Gravity>();
    }
}

fn update_bodies(
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Body, &mut GlobalTransform)>,
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
                let ab = a_transform.translation - b_transform.translation;
                let radius_ab = radius_a + radius_b;
                let radius_ab_sq = radius_ab * radius_ab;
                let ab_len_sq = ab.length_squared();
                if ab_len_sq <= radius_ab_sq {
                    let ab_normal = ab.normalize();
                    contact_events.send(ContactEvent {
                        entity_a: a,
                        entity_b: b,
                        world_point_a: a_transform.translation + ab_normal * radius_a,
                        world_point_b: a_transform.translation - ab_normal * radius_b,
                        normal: ab_normal,
                    });
                }
            }
        };
    }
}

fn resolve_contact(
    mut contact_events: EventReader<ContactEvent>,
    mut query: Query<(Entity, &mut Body, &mut GlobalTransform)>,
) {
    for contact in contact_events.iter() {

        // SAFETY: There no safe way to access the query twice the same time
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (a, mut a_body, mut a_transform) = query.get_unchecked(contact.entity_a).unwrap();
            let (b, mut b_body, mut b_transform) = query.get_unchecked(contact.entity_b).unwrap();

            a_body.linear_velocity = Vec3::ZERO;
            b_body.linear_velocity = Vec3::ZERO;

            // let tA = a_body.inv_mass / (a_body.inv_mass + b_body.inv_mass);
            // let tB = b_body.inv_mass / (a_body.inv_mass + b_body.inv_mass);

            // let ds = contact.world_point_a - contact.world_point_b;
            // a_transform.translation += ds * tA;
            // b_transform.translation += ds * tB;
        }
    }
}

fn apply_linear_velocity(
    mut query: Query<(Entity, &mut Body, &mut GlobalTransform)>,
    time: Res<Time>,
) {
    // apply linear velocity
    let dt = time.delta_seconds();
    for (_, body, mut global_transform) in query.iter_mut() {
        global_transform.translation += body.linear_velocity * dt;
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
