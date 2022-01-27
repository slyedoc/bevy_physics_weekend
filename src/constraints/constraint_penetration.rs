use bevy::prelude::*;

use super::ConstraintConfig;
use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
    primitives::*,
    PhysicsTime,
};

#[derive(Component, Copy, Clone, Debug)]
pub struct ConstraintPenetration {
    config: ConstraintConfig,

    jacobian: MatMN<3, 12>,
    cached_lambda: VecN<3>,
    normal: Vec3, // in body A's local space
    baumgarte: f32,
    friction: f32,
}

impl ConstraintPenetration {
    pub fn new(config: ConstraintConfig, normal: Vec3) -> Self {
        Self {
            config,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            normal,
            baumgarte: 0.0,
            friction: 0.0,
        }
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn clear_cached_lambda(&mut self) {
        self.cached_lambda = VecN::zero();
    }
}

pub fn pre_solve_system(
    mut commands: Commands,
    pt: Res<PhysicsTime>,
    mut query: Query<(Entity, &mut ConstraintPenetration)>,
    mut bodies: Query<(&mut Body, &mut GlobalTransform)>,
) {
    for (e, mut constraint) in query.iter_mut() {
        unsafe {
            let a = bodies.get_unchecked(constraint.config.handle_a);
            let b = bodies.get_unchecked(constraint.config.handle_a);
            if a.is_err() || b.is_err() {
                warn!("Pre Solve System Remove: {:?}", e);
                commands.entity(e).despawn();
                continue;
            }
            let (body_a, trans_a) = a.unwrap();
            let (body_b, trans_b) = b.unwrap();

            // get the world space position of the hinge from body_a's orientation
            let world_anchor_a = body_a.local_to_world(&trans_a, constraint.config.anchor_a);

            // get the world space position of the hinge from body_b's orientation
            let world_anchor_b = body_b.local_to_world(&trans_b, constraint.config.anchor_b);

            let ra = world_anchor_a - body_a.centre_of_mass_world(&trans_a);
            let rb = world_anchor_b - body_b.centre_of_mass_world(&trans_b);
            constraint.friction = body_a.friction * body_b.friction;

            // should be equivalent to Vec3::GetOrtho() from the book
            let (mut u, mut v) = constraint.normal.any_orthonormal_pair();

            // convert tangent space from model space to world space
            let normal = trans_a.rotation * constraint.normal;
            u = trans_a.rotation * u;
            v = trans_a.rotation * v;

            // penetration constraint
            constraint.jacobian = MatMN::zero();

            // first row is the primary distance constraint that holds the anchor points together
            {
                let j1 = -normal;
                constraint.jacobian.rows[0][0] = j1.x;
                constraint.jacobian.rows[0][1] = j1.y;
                constraint.jacobian.rows[0][2] = j1.z;
            }

            {
                let j2 = ra.cross(-normal);
                constraint.jacobian.rows[0][3] = j2.x;
                constraint.jacobian.rows[0][4] = j2.y;
                constraint.jacobian.rows[0][5] = j2.z;
            }

            {
                let j3 = normal;
                constraint.jacobian.rows[0][6] = j3.x;
                constraint.jacobian.rows[0][7] = j3.y;
                constraint.jacobian.rows[0][8] = j3.z;
            }

            {
                let j4 = rb.cross(normal);
                constraint.jacobian.rows[0][9] = j4.x;
                constraint.jacobian.rows[0][10] = j4.y;
                constraint.jacobian.rows[0][11] = j4.z;
            }

            // friction jacobians
            if constraint.friction > 0.0 {
                {
                    let j1 = -u;
                    constraint.jacobian.rows[1][0] = j1.x;
                    constraint.jacobian.rows[1][1] = j1.y;
                    constraint.jacobian.rows[1][2] = j1.z;
                }
                {
                    let j2 = ra.cross(-u);
                    constraint.jacobian.rows[1][3] = j2.x;
                    constraint.jacobian.rows[1][4] = j2.y;
                    constraint.jacobian.rows[1][5] = j2.z;
                }
                {
                    let j3 = u;
                    constraint.jacobian.rows[1][6] = j3.x;
                    constraint.jacobian.rows[1][7] = j3.y;
                    constraint.jacobian.rows[1][8] = j3.z;
                }
                {
                    let j4 = rb.cross(u);
                    constraint.jacobian.rows[1][9] = j4.x;
                    constraint.jacobian.rows[1][10] = j4.y;
                    constraint.jacobian.rows[1][11] = j4.z;
                }

                {
                    let j1 = -v;
                    constraint.jacobian.rows[2][0] = j1.x;
                    constraint.jacobian.rows[2][1] = j1.y;
                    constraint.jacobian.rows[2][2] = j1.z;
                }
                {
                    let j2 = ra.cross(-v);
                    constraint.jacobian.rows[2][3] = j2.x;
                    constraint.jacobian.rows[2][4] = j2.y;
                    constraint.jacobian.rows[2][5] = j2.z;
                }
                {
                    let j3 = v;
                    constraint.jacobian.rows[2][6] = j3.x;
                    constraint.jacobian.rows[2][7] = j3.y;
                    constraint.jacobian.rows[2][8] = j3.z;
                }
                {
                    let j4 = rb.cross(v);
                    constraint.jacobian.rows[2][9] = j4.x;
                    constraint.jacobian.rows[2][10] = j4.y;
                    constraint.jacobian.rows[2][11] = j4.z;
                }
            }

            // apply warm starting from last frame
            let impulses = constraint.jacobian.transpose() * constraint.cached_lambda;
            constraint.config.apply_impulses(&mut bodies, impulses);

            // calculate the baumgarte stabilization
            let mut c = (world_anchor_b - world_anchor_a).dot(normal);
            c = f32::min(0.0, c + 0.02); // add slop
            let beta = 0.25;
            constraint.baumgarte = beta * c / pt.time;
        }
    }
}

pub fn solve_system(
    mut commands: Commands,
    mut query: Query<(Entity, &mut ConstraintPenetration)>,
    mut bodies: Query<(&mut Body, &mut GlobalTransform)>,
) {
    for (e, mut constraint) in query.iter_mut() {
        unsafe {
            let a = bodies.get_unchecked(constraint.config.handle_a);
            let b = bodies.get_unchecked(constraint.config.handle_a);
            if a.is_err() || b.is_err() {
                warn!("Solve System Remove: {:?}", e);
                commands.entity(e).despawn();
                continue;
            }
            let (body_a, _trans_a) = a.unwrap();
            let (body_b, _trans_b) = b.unwrap();

            let jacobian_transpose = constraint.jacobian.transpose();

            // build the system of equations
            let q_dt = constraint.config.get_velocities(&mut bodies);
            let inv_mass_matrix = constraint.config.get_inverse_mass_matrix(&mut bodies);
            let j_w_jt = constraint.jacobian * inv_mass_matrix * jacobian_transpose;
            let mut rhs = constraint.jacobian * q_dt * -1.0;
            rhs[0] -= constraint.baumgarte;

            // solve for the Lagrange multipliers
            let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

            // accumulate the impulses and clamp within the constraint limits
            let old_lambda = constraint.cached_lambda;
            constraint.cached_lambda += lambda_n;
            let lambda_limit = 0.0;
            if constraint.cached_lambda[0] < lambda_limit {
                constraint.cached_lambda[0] = lambda_limit;
            }

            if constraint.friction > 0.0 {
                let umg = constraint.friction * 10.0 * 1.0 / (body_a.inv_mass + body_b.inv_mass);
                let normal_force = (lambda_n[0] * constraint.friction).abs();
                let max_force = if umg > normal_force {
                    umg
                } else {
                    normal_force
                };

                if constraint.cached_lambda[1] > max_force {
                    constraint.cached_lambda[1] = max_force;
                }
                if constraint.cached_lambda[1] < -max_force {
                    constraint.cached_lambda[1] = -max_force;
                }

                if constraint.cached_lambda[2] > max_force {
                    constraint.cached_lambda[2] = max_force;
                }
                if constraint.cached_lambda[2] < -max_force {
                    constraint.cached_lambda[2] = -max_force;
                }
            }
            lambda_n = constraint.cached_lambda - old_lambda;

            // apply the impulses
            let impulses = jacobian_transpose * lambda_n;
            constraint.config.apply_impulses(&mut bodies, impulses);
        }
    }
}
