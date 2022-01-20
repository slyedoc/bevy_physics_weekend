#![allow(dead_code)]
mod constraint_constant_velocity;
mod constraint_distance;
mod constraint_hinge_quat;
mod constraint_motor;
mod constraint_mover;
mod constraint_orientation;
mod constraint_penetration;

use crate::{
    math::{MatMN, VecN},
    body::Body,
};
use bevy::prelude::*;
use constraint_constant_velocity::ConstraintConstantVelocityLimited;
use constraint_distance::ConstraintDistance;
use constraint_hinge_quat::ConstraintHingeQuatLimited;
use constraint_motor::ConstraintMotor;
use constraint_mover::ConstraintMoverSimple;
use constraint_orientation::ConstraintOrientation;
pub use constraint_penetration::ConstraintPenetration;

pub fn quat_left(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, -q.z, q.y),
        Vec4::new(q.y, q.z, q.w, -q.x),
        Vec4::new(q.z, -q.y, q.x, q.w),
    )
}

pub fn quat_right(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, q.z, -q.y),
        Vec4::new(q.y, -q.z, q.w, q.x),
        Vec4::new(q.z, q.y, -q.x, q.w),
    )
}

pub trait Constraint: Send + Sync {
    fn pre_solve(
        &mut self,
        query: &mut Query<(Entity, &mut Body, &mut Transform)>,
        dt_sec: f32,
    );
    fn solve(&mut self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>);
    fn post_solve(&mut self) {}
}

#[derive(Default)]
pub struct ConstraintArena {
    constraints: Vec<Box<dyn Constraint>>,
}


impl ConstraintArena {
    pub fn clear(&mut self) {
        self.constraints.clear();
    }

    pub fn add_orientation_constraint(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        handle_a: Entity,
        handle_b: Entity,
    ) {
        unsafe {
        let (_, body_a, trans_a) = bodies.get_unchecked(handle_a).unwrap();
        let (_, body_b, trans_b) = bodies.get_unchecked(handle_b).unwrap();

        let world_space_anchor = trans_a.translation;

        self.constraints.push(Box::new(ConstraintOrientation::new(
            ConstraintConfig {
                handle_a: Some(handle_a),
                handle_b: Some(handle_b),
                anchor_a: body_a.world_to_local(&trans_a, world_space_anchor),
                anchor_b: body_b.world_to_local(&trans_b, world_space_anchor),
                ..ConstraintConfig::default()
            },
            trans_a.rotation.inverse() * trans_b.rotation,
        )))
    }
    }

    pub fn add_distance_constraint(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        handle_a: Entity,
        handle_b: Entity,
    ) {
        unsafe {
        let (_, body_a, trans_a) = bodies.get_unchecked(handle_a).unwrap();
        let (_, body_b, trans_b) = bodies.get_unchecked(handle_b).unwrap();

        let joint_world_space_anchor = trans_a.translation;

        let anchor_a = body_a.world_to_local(&trans_a, joint_world_space_anchor);
        let anchor_b = body_b.world_to_local(&trans_b, joint_world_space_anchor);

        self.constraints
            .push(Box::new(ConstraintDistance::new(ConstraintConfig {
                handle_a: Some(handle_a),
                handle_b: Some(handle_b),
                anchor_a,
                axis_a: Vec3::ZERO,
                anchor_b,
                axis_b: Vec3::ZERO,
            })))
        }
        
    }

    pub fn add_hinge_constraint(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        handle_a: Entity,
        handle_b: Entity,
        world_space_anchor: Vec3,
        axis: Vec3,
    ) {
        unsafe {
        let (_, body_a, trans_a) = bodies.get_unchecked(handle_a).unwrap();
        let (_, body_b, trans_b) = bodies.get_unchecked(handle_b).unwrap();

        let relative_orientation = trans_a.rotation.inverse() * trans_b.rotation;

        self.constraints
            .push(Box::new(ConstraintHingeQuatLimited::new(
                ConstraintConfig {
                    handle_a: Some(handle_a),
                    handle_b: Some(handle_b),
                    anchor_a: body_a.world_to_local(&trans_a, world_space_anchor),
                    anchor_b: body_b.world_to_local(&trans_b, world_space_anchor),
                    axis_a: axis,
                    axis_b: Vec3::ZERO,
                },
                relative_orientation,
            )))
        }
    }

    pub fn add_constant_velocity_constraint(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        handle_a: Entity,
        handle_b: Entity,
        world_space_anchor: Vec3,
        axis: Vec3,
    ) {
        unsafe {
        
        let (_, body_a, trans_a) = bodies.get_unchecked(handle_a).unwrap();
        let (_, body_b, trans_b) = bodies.get_unchecked(handle_b).unwrap();

        let relative_orientation = trans_a.rotation.inverse() * trans_b.rotation;

        self.constraints
            .push(Box::new(ConstraintConstantVelocityLimited::new(
                ConstraintConfig {
                    handle_a: Some(handle_a),
                    handle_b: Some(handle_b),
                    anchor_a: body_a.world_to_local(&trans_a, world_space_anchor),
                    anchor_b: body_b.world_to_local(&trans_b,world_space_anchor),
                    axis_a: axis,
                    axis_b: Vec3::ZERO,
                },
                relative_orientation,
            )))
        }
    }

    pub fn add_constraint_motor(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        handle_a: Entity,
        handle_b: Entity,
        world_space_anchor: Vec3,
        motor_axis: Vec3,
        motor_speed: f32,
    ) {
        unsafe {
        let (_, body_a, trans_a) = bodies.get_unchecked(handle_a).unwrap();
        let (_, body_b, trans_b) = bodies.get_unchecked(handle_b).unwrap();

        // set the initial relative orientation (in body_a's space)
        let q0 = trans_a.rotation.inverse() * trans_b.rotation;

        self.constraints.push(Box::new(ConstraintMotor::new(
            ConstraintConfig {
                handle_a: Some(handle_a),
                handle_b: Some(handle_b),
                anchor_a: body_a.world_to_local(&trans_a, world_space_anchor),
                anchor_b: body_b.world_to_local(&trans_b, world_space_anchor),
                ..ConstraintConfig::default()
            },
            q0,
            motor_axis,
            motor_speed,
        )))
    }
    }

    pub fn add_constraint_mover(&mut self, _bodies: &mut Query<(Entity, &mut Body, &mut Transform)>, handle_a: Entity) {
        self.constraints
            .push(Box::new(ConstraintMoverSimple::new(ConstraintConfig {
                handle_a: Some(handle_a),
                ..ConstraintConfig::default()
            })))
    }

    pub fn pre_solve(&mut self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>, dt_sec: f32) {
        for constraint in &mut self.constraints {
            constraint.pre_solve(bodies, dt_sec);
        }
    }

    pub fn solve(&mut self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>) {
        for constraint in &mut self.constraints {
            constraint.solve(bodies);
        }
    }

    pub fn post_solve(&mut self) {
        for constraint in &mut self.constraints {
            constraint.post_solve();
        }
    }
}

#[derive(Copy, Default, Clone, Debug)]
pub struct ConstraintConfig {
    pub handle_a: Option<Entity>,
    pub handle_b: Option<Entity>,

    pub anchor_a: Vec3, // the anchor location in body_a's space
    pub axis_a: Vec3,   // the axis direction in body_a's space

    pub anchor_b: Vec3, // the anchor location in body_b's space
    pub axis_b: Vec3,   // the axis direction in body_b's space
}

impl ConstraintConfig {
    fn get_inverse_mass_matrix(
        &self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
    ) -> MatMN<12, 12> {
        let mut inv_mass_matrix = MatMN::zero();

        {
            let (_, body_a, trans_a) = bodies.get_mut(self.handle_a.unwrap()).unwrap();


            inv_mass_matrix.rows[0][0] = body_a.inv_mass;
            inv_mass_matrix.rows[1][1] = body_a.inv_mass;
            inv_mass_matrix.rows[2][2] = body_a.inv_mass;

            let inv_intertia_a = body_a.inv_inertia_tensor_world(&trans_a);
            for i in 0..3 {
                inv_mass_matrix.rows[3 + i][3] = inv_intertia_a.col(i)[0];
                inv_mass_matrix.rows[3 + i][3 + 1] = inv_intertia_a.col(i)[1];
                inv_mass_matrix.rows[3 + i][3 + 2] = inv_intertia_a.col(i)[2];
            }
        }

        {
            let (_, body_b, trans_b) = bodies.get_mut(self.handle_b.unwrap()).unwrap();
            inv_mass_matrix.rows[6][6] = body_b.inv_mass;
            inv_mass_matrix.rows[7][7] = body_b.inv_mass;
            inv_mass_matrix.rows[8][8] = body_b.inv_mass;

            let inv_intertia_b = body_b.inv_inertia_tensor_world(&trans_b);
            for i in 0..3 {
                inv_mass_matrix.rows[9 + i][9] = inv_intertia_b.col(i)[0];
                inv_mass_matrix.rows[9 + i][9 + 1] = inv_intertia_b.col(i)[1];
                inv_mass_matrix.rows[9 + i][9 + 2] = inv_intertia_b.col(i)[2];
            }
        }

        inv_mass_matrix
    }

    fn get_velocities(&self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>) -> VecN<12> {
        let mut q_dt = VecN::zero();

        {
            let (_, body_a, _trans_a) = bodies.get_mut(self.handle_a.unwrap()).unwrap();
  

            q_dt[0] = body_a.linear_velocity.x;
            q_dt[1] = body_a.linear_velocity.y;
            q_dt[2] = body_a.linear_velocity.z;

            q_dt[3] = body_a.angular_velocity.x;
            q_dt[4] = body_a.angular_velocity.y;
            q_dt[5] = body_a.angular_velocity.z;
        }

        {
            let (_, body_b, _trans_b) = bodies.get_mut(self.handle_b.unwrap()).unwrap();

            q_dt[6] = body_b.linear_velocity.x;
            q_dt[7] = body_b.linear_velocity.y;
            q_dt[8] = body_b.linear_velocity.z;

            q_dt[9] = body_b.angular_velocity.x;
            q_dt[10] = body_b.angular_velocity.y;
            q_dt[11] = body_b.angular_velocity.z;
        }

        q_dt
    }

    fn apply_impulses(
        &self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        impulses: VecN<12>,
    ) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);
            let (_, mut body_a, trans_a) = bodies.get_mut(self.handle_a.unwrap()).unwrap();
            body_a.apply_impulse_linear(force_internal_a);
            body_a.apply_impulse_angular( torque_internal_a, &trans_a);
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            let (_, mut body_b, trans_b) = bodies.get_mut(self.handle_b.unwrap()).unwrap();
            body_b.apply_impulse_linear(force_internal_b);
            body_b.apply_impulse_angular(torque_internal_b, &trans_b);
        }
    }
}
