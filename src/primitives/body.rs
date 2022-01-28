
use bevy::prelude::*;

#[derive(Component, Clone)]
pub struct Body {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inv_mass: f32,
    pub elasticity: f32, // min = 0.0, max = 1.0
    pub friction: f32, // min = 0.0, max = 1.0

    // will be set by collider
    pub center_of_mass: Vec3,
    pub inertia_tensor: Mat3,
}


impl Default for Body {
    fn default() -> Self {
        Self {
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            inv_mass: 1.0,
            elasticity: 0.5,
            friction: 0.5,
            center_of_mass: Vec3::default(),
            inertia_tensor: Mat3::default(),
        }
    }
}

impl Body {

    #[inline]
    pub fn has_infinite_mass(&self) -> bool {
        self.inv_mass == 0.0
    }

    pub fn centre_of_mass_world(&self, t: &GlobalTransform) -> Vec3 {
        t.translation + t.rotation * self.center_of_mass
    }

    pub fn world_to_local(&self, t: &GlobalTransform, world_point: Vec3) -> Vec3 {
        let tmp = world_point - self.centre_of_mass_world(t);
        let inv_orientation = t.rotation.conjugate();
        inv_orientation * tmp
    }

    pub fn local_to_world(&self, t: &GlobalTransform, body_point: Vec3) -> Vec3 {
        self.centre_of_mass_world(t) + t.rotation * body_point
    }

    pub fn inv_inertia_tensor_local(&self) -> Mat3 {
        self.inertia_tensor.inverse() * self.inv_mass
    }

    pub fn inv_inertia_tensor_world(&self, t: &GlobalTransform) -> Mat3 {
        let inv_inertia_tensor = self.inv_inertia_tensor_local();
        let orientation = Mat3::from_quat(t.rotation);
        orientation * inv_inertia_tensor * orientation.transpose()
    }

    pub fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3, t: &GlobalTransform) {
        if self.inv_mass == 0.0 {
            return;
        }
        // impulse_point is in world space location of the applied impulse
        // impulse is in world space direction and magnitude of the impulse
        self.apply_impulse_linear(impulse);

        let position = self.centre_of_mass_world(t); // aplying impluses must produce torgues though the center of mass
        let r = impulse_point - position;
        let dl = r.cross(impulse); // this is in world space
        self.apply_impulse_angular(dl, t);
    }

    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.inv_mass == 0.0 {
            return;
        }
        // p = mv
        // dp = m dv = J
        // => dv = J / m
        self.linear_velocity += impulse * self.inv_mass;
    }
    pub fn apply_impulse_angular(&mut self, impulse: Vec3, t: &GlobalTransform) {
        if self.inv_mass == 0.0 {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        self.angular_velocity += self.inv_inertia_tensor_world(t) * impulse;

        // clamp angular_velocity - 30 rad/s is fast enough for us
        const MAX_ANGULAR_SPEED: f32 = 30.0;
        const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;
        if self.angular_velocity.length_squared() > MAX_ANGULAR_SPEED_SQ {
            self.angular_velocity = self.angular_velocity.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    pub fn update(&mut self, transform: &mut GlobalTransform, dt: f32) {
        // apply linear velocity
        transform.translation += self.linear_velocity * dt;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model

        let position_com = self.centre_of_mass_world(transform);
        let com_to_position = transform.translation - position_com;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation);
        let inertia_tensor = orientation * self.inertia_tensor * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (self
                .angular_velocity
                .cross(inertia_tensor * self.angular_velocity));
        self.angular_velocity += alpha * dt;

        // update orientation
        let d_angle = self.angular_velocity * dt;
        let angle = d_angle.length();
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();

        // now get the new body position
        transform.translation = position_com + dq * com_to_position;
    }

    /// The old way use to move the transform to find local collision point at time of impact,
    /// then role it back by running it with -time, required mutablity and required running the update logic twice
    /// This removes that and simulates moving the transform
    /// return translation and local collision point for time of impact
    // TODO: Hate coping this much logic
    pub fn local_collision_point(&self, transform: &GlobalTransform, toi: f32, world_point: Vec3) -> (Vec3, Vec3) {

         let mut tmp = transform.to_owned();

         // Start Update Simulation

         // apply linear velocity
         tmp.translation += self.linear_velocity * toi;

         // we have an angular velocity around the centre of mass, this needs to be converted to
         // relative body translation. This way we can properly update the rotation of the model

         let position_com = self.centre_of_mass_world(&tmp);
         let com_to_position = tmp.translation - position_com;
 
         // total torque is equal to external applied torques + internal torque (precession)
         // T = T_external + omega x I * omega
         // T_external = 0 because it was applied in the collision response function
         // T = Ia = w x I * w
         // a = I^-1 (w x I * w)
         let orientation = Mat3::from_quat(tmp.rotation);
         let inertia_tensor = orientation * self.inertia_tensor * orientation.transpose();
         let alpha = inertia_tensor.inverse()
             * (self
                 .angular_velocity
                 .cross(inertia_tensor * self.angular_velocity));

         let tmp_angular_velocity = self.angular_velocity + alpha * toi;
 
         // update orientation
         let d_angle = tmp_angular_velocity * toi;
         let angle = d_angle.length();
         let inv_angle = angle.recip();
         let dq = if inv_angle.is_finite() {
             Quat::from_axis_angle(d_angle * inv_angle, angle)
         } else {
             Quat::IDENTITY
         };
         tmp.rotation = (dq * tmp.rotation).normalize();
 
         // now get the new body position
         tmp.translation = position_com + dq * com_to_position;

         // Simulated update complete
         // now we can use tmp to find the local collision point
         (tmp.translation, self.world_to_local(&tmp, world_point))
    }
}
