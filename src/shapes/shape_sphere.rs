use super::{ShapeTrait, Shape};
use crate::bounds::Bounds;
use bevy::{math::{Mat3, Quat, Vec3}, prelude::Transform};

#[derive(Copy, Clone, Debug)]
pub struct ShapeSphere {
    pub radius: f32,
}

impl ShapeTrait for ShapeSphere {
    fn centre_of_mass(&self) -> Vec3 {
        Vec3::ZERO
    }

    fn inertia_tensor(&self) -> Mat3 {
        let i = 2.0 * self.radius * self.radius / 5.0;
        Mat3::from_diagonal(Vec3::splat(i))
    }

    fn local_bounds(&self) -> Bounds {
        Bounds {
            mins: Vec3::splat(-self.radius),
            maxs: Vec3::splat(self.radius),
        }
    }

    fn bounds(&self, transform: &Transform) -> Bounds {
        Bounds {
            mins: Vec3::splat(-self.radius) + transform.translation,
            maxs: Vec3::splat(self.radius) + transform.translation,
        }
    }

    fn support(&self, dir: Vec3, transform: &Transform, bias: f32) -> Vec3 {
        transform.translation + dir * (self.radius + bias)
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        unimplemented!()
    }
}
