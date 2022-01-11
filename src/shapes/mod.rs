mod shape_box;
mod shape_convex;
mod shape_sphere;

pub use shape_box::ShapeBox;
pub use shape_convex::ShapeConvex;
pub use shape_sphere::ShapeSphere;

use bevy::{math::{Vec3, Mat3}, prelude::Transform};

use crate::bounds::Bounds;

pub trait ShapeTrait {
    fn centre_of_mass(&self) -> Vec3;
    fn inertia_tensor(&self) -> Mat3;
    fn local_bounds(&self) -> Bounds;
    fn bounds(&self, transform: &Transform) -> Bounds;
    fn support(&self, dir: Vec3, transform: &Transform, bias: f32) -> Vec3;
    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32;
}

#[derive( Clone, Debug)]
pub enum Shape {
    Sphere(ShapeSphere),
    Box(ShapeBox),
    Convex(ShapeConvex),
}

impl Default for Shape {
    fn default() -> Shape {
        Shape::Sphere(ShapeSphere { radius: 1.0 })
    }
}

impl Shape {
    pub fn make_sphere(radius: f32) -> Self {
        Shape::Sphere(ShapeSphere { radius })
    }

    pub fn make_box(data: ShapeBox) -> Self {
        Shape::Box(data)
    }

    fn shape_trait(&self) -> &dyn ShapeTrait {
        // TODO: check the overhead of this
        match self {
            Shape::Sphere(data) => data,
            Shape::Box(data) => data,
            Shape::Convex(data) => data,
        }
    }

    pub fn centre_of_mass(&self) -> Vec3 {
        self.shape_trait().centre_of_mass()
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.shape_trait().inertia_tensor()
    }

    pub fn bounds(&self, transform: &Transform) -> Bounds {
        self.shape_trait().bounds(transform)
    }
}
