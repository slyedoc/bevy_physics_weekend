mod convex;

pub use convex::Convex;

use bevy::{
    math::{Mat3, Vec3},
    prelude::Transform,
};

use crate::bounds::Bounds;

#[derive(Clone, Debug)]
pub struct Collider {
    pub center_of_mass: Vec3,
    pub inertia_tensor: Mat3,
    pub bounds: Bounds,
    pub shape: ShapeType,
}

impl Default for Collider {
    fn default() -> Self {
        Sphere::default().into()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ShapeType {
    Sphere,
    Box,
    Convex,
}

impl Collider {
    pub fn bounds(&self, transform: &Transform) -> Bounds {
        match self.shape {
            ShapeType::Sphere => {
                Bounds {
                    mins: self.bounds.mins + transform.translation,
                    maxs: self.bounds.maxs + transform.translation,
                }
            },
            ShapeType::Box => todo!(),
            ShapeType::Convex => todo!(),
        }
    }

    // find the point in the furthest in direction
    pub fn support(&self, dir: Vec3, transform: &Transform, bias: f32) -> Vec3 {
        match self.shape {
            ShapeType::Sphere => transform.translation + dir * (self.bounds.maxs.x + bias),
            ShapeType::Box => {

                // TODO: shouldn't generate this each time, should be simpler way to find furthest point
                let points = [
                    Vec3::new(self.bounds.mins.x, self.bounds.mins.y, self.bounds.mins.z),
                    Vec3::new(self.bounds.maxs.x, self.bounds.mins.y, self.bounds.mins.z),
                    Vec3::new(self.bounds.mins.x, self.bounds.maxs.y, self.bounds.mins.z),
                    Vec3::new(self.bounds.mins.x, self.bounds.mins.y, self.bounds.maxs.z),
                    Vec3::new(self.bounds.maxs.x, self.bounds.maxs.y, self.bounds.maxs.z),
                    Vec3::new(self.bounds.mins.x, self.bounds.maxs.y, self.bounds.maxs.z),
                    Vec3::new(self.bounds.maxs.x, self.bounds.mins.y, self.bounds.maxs.z),
                    Vec3::new(self.bounds.maxs.x, self.bounds.maxs.y, self.bounds.mins.z),
                ];

                let mut max_pt = (transform.rotation * points[0]) + transform.translation;
                let mut max_dist = dir.dot(max_pt);
                for pt in &points[1..] {
                    let pt = (transform.rotation * *pt) + transform.translation;
                    let dist = dir.dot(pt);
                    if dist > max_dist {
                        max_dist = dist;
                        max_pt = pt;
                    }
                }

                let norm = dir.normalize() * bias;
                
                max_pt + norm
            }
            ShapeType::Convex => todo!(),
        }
    }
    pub fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
         unimplemented!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Sphere {
    pub radius: f32,
}

impl Default for Sphere {
    fn default() -> Self {
        Self {
            radius: 1.0,
        }
    }
}

impl From<Sphere> for Collider {
    fn from(sphere: Sphere) -> Self {
        let i = 2.0 * sphere.radius * sphere.radius / 5.0;
        let tensor = Mat3::from_diagonal(Vec3::splat(i));

        Collider {
            center_of_mass: Vec3::ZERO,
            inertia_tensor: tensor,
            bounds: Bounds {
                mins: Vec3::splat(-sphere.radius),
                maxs: Vec3::splat(sphere.radius),
            },
            shape: ShapeType::Sphere,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Box {
    bounds: Bounds
}

impl From<Box> for Collider {
    fn from(value: Box) -> Self {
        // inertia tensor for box centered around zero
        let d = value.bounds.maxs - value.bounds.mins;
        let dd = d * d;
        let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
        let tensor = Mat3::from_diagonal(diagonal);

        // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
        // not centered around the origin

        let cm = (value.bounds.maxs + value.bounds.mins) * 0.5;

        // the displacement from the center of mass to the origin
        let r = -cm;
        let r2 = r.length_squared();

        let pat_tensor = Mat3::from_cols(
            Vec3::new(r2 - r.x * r.x, r.x * r.y, r.x * r.z),
            Vec3::new(r.y * r.x, r2 - r.y * r.y, r.y * r.z),
            Vec3::new(r.z * r.x, r.z * r.y, r2 - r.z * r.z),
        );

        // now we need to add the centre of mass tensor and the parallel axis theorem tensor
        // together

        Collider {
            center_of_mass: cm,
            inertia_tensor: tensor + pat_tensor,
            bounds: value.bounds,
            shape: ShapeType::Box,
        }
    }
}

