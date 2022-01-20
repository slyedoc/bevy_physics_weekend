mod convex;

pub use convex::Convex;

use crate::bounds::Bounds;
use bevy::{
    math::{Mat3, Vec3},
    prelude::{shape, Transform},
};

#[derive(Clone, Debug)]
pub struct Collider {
    pub center_of_mass: Vec3,
    pub inertia_tensor: Mat3,
    pub bounds: Bounds,
    pub shape: ShapeType,
    pub points: Vec<Vec3>,
}

impl Default for Collider {
    fn default() -> Self {
        Sphere::default().into()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ShapeType {
    Sphere{ radius: f32 },
    Box,
    Convex,
}

impl Collider {
    pub fn bounds(&self, transform: &Transform) -> Bounds {
        match self.shape {
            ShapeType::Sphere { radius } => Bounds {
                mins: -radius + transform.translation,
                maxs: self.bounds.maxs + transform.translation,
            },
            ShapeType::Box => Bounds {
                mins: self.bounds.mins + transform.translation,
                maxs: self.bounds.maxs + transform.translation,
            },
            ShapeType::Convex => todo!(),
        }
    }

    // find the point in the furthest in direction
    pub fn support(&self, dir: Vec3, transform: &Transform, bias: f32) -> Vec3 {
        match self.shape {
            ShapeType::Sphere { radius} => transform.translation + dir * (radius + bias),
            ShapeType::Box => {

                let mut max_pt = (transform.rotation * self.points[0]) + transform.translation;
                let mut max_dist = dir.dot(max_pt);
                for pt in &self.points[1..] {
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
    pub fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        match self.shape {
            ShapeType::Sphere { .. } => 0.0,
            ShapeType::Box => {
                let mut max_speed = 0.0;
                for pt in &self.points {
                    let r = *pt - self.center_of_mass;
                    let linear_velocity = angular_velocity.cross(r);
                    let speed = dir.dot(linear_velocity);
                    if speed > max_speed {
                        max_speed = speed;
                    }
                }
                max_speed
            },
            ShapeType::Convex => todo!(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Sphere {
    pub radius: f32,
}

impl Default for Sphere {
    fn default() -> Self {
        Self { radius: 1.0 }
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
            points: vec![Vec3::ZERO],
            shape: ShapeType::Sphere { radius: sphere.radius } ,
        }
    }
}

impl From<shape::Box> for Collider {
    fn from(value: shape::Box) -> Self {
        let points = vec![
            Vec3::new(value.min_x, value.max_y, value.min_z),
            Vec3::new(value.max_x, value.max_y, value.min_z),
            Vec3::new(value.min_x, value.max_y, value.max_z),
            Vec3::new(value.max_x, value.max_y, value.max_z),
            Vec3::new(value.min_x, value.min_y, value.min_z),
            Vec3::new(value.max_x, value.min_y, value.min_z),
            Vec3::new(value.min_x, value.min_y, value.max_z),
            Vec3::new(value.max_x, value.min_y, value.max_z),
        ] ;
        let bounds = Bounds {
            mins: Vec3::new(value.min_x, value.min_y, value.min_z),
            maxs: Vec3::new(value.max_x, value.max_y, value.max_z),
        };
        // inertia tensor for box centered around zero
        let d = bounds.maxs - bounds.mins;

        let dd = d * d;
        let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
        let tensor = Mat3::from_diagonal(diagonal);

        // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
        // not centered around the origin

        let cm = (bounds.maxs + bounds.mins) * 0.5;

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
            bounds,
            points,
            shape: ShapeType::Box,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub a: u32,
    pub b: u32,
    pub c: u32,
}


#[derive(Copy, Clone, Debug)]
pub struct Edge {
    pub a: u32,
    pub b: u32,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for Edge {}

// This will compare the incoming edge with all the edges in the facing tris and then return true
// if it's unique.
fn is_edge_unique(tris: &[Tri], facing_tris: &[u32], ignore_tri: u32, edge: &Edge) -> bool {
    for &tri_idx in facing_tris {
        if ignore_tri == tri_idx {
            continue;
        }

        let tri = tris[tri_idx as usize];

        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        for e in &edges {
            if *edge == *e {
                return false;
            }
        }
    }

    true
}