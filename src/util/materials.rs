// MATERIALS - Implements various materials
use cgmath::*;
use rayon::str::MatchIndices;
use std::f32::consts::PI;
use rand::Rng;

use super::tracing::*;

// Abstract material definition
pub trait Material {
    fn scatter(&self, hit: &RayHit, ray: &Ray) -> (Ray, Color, f32); // returns a new ray, its attenuation, and the probabiltiy it was chosen for a given material
    fn emission(&self) -> Color;
}


// LAMBERTIAN
#[derive(Clone, Copy)]
pub struct Lambertian {
    pub albedo: Vec3,
    pub emission: Vec3,
    // pub ray_sampler: ImportanceSampler,
}
impl Default for Lambertian {
    fn default() -> Lambertian {
        Lambertian { 
            albedo: vec3(1.0,1.0,1.0),
            emission: Vec3::zero(),
            // ray_sampler: sample_hemisphere as ImportanceSampler,
        }
    }

}
impl Material for Lambertian {
    fn scatter(&self, hit: &RayHit, ray: &Ray) -> (Ray, Color, f32) {
        let (dir, pdf) = sample_hemisphere(hit);
        (
            Ray {
                origin: hit.hitpoint,
                direction: dir,
            },
            self.albedo / PI,
            pdf,
        )
    }
    fn emission(&self) -> Color {
        self.emission
    }
}

// METAL
pub struct Metal {
    pub albedo: Color,
    pub glossiness: f32,
}
impl Material for Metal {
    fn scatter(&self, hit: &RayHit, ray: &Ray) -> (Ray, Color, f32) {
        (
            Ray {
                origin: hit.hitpoint,
                direction: reflect(&ray.direction, &hit.normal) + self.glossiness*rand_sphere_vec(),
            },
            self.albedo,
            1.0
        )
    }
    fn emission(&self) -> Color {
        Vec3::zero()
    }
}



// SAMPLING FUNCTIONS
// uniformly samples a hemisphere given by normal n
pub fn sample_hemisphere(hit: &RayHit) -> (Vec3, f32) {
    // get random vector in hemisphere
    let mut dir = rand_sphere_vec();
    dir.y = dir.y.abs();
    // rotate relative to given normal
    let rotation = cgmath::Basis3::between_vectors(Vec3::unit_y(), hit.normal);
    (rotation.rotate_vector(dir), 1.0/(2.0*PI))
}

// based on http://three-eyed-games.com/2018/05/12/gpu-path-tracing-in-unity-part-2/
pub fn alpha_sample(hit: &RayHit) -> (Vec3, f32) {
    let alpha = 1.0;
    let mut rng = rand::thread_rng();
    // pick random point on sphere sitting on xz plane
    let cos_theta = f32::powf(rng.gen_range(0.0..1.0), 1.0/(alpha+1.0));
    let sin_theta = f32::sqrt(f32::max(0.0, 1.0 - cos_theta*cos_theta));
    let phi = 2.0*PI*rng.gen_range(0.0..1.0);
    let vec = vec3(f32::cos(phi)*sin_theta, f32::sin(phi)*sin_theta, cos_theta);
    
    // rotate relative to given normal
    let rotation = cgmath::Basis3::between_vectors(Vec3::unit_z(), hit.normal);
    (rotation.rotate_vector(vec), (alpha+1.0)*f32::powf(cos_theta, alpha) / (2.0*PI))
}

// based on raytracing in one weekend
pub fn rtow_sample(hit: &RayHit) -> (Vec3, f32) {
    let mut rng = rand::thread_rng();
    let dir = rand_sphere_vec();
    (hit.hitpoint + hit.normal + dir, 1.0/(2.0*PI))
}
