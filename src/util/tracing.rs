// TRACING - Implements a scene, camera, ray, and other tracing utilities

#![allow(dead_code)]


use std::f32::consts::PI;
////////////////////////////////////////////////////////
/////   INCLUDES
////////////////////////////////////////////////////////
use std::{ops::Neg, iter::Scan};
use image::*;
use cgmath::*;
use rand::Rng;
use indicatif::ProgressBar;
use rayon::str::MatchIndices;
use std::sync::mpsc;
use std::sync::Arc;
use crossbeam::thread;
use rayon::prelude::*;
use super::mesh::StaticMesh;
use super::mesh::{AABB};

////////////////////////////////////////////////////////
/////   CONSTANTS, TYPEDEFS, ENUMS
////////////////////////////////////////////////////////
type Vec3 = Vector3<f32>;
type Color = Vec3;
const TRACE_RECURSION_DEPTH: u32 = 3;
const TRACE_SAMPLES: u32 = 50;

#[derive(Debug, Clone, Copy)]
pub enum CameraProjectionMode {
    Orthographic,
    Perspective,
}

////////////////////////////////////////////////////////
/////   TRAITS
////////////////////////////////////////////////////////
pub trait Intersectable {
    // tests for intersection with a given ray and returns hit info
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit>;
    // returns the axis-aligned bounding box of the intersectable, if there is one
    fn bounding_box(&self) -> Option<AABB>; // Option because not all primitives have bounding boxes (e.g. plane)
}


////////////////////////////////////////////////////////
/////   UTILITY FUNCTIONS
////////////////////////////////////////////////////////

// uniformly samples a hemisphere given by normal n
fn sample_hemisphere(n: Vec3) -> Vec3 {
    let mut rand;
    // sample unrotated sphere
    loop {
        rand = Vec3 { x: rand::thread_rng().gen_range(-1.0..1.0), y: rand::thread_rng().gen_range(-1.0..1.0), z: rand::thread_rng().gen_range(-1.0..1.0) }.normalize();
        if rand.magnitude2() <= 1.0 {
            if rand.y < 0.0 { rand.y *= -1.0 };
            rand = rand.normalize();
            break;
        }
    }
    // rotate relative to given normal
    let rotation = cgmath::Basis3::between_vectors(Vec3::unit_y(), n);
    rotation.rotate_vector(rand)
}


////////////////////////////////////////////////////////
/////   CLASSES
////////////////////////////////////////////////////////

// RAY / RAYHIT / MATERIAL
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    pub distance: f32,
    pub hitpoint: Vec3,
    pub normal: Vec3,
    pub material: Material,
}
#[derive(Debug, Clone, Copy)]
pub struct Material {
    pub albedo: Vec3,
    pub emission: Vec3,
}
impl Default for Material {
    fn default() -> Material {
        Material { 
            albedo: vec3(1.0,1.0,1.0),
            emission: Vec3::zero(),
        }
    }
}

// CAMERA
#[derive(Debug, Clone)]
pub struct Camera {
    // camera model based on 419 lectures
    pub eyepoint: Vec3,
    pub view_dir: Vec3,
    pub up: Vec3,
    pub view_plane_dist: f32,
    pub pixel_size: f32,
    pub projection_mode: CameraProjectionMode,
    pub screen_width: u32,
    pub screen_height: u32,
    pub aa_sample_count: u32, // Must be a perfect square
    pub max_trace_dist: f32,
}
impl Camera {
    // generate camera rays given pixel coordinates and sample count
    // currently uses multi-jittered sampling
    pub fn generate_rays(&self, screen_x: u32, screen_y: u32) -> Vec<Ray> {
        let mut rays = Vec::new();
        let n = self.aa_sample_count as f32;
        let rootn = n.sqrt();
        let mut rng = rand::thread_rng();
        for i in 0..self.aa_sample_count {
            // compute multi-jittered pixel offset
            let rand_x = rng.gen_range(0..self.aa_sample_count) as f32;
            let rand_y = rng.gen_range(0..self.aa_sample_count) as f32;
            let subpixel_x = (i / rootn as u32) as f32;
            let subpixel_y = (i % rootn as u32) as f32;
            let subpixel_offset = vec2(
                (subpixel_x - 0.5*rootn)*self.pixel_size/rootn + (rand_x - 0.5*n)*self.pixel_size/n,
                (subpixel_y - 0.5*rootn)*self.pixel_size/rootn + (rand_y - 0.5*n)*self.pixel_size/n,
             );
            
            // compute pixel center and offset by jitter
            let cam_space_pixel_center = vec3(
                self.pixel_size*(screen_x as f32 - 0.5*(self.screen_width as f32) + 0.5) + subpixel_offset.x,
                self.pixel_size*(0.5 + 0.5*(self.screen_height as f32) - screen_y as f32) + subpixel_offset.y,
                -self.view_plane_dist
            ).normalize();

            // create ray with direction still in camera space
            let mut ray = Ray {
                origin: match self.projection_mode {
                    CameraProjectionMode::Orthographic => vec3(cam_space_pixel_center.x, cam_space_pixel_center.y, 0.0 ),
                    CameraProjectionMode::Perspective => self.eyepoint,
                },
                direction: match self.projection_mode {
                    CameraProjectionMode::Orthographic => self.view_dir,
                    CameraProjectionMode::Perspective => cam_space_pixel_center
                },
            };

            // rotate ray direction to world space
            let rotation = Matrix3::from_cols(
                self.view_dir.cross(self.up).normalize(),
                self.up,
                -self.view_dir
            );
            ray.direction = rotation * ray.direction;
            rays.push(ray);
        }
        return rays;
    }
}

// SCENE
pub struct Scene {
    pub camera: Camera,
    pub objects: Arc<Vec<Arc<dyn Intersectable + Send + Sync>>>,
    pub point_light_pos: Vec3,
    pub ambient: Vec3,
}
impl Scene {
    pub fn render_to_image(&self) -> RgbImage {
        println!("Rendering...");
        let progress_bar = ProgressBar::new((self.camera.screen_width*self.camera.screen_height) as u64);
        // create image and thread channel
        let mut img = RgbImage::new(self.camera.screen_width, self.camera.screen_height);
        // iterate through pixels...
        img.as_parallel_slice_mut().into_par_iter().chunks(self.camera.screen_width as usize * 3).enumerate().for_each(|(y, mut data)| {
            for x in 0..self.camera.screen_width as usize {

                // get rays, trace, and take average of outputs for AA
                let cam_rays = self.camera.generate_rays(x as u32, y as u32);
                let mut final_color = Vec3::zero();
                for sample_idx in 0..cam_rays.len() {
                    final_color += match self.intersect_ray(&cam_rays[sample_idx], 0.0, self.camera.max_trace_dist) {
                        None => Vec3::zero(),
                        Some(hit) => {
                            // run shading algorithm
                            self.shade_hit(&hit, 0)
                            // self.phong_shade_hit(&hit)
                        }
                    };
                }
                final_color = final_color / cam_rays.len() as f32;
                *(data[3*x])   = (final_color.x.clamp(0.0,1.0) * 255.9999) as u8;
                *(data[3*x+1]) = (final_color.y.clamp(0.0,1.0) * 255.9999) as u8;
                *(data[3*x+2]) = (final_color.z.clamp(0.0,1.0) * 255.9999) as u8;
                progress_bar.inc(1);
            }
        });
        progress_bar.finish();
        println!("Done.");
        return img;
    }
    // computes phong shading for a given rayhit
    fn phong_shade_hit(&self, hit: &RayHit) -> Color {
        // standard phong shading
        let to_light = (self.point_light_pos - hit.hitpoint).normalize();
        let to_camera = (self.camera.eyepoint - hit.hitpoint).normalize();
        let reflected = -to_light + 2.0*dot(to_light, hit.normal)*hit.normal;
        let diffuse_weight = (dot(hit.normal, to_light)).clamp(0.0, 1.0);
        let specular_weight = dot(to_camera, reflected).clamp(0.0, 1.0).powf(40.0);
        // cast shadow ray
        let shadow_ray = Ray { origin: hit.hitpoint + 0.01*hit.normal, direction: to_light };
        let shadow_weight = match self.intersect_ray(&shadow_ray, 0.0, (self.point_light_pos - hit.hitpoint).magnitude()) {
            None => 1.0,
            Some(hit) => if hit.distance*hit.distance > (self.point_light_pos - hit.hitpoint).magnitude2() { 1.0 } else { 0.3 }
        };
        return shadow_weight * (self.ambient + diffuse_weight*hit.material.albedo + specular_weight*vec3(0.4, 0.4, 0.4));
    }
    // computes shading for a ray hit according to the monte-carlo integrated rendering equation
    fn shade_hit(&self, hit: &RayHit, depth: u32) -> Color {
        if depth >= TRACE_RECURSION_DEPTH { 
            return vec3(0.4,0.4,0.4); 
        }
        // accumulate integral
        let mut integral = Color::zero();
        for i in 0..TRACE_SAMPLES {
            // pick new direction, generate ray, and recurse
            let new_dir = sample_hemisphere(hit.normal);
            let new_ray = Ray { origin: hit.hitpoint + 0.001*hit.normal, direction: new_dir };
            let brdf_term = hit.material.albedo / PI; // for now just using the lambersion brdf
            let dot_term = new_dir.dot(hit.normal).clamp(0.0,1.0);
            let incoming_light = match self.intersect_ray(&new_ray, 0.0, self.camera.max_trace_dist) {
                Some(hit) => self.shade_hit(&hit, depth+1),
                None => Vec3::zero() // background is black for now
            };
            // accumulate into integral
            integral += dot_term*(brdf_term.mul_element_wise(incoming_light));
        }
        integral *= 2.0*PI / TRACE_SAMPLES as f32; 

        // total light = integrated + emitted light
        hit.material.emission + integral
    }

}
impl Intersectable for Scene {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // for now, just iterate over all intersectables and return shortest (this will probably be a BVH or something later)
        let mut best_hit = None;
        for object in self.objects.iter() {
            if let Some(hit) = object.intersect_ray(ray, t_min, t_max) {
                best_hit = match best_hit {
                    None => Some(hit),
                    Some(current_best) => {
                        if hit.distance < current_best.distance {
                            Some(hit)
                        }
                        else {
                            Some(current_best)
                        }
                    }
                }
            }
        }
        return best_hit;
    }
    fn bounding_box(&self) -> Option<AABB> {
        None
    }
}

// SPHERE
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
    pub material: Material,
}
impl Intersectable for Sphere {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-sphere intersection
        let f = ray.origin - self.center;
        let a = ray.direction.magnitude2();
        let b = 2.0*f.dot(ray.direction);
        let c = f.magnitude2() - self.radius*self.radius;
        let d = b*b - 4.0*a*c;
        if d < 0.0 {
            return None;
        }
        else {
            let t = (-b - d.sqrt()) / (2.0*a);
            let hitpoint = ray.origin + t*ray.direction;
            if t < t_min || t > t_max { return None }
            return Some(RayHit {
                distance: t,
                hitpoint: hitpoint,
                normal: (hitpoint - self.center).normalize(),
                material: self.material,
            })
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(AABB {
            min: self.center - vec3(self.radius,self.radius,self.radius),
            max: self.center + vec3(self.radius,self.radius,self.radius),
        })
    }
}

// TRIANGLE
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub material: Material,
}
impl Intersectable for Triangle {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-triangle intersection
        const EPSILON : f32 = 0.0001;
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let q = ray.direction.cross(e2);
        let a = e1.dot(q);
        if a.abs() < EPSILON { return None; }
        let f = 1.0/a;
        let s = ray.origin - self.a;
        let u = f*s.dot(q);
        if u < 0.0 { return None; }
        let r = s.cross(e1);
        let v = f*ray.direction.dot(r);
        if v < 0.0 || u+v > 1.0 { return None }
        let t = f*e2.dot(r);
        let hitpoint = ray.origin + t*ray.direction;
        if t < t_min || t > t_max { return None }
        return Some(RayHit {
            distance: t,
            hitpoint: hitpoint,
            normal: e1.cross(e2).normalize(),
            material: self.material,
        })
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(AABB {
            min: vec3(
                f32::min(self.a.x,f32::min(self.b.x, self.c.x)),
                f32::min(self.a.y,f32::min(self.b.y, self.c.y)),
                f32::min(self.a.z,f32::min(self.b.z, self.c.z))
            ),
            max: vec3(
                f32::max(self.a.x,f32::max(self.b.x, self.c.x)),
                f32::max(self.a.y,f32::max(self.b.y, self.c.y)),
                f32::max(self.a.z,f32::max(self.b.z, self.c.z))
            ),
        })
    }
}

// PLANE
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
    pub material: Material,
}
impl Intersectable for Plane {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-plane intersection
        let to_ray_origin = ray.origin - self.point;
        let origin_dist = dot(to_ray_origin, self.normal);
        let n = origin_dist.signum() * self.normal;
        let d = ray.direction.dot(n);
        if d >= 0.0 { 
            return None;
        }
        else {
            let t = origin_dist.abs() / d.abs();
            if t < t_min || t > t_max { return None }
            let hitpoint = ray.origin + t*ray.direction;
            return Some(RayHit {
                distance: t,
                hitpoint: hitpoint,
                normal: n,
                material: self.material,
            })
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        None
    }
}










// runs ray tracer
pub fn run() {
    // initialize scene
    let scene = Scene {
        camera: Camera {
            eyepoint: vec3(0.0, 2.0, 4.0),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            view_plane_dist: 0.1,
            pixel_size: 0.001,
            projection_mode: CameraProjectionMode::Perspective,
            screen_width: 200,
            screen_height: 200,
            aa_sample_count: 9,
            max_trace_dist: 100000.0,
        },
        objects: Arc::new(vec![

            //Arc::new(StaticMesh::load_from_file("./obj/teapot.obj")),
            Arc::new(Plane {
                point: vec3(0.0, -2.0, 0.0),
                normal: Vec3::unit_y(),
                material: Material { albedo: vec3(0.6,0.6,0.6), ..Default::default() },
            }),
            Arc::new(Plane {
                point: vec3(0.0, 0.0, -4.0),
                normal: Vec3::unit_z(),
                material: Material { albedo: vec3(0.6,0.6,0.6), ..Default::default() },
            }),
            Arc::new(Plane {
                point: vec3(0.0, 5.0, 0.0),
                normal: -Vec3::unit_y(),
                material: Material { albedo: vec3(0.6,0.6,0.6), ..Default::default() },
            }),
            Arc::new(Plane {
                point: vec3(-5.0, 0.0, 0.0),
                normal: Vec3::unit_x(),
                material: Material { albedo: vec3(0.6,0.0,0.0), ..Default::default() },
            }),
            Arc::new(Plane {
                point: vec3(5.0, 0.0, 0.0),
                normal: -Vec3::unit_x(),
                material: Material { albedo: vec3(0.0,0.6,0.0), ..Default::default() },
            }),
            Arc::new(Sphere {
                center: vec3(0.0,6.0,-1.0),
                radius: 1.6,
                material: Material { albedo: vec3(0.6,0.3,0.3), emission: vec3(10.0,10.0,10.0) },
            }),
            // Arc::new(Triangle {
            //     a: vec3(0.0, 6.0, -6.0),
            //     b: vec3(-4.0, 6.0, -2.0),
            //     c: vec3(4.0, 6.0, -2.0),
            //     material: Material { albedo: vec3(1.0,1.0,1.0), emission: vec3(7.0,7.0,7.0) },
            // }),
        ]),
        point_light_pos: vec3(0.0,1.0,5.0), // for phong shading only
        ambient: vec3(0.1,0.1,0.1), // for phong shading only
    };

    // render and write output
    scene.render_to_image().save_with_format("mp1.png", ImageFormat::Png);

}