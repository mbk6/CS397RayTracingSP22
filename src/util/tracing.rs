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
use indicatif::{ProgressBar, ProgressStyle};
use rayon::str::MatchIndices;
use std::sync::mpsc;
use std::sync::Arc;
use crossbeam::thread;
use rayon::prelude::*;
use super::geometry::*;

////////////////////////////////////////////////////////
/////   CONSTANTS, TYPEDEFS, ENUMS
////////////////////////////////////////////////////////
type Vec3 = Vector3<f32>;
type Color = Vec3;
type ImportanceSampler = fn(&RayHit) -> (Vec3, f32); // normal -> (sample direction, contribution)
const TRACE_RECURSION_DEPTH: u32 = 100;
const TRACE_SAMPLES: u32 = 1;

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
pub fn sample_hemisphere(hit: &RayHit) -> (Vec3, f32) {
    let mut rng = rand::thread_rng();
    let mut dir;
    // sample unrotated sphere
    loop {
        dir = Vec3 { x: rng.gen_range(-1.0..1.0), y: rng.gen_range(-1.0..1.0), z: rng.gen_range(-1.0..1.0) }.normalize();
        if dir.magnitude2() <= 1.0 {
            if dir.y < 0.0 { dir.y *= -1.0 };
            dir = dir.normalize();
            break;
        }
    }
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
    let mut dir;
    // sample unrotated sphere
    loop {
        dir = Vec3 { x: rng.gen_range(-1.0..1.0), y: rng.gen_range(-1.0..1.0), z: rng.gen_range(-1.0..1.0) }.normalize();
        if dir.magnitude2() <= 1.0 {
            break;
        }
    }
    (hit.hitpoint + hit.normal + dir, 1.0/(2.0*PI))
}


////////////////////////////////////////////////////////
/////   CLASSES
////////////////////////////////////////////////////////

// RAY / RAYHIT / MATERIAL
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}
#[derive(Clone, Copy)]
pub struct RayHit {
    pub distance: f32,
    pub hitpoint: Vec3,
    pub normal: Vec3,
    pub material: Material,
}
#[derive(Clone, Copy)]
pub struct Material {
    pub albedo: Vec3,
    pub emission: Vec3,
    pub ray_sampler: ImportanceSampler,
}
impl Default for Material {
    fn default() -> Material {
        Material { 
            albedo: vec3(1.0,1.0,1.0),
            emission: Vec3::zero(),
            ray_sampler: sample_hemisphere as ImportanceSampler,
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
    pub focal_length: f32,
    pub projection_mode: CameraProjectionMode,
    pub screen_width: u32,
    pub screen_height: u32,
    pub aa_sample_count: u32, // Must be a perfect square
    pub max_trace_dist: f32,
    pub gamma: f32,
}
impl Camera {
    // generate camera rays given pixel coordinates and sample count
    // currently uses multi-jittered sampling
    pub fn generate_rays(&self, screen_x: u32, screen_y: u32) -> Vec<Ray> {
        let pixel_size = 1.0 / self.screen_height as f32;
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
                (subpixel_x - 0.5*rootn)*pixel_size/rootn + (rand_x - 0.5*n)*pixel_size/n,
                (subpixel_y - 0.5*rootn)*pixel_size/rootn + (rand_y - 0.5*n)*pixel_size/n,
             );
            
            // compute pixel center and offset by jitter
            let cam_space_pixel_center = vec3(
                pixel_size*(screen_x as f32 - 0.5*(self.screen_width as f32) + 0.5) + subpixel_offset.x,
                pixel_size*(0.5 + 0.5*(self.screen_height as f32) - screen_y as f32) + subpixel_offset.y,
                -self.focal_length
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
        progress_bar.set_style(ProgressStyle::default_bar().template("[{elapsed_precise}, {eta_precise}] {wide_bar:.green/blue} {pos:>7}/{len:7}").progress_chars("##-"));
        // create image and thread channel
        let mut img = RgbImage::new(self.camera.screen_width, self.camera.screen_height);
        // iterate through pixels...
        img.as_parallel_slice_mut().into_par_iter().chunks(self.camera.screen_width as usize * 3).enumerate().for_each(|(y, mut data)| {
            for x in 0..self.camera.screen_width as usize {

                // get rays, trace, and take average of outputs for AA
                let cam_rays = self.camera.generate_rays(x as u32, y as u32);
                let mut final_color = Vec3::zero();
                for sample_idx in 0..cam_rays.len() {
                    final_color += self.shade_ray(&cam_rays[sample_idx], 0);
                    // final_color += self.phong_shade_ray(&cam_rays[sample_idx]);
                }
                final_color = final_color / cam_rays.len() as f32;
                *(data[3*x])   = (f32::powf(final_color.x.clamp(0.0,1.0), 1.0/self.camera.gamma) * 255.9999) as u8;
                *(data[3*x+1]) = (f32::powf(final_color.y.clamp(0.0,1.0), 1.0/self.camera.gamma) * 255.9999) as u8;
                *(data[3*x+2]) = (f32::powf(final_color.z.clamp(0.0,1.0), 1.0/self.camera.gamma) * 255.9999) as u8;
                progress_bar.inc(1);
            }
        });
        progress_bar.finish();
        println!("Done.");
        return img;
    }
    
    // computes same background color as raytracing in one weekend
    fn background_color(v: &Vec3) -> Color {
        let u = v.normalize();
        let t = 0.5*(u.y+1.0);
        (1.0-t)*vec3(1.0, 1.0, 1.0) + t*vec3(0.5, 0.7, 1.0)
    }
    
    // computes phong shading for a given rayhit
    fn phong_shade_ray(&self, ray: &Ray) -> Color {
        // get hit
        match self.intersect_ray(ray, 0.0, self.camera.max_trace_dist) {
            None => Scene::background_color(&ray.direction),
            Some(hit) => {
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
                
                shadow_weight * (self.ambient + diffuse_weight*hit.material.albedo + specular_weight*vec3(0.4, 0.4, 0.4))
            }
        }
    }
    
    // computes shading for a ray hit according to the monte-carlo integrated rendering equation
    fn shade_ray(&self, ray: &Ray, depth: u32) -> Color {
        if depth >= TRACE_RECURSION_DEPTH { 
            return 0.3*Scene::background_color(&ray.direction); // ambient light model 
        }
        // get hit
        match self.intersect_ray(ray, 0.001, self.camera.max_trace_dist.clone()) {
            None => Scene::background_color(&ray.direction),
            Some(hit) => {
                // accumulate integral
                let mut integral = Color::zero();
                for _i in 0..TRACE_SAMPLES {
                    // pick new direction, generate ray, and recurse
                    let (new_dir, pdf) = (hit.material.ray_sampler)(&hit);
                    let new_ray = Ray { origin: hit.hitpoint, direction: new_dir };
                    
                    let brdf_term = hit.material.albedo / PI; // for now just using the lambersion brdf
                    let dot_term = new_dir.dot(hit.normal).clamp(0.0,1.0);
                    let incoming_light = self.shade_ray(&new_ray, depth+1);
                    // accumulate into integral
                    integral += (dot_term*(brdf_term.mul_element_wise(incoming_light))) / pdf;
                }
                integral /= TRACE_SAMPLES as f32; 
        
                // total light = integrated + emitted light
                hit.material.emission + integral
            }
        }        
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




// runs ray tracer
pub fn run() {
    // initialize scene
    let scene = Scene {
        camera: Camera {
            eyepoint: vec3(0.0, 2.0, 5.0),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            focal_length: 0.6,
            projection_mode: CameraProjectionMode::Perspective,
            screen_width: 400,
            screen_height: 400,
            aa_sample_count: 100,
            max_trace_dist: 100000.0,
            gamma: 1.0,
        },
        objects: Arc::new(vec![

            Arc::new(StaticMesh::load_from_file("./obj/teapot.obj")),
            Arc::new(Plane {
                point: vec3(0.0, 0.0, 0.0),
                normal: Vec3::unit_y(),
                material: Material { albedo: vec3(0.4,0.4,0.4), ray_sampler: alpha_sample as ImportanceSampler, ..Default::default() },
            }),
            // Arc::new(Plane {
            //     point: vec3(0.0, 0.0, -4.0),
            //     normal: Vec3::unit_z(),
            //     material: Material { albedo: vec3(0.6,0.6,0.6), ray_sampler: sample_hemisphere as ImportanceSampler, ..Default::default() },
            // }),
            // Arc::new(Plane {
            //     point: vec3(0.0, 5.0, 0.0),
            //     normal: -Vec3::unit_y(),
            //     material: Material { albedo: vec3(0.6,0.6,0.6), ray_sampler: sample_hemisphere as ImportanceSampler, ..Default::default() },
            // }),
            // Arc::new(Plane {
            //     point: vec3(-5.0, 0.0, 0.0),
            //     normal: Vec3::unit_x(),
            //     material: Material { albedo: vec3(0.6,0.0,0.0), ray_sampler: sample_hemisphere as ImportanceSampler, ..Default::default() },
            // }),
            // Arc::new(Plane {
            //     point: vec3(5.0, 0.0, 0.0),
            //     normal: -Vec3::unit_x(),
            //     material: Material { albedo: vec3(0.0,0.6,0.0), ray_sampler: sample_hemisphere as ImportanceSampler, ..Default::default() },
            // }),
            // Arc::new(Sphere {
            //     center: vec3(0.0,6.0,-1.0),
            //     radius: 1.6,
            //     material: Material { albedo: vec3(0.6,0.3,0.3), emission: vec3(10.0,10.0,10.0), ray_sampler: sample_hemisphere as ImportanceSampler },
            // }),

            
            Arc::new(Sphere {
                center: vec3(-0.8,0.5,2.0),
                radius: 0.4,
                material: Material { albedo: vec3(0.3,0.3,0.3), emission: vec3(5.0,0.0,5.0), ray_sampler: alpha_sample as ImportanceSampler },
            }),
        ]),
        point_light_pos: vec3(0.0,1.0,5.0), // for phong shading only
        ambient: vec3(0.1,0.1,0.1), // for phong shading only
    };

    // render and write output
    scene.render_to_image().save_with_format("render.png", ImageFormat::Png).unwrap();

}