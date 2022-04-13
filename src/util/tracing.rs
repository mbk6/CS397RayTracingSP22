// TRACING - Implements a scene, camera, ray, and other tracing utilities

#![allow(dead_code)]



use std::ops::Neg;
////////////////////////////////////////////////////////
/////   INCLUDES
////////////////////////////////////////////////////////
use image::*;
use cgmath::*;
use rand::Rng;
use indicatif::{ProgressBar, ProgressStyle};
use std::sync::Arc;
use rayon::prelude::*;

use super::geometry::*;
use super::materials::*;

////////////////////////////////////////////////////////
/////   CONSTANTS, TYPEDEFS, ENUMS
////////////////////////////////////////////////////////
pub type Vec3 = Vector3<f32>;
pub type Vec2 = Vector2<f32>;
pub type Color = Vec3;
type ImportanceSampler = fn(&RayHit) -> (Vec3, f32); // normal -> (sample direction, contribution)

#[derive(Debug, Clone, Copy)]
pub enum CameraProjectionMode {
    Orthographic,
    Perspective,
}
#[derive(Debug, Clone, Copy)]
pub enum ShadingMode {
    Phong,
    PathTrace,
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
pub fn reflect(v: &Vec3, n: &Vec3) -> Vec3 {
    v - 2.0*v.dot(*n)*n
}
// Approximates the fresnel reflection-transmission coefficient using Schlick's approximation (https://en.wikipedia.org/wiki/Schlick%27s_approximation)
pub fn fresnel(v: &Vec3, n: &Vec3, ir: f32) -> f32 {
    // (first index of refraction is assumed to be air (1.0). the equation is symmetric so it doesn't matter which medium is first)
    let r0 = ((ir-1.0)/(ir+1.0)).powi(2);
    r0 + (1.0-r0)*(1.0-v.dot(*n).abs()).powi(5)
}
// GLSL refract function
// pub fn refract(v: &Vec3, n: &Vec3, eta: f32) -> Vec3 {
//     let k = 1.0 - eta * eta * (1.0 - v.dot(*n)*v.dot(*n));
//     if k < 0.0 { Vec3::zero() } else { eta*v - (eta*v.dot(*n) + f32::sqrt(k))*n }
// }
// Raytracing in one weekend refract function:
pub fn refract(v: &Vec3, n: &Vec3, eta: f32) -> Vec3 {
    let cos_theta = f32::min((v.neg()).dot(*n), 1.0);
    let r_out_perp =  eta * (v + cos_theta*n);
    let r_out_parallel = -f32::sqrt((1.0 - r_out_perp.magnitude2()).abs()) * n;
    return r_out_perp + r_out_parallel;
}
// random vector in a unit sphere (rejection method)
pub fn rand_sphere_vec() -> Vec3 {
    let mut rng = rand::thread_rng();
    loop {
        let dir = Vec3 { x: rng.gen_range(-1.0..1.0), y: rng.gen_range(-1.0..1.0), z: rng.gen_range(-1.0..1.0) };
        if dir.magnitude2() <= 1.0 {
            return dir;
        }
    }
}
// random vector in a unit disk in xy plane (rejection method)
pub fn rand_disk_vec() -> Vec3 {
    let mut rng = rand::thread_rng();
    loop {
        let dir = Vec3 { x: rng.gen_range(-1.0..1.0), y: rng.gen_range(-1.0..1.0), z: 0.0 };
        if dir.magnitude2() <= 1.0 {
            return dir;
        }
    }
}
pub fn clampvec(v: Vec3, min: f32, max: f32) -> Vec3 {
    vec3(v.x.clamp(min, max), v.y.clamp(min, max), v.z.clamp(min, max))
}


////////////////////////////////////////////////////////
/////   CLASSES
////////////////////////////////////////////////////////

// RAY / RAYHIT
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}
#[derive(Clone)]
pub struct RayHit {
    pub distance: f32,
    pub hitpoint: Vec3,
    pub normal: Vec3,
    pub material: Arc<dyn Material + Send + Sync>,
    pub frontface: bool,
    pub tex_coords: Option<Vec2>,
}
impl RayHit {
    pub fn new(distance: f32, normal: Vec3, material: Arc<dyn Material + Send + Sync>, ray: &Ray) -> RayHit {
        let frontface = normal.dot(ray.direction) < 0.0;
        RayHit { 
            distance: distance,
            hitpoint: ray.origin+ray.direction*distance,
            normal: if frontface {normal} else {-normal},
            material: material,
            frontface: frontface,
            tex_coords: None,
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
    pub projection_mode: CameraProjectionMode,
    pub shading_mode: ShadingMode,
    pub path_depth: u32,
    pub path_samples: u32,
    pub screen_width: u32,
    pub screen_height: u32,
    pub focal_length: f32,
    pub focus_dist: f32,
    pub lens_radius: f32,
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
            );
            let focus_plane_pixel_center = cam_space_pixel_center.normalize()*self.focus_dist;
            let lens_origin = self.lens_radius*rand_disk_vec();

            // find rotation from camera to world space:
            let rotation = Matrix3::from_cols(
                self.view_dir.cross(self.up).normalize(),
                self.up,
                -self.view_dir
            );
           
            // create ray with direction still in camera space
            let mut ray = Ray {
                origin: match self.projection_mode {
                    CameraProjectionMode::Orthographic => vec3(cam_space_pixel_center.x, cam_space_pixel_center.y, 0.0 ),
                    CameraProjectionMode::Perspective => self.eyepoint + rotation*lens_origin,
                },
                direction: match self.projection_mode {
                    CameraProjectionMode::Orthographic => self.view_dir,
                    CameraProjectionMode::Perspective => (focus_plane_pixel_center - lens_origin).normalize()
                },
            };
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
                    if matches!(self.camera.shading_mode, ShadingMode::Phong) {
                        final_color += self.phong_shade_ray(&cam_rays[sample_idx]);
                    }
                    else {
                        final_color += self.shade_ray(&cam_rays[sample_idx], 0);
                    }
                }
                final_color = final_color / cam_rays.len() as f32;
                
                let tmp = final_color.clone();
                for i in 0..3 {
                    let d = tmp[i] - 1.0;
                    if d > 0.0 {
                        final_color[(i+1)%3] += d;
                        final_color[(i+2)%3] += d;
                    }
                }


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
    fn background_color(_v: &Vec3) -> Color {
        // let u = v.normalize();
        // let t = 0.5*(u.y+1.0);
        // (1.0-t)*vec3(1.0, 1.0, 1.0) + t*vec3(0.5, 0.7, 1.0)
        Vec3::zero()
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
                hit.material.scatter(&hit, ray).1
                //shadow_weight * (self.ambient + diffuse_weight*hit.material.scatter(&hit, ray).1 + specular_weight*vec3(0.4, 0.4, 0.4))
            }
        }
    }
    
    // computes shading for a ray hit according to the monte-carlo integrated rendering equation
    fn shade_ray(&self, ray: &Ray, depth: u32) -> Color {
        if depth >= self.camera.path_depth { 
            return 0.3*Scene::background_color(&ray.direction); // ambient light model 
        }
        // get hit
        match self.intersect_ray(ray, 0.001, self.camera.max_trace_dist.clone()) {
            None => Scene::background_color(&ray.direction),
            Some(hit) => {
                // accumulate integral
                let mut integral = Color::zero();
                for _i in 0..self.camera.path_samples {
                    // pick new direction, generate ray, and recurse
                    let (new_ray, brdf_term, pdf) = hit.material.scatter(&hit, ray);
                    let dot_term = if hit.normal.magnitude2() > 0.0 {new_ray.direction.dot(hit.normal).abs().clamp(0.0,1.0)} else {1.0};
                    let incoming_light = self.shade_ray(&new_ray, depth+1);
                    // accumulate into integral
                    integral += (dot_term*(brdf_term.mul_element_wise(incoming_light))) / pdf;
                }
                integral /= self.camera.path_samples as f32; 
        
                // total light = integrated + emitted light
                hit.material.emission() + integral
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
            eyepoint: vec3(0.0, 2.0, 5.5),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            focal_length: 0.6,  // distance from eyepoint to image plane
            focus_dist: 5.0,    // distance from eyepoint to focus plane
            lens_radius: 0.0,   // radius of thin-lens approximation
            projection_mode: CameraProjectionMode::Perspective,
            shading_mode: ShadingMode::PathTrace,
            screen_width: 400,
            screen_height: 400,
            aa_sample_count: 500,
            path_depth: 10,     // path-tracing recursion depth
            path_samples: 1,    // sub-rays cast per recursion (slow if more than 1)
            max_trace_dist: 100.0,
            gamma: 2.0,
        },
        objects: Arc::new(vec![
            Arc::new(StaticMesh::load_from_file(
                "./obj/sphere.obj",
                Some("./texture/earthmap.jpg"),
                // Arc::new(Lambertian { albedo: vec3(0.0,0.6,0.0), ..Default::default() }),
                None,
                Matrix4::from_translation(vec3(0.0,1.0,1.0)),
            )),          
            Arc::new(Sphere {
                center: vec3(-1.3,0.5,2.0),
                radius: 0.5,
                material: Arc::new(Dielectric { idx_of_refraction: 2.5 })
            }),
            Arc::new(Sphere {
                center: vec3(-3.0,3.8,-2.0),
                radius: 1.5,
                material: Arc::new(Metal { albedo: vec3(1.0,1.0,0.3), glossiness: 0.0 })
            }),
            Arc::new(Sphere {
                center: vec3(3.0,3.8,-2.0),
                radius: 1.5,
                material: Arc::new(Metal { albedo: vec3(1.0,1.0,1.0), glossiness: 0.1 })
            }),
            Arc::new(Sphere {
                center: vec3(0.0,2.0,-2.5),
                radius: 2.0,
                material: Arc::new(Metal { albedo: vec3(0.2,0.2,0.9), glossiness: 0.05 })
            }),
            Arc::new(Sphere {
                center: vec3(1.0,0.5,2.0),
                radius: 0.5,
                // material: Arc::new(Metal { albedo: vec3(0.7,0.7,0.7), glossiness: 0.2, }),
                material: Arc::new(Dielectric { idx_of_refraction: 1.7 })
            }),
            Arc::new(Sphere {
                center: vec3(0.2,0.35,2.0),
                radius: 0.35,
                material: Arc::new(Lambertian { albedo: vec3(0.3,0.3,0.3), emission: vec3(0.0,1.0,1.0),}),
            }),
            Arc::new(Sphere {
                center: vec3(0.52,0.23,2.5),
                radius: 0.23,
                material: Arc::new(Dielectric { idx_of_refraction: 2.5 })
            }),
            Arc::new(Sphere {
                center: vec3(-0.42,0.5,2.5),
                radius: 0.5,
                material: Arc::new(Dielectric { idx_of_refraction: 2.1})
            }),

            Arc::new(ConvexVolume {
                boundary: Arc::new(Sphere {
                    center: vec3(-2.0,1.0,1.0),
                    radius: 1.0,
                    material: Arc::new(Dielectric { idx_of_refraction: 1.5 }) /* arbitrary */,
                }),
                phase_function: Arc::new(Isotropic { albedo: vec3(1.0,1.0,1.0), emission: Vec3::zero() }),
                density: 0.6,
            }),
            Arc::new(ConvexVolume {
                boundary: Arc::new(Sphere {
                    center: vec3(2.0,1.0,1.0),
                    radius: 1.0,
                    material: Arc::new(Dielectric { idx_of_refraction: 1.5 }) /* arbitrary */,
                }),
                phase_function: Arc::new(Isotropic { albedo: vec3(0.0,0.0,0.0), emission: Vec3::zero() }),
                density: 0.6,
            }),



            // BOX
            Arc::new(Plane {
                point: vec3(0.0, 0.0, 0.0),
                normal: Vec3::unit_y(),
                material: Arc::new(Lambertian { albedo: vec3(0.73,0.73,0.73), ..Default::default() }),
            }),  
            //  Arc::new(Plane {
            //     point: vec3(0.0, 0.0, -4.0),
            //     normal: Vec3::unit_z(),
            //     material: Arc::new(Lambertian { albedo: vec3(0.73,0.73,0.73), ..Default::default() }),
            // }),
            // Arc::new(Plane {
            //     point: vec3(0.0, 5.0, 0.0),
            //     normal: -Vec3::unit_y(),
            //     material: Arc::new(Lambertian { albedo: vec3(0.73,0.73,0.73), ..Default::default() }),
            // }),
            // Arc::new(Plane {
            //     point: vec3(-4.0, 0.0, 0.0),
            //     normal: Vec3::unit_x(),
            //     material: Arc::new(Lambertian { albedo: vec3(0.12,0.45,0.15), ..Default::default() }),
            // }),
            // Arc::new(Plane {
            //     point: vec3(4.0, 0.0, 0.0),
            //     normal: -Vec3::unit_x(),
            //     material: Arc::new(Lambertian { albedo: vec3(0.65,0.05,0.05), ..Default::default() }),
            // }),
            
            // LIGHT
            Arc::new(Triangle {
                a: vec3(-1.5, 4.95, -0.5),
                b: vec3(1.5, 4.95,  -0.5),
                c: vec3(1.5, 4.95, 1.5),
                material: Arc::new(Lambertian { albedo: vec3(0.0,0.6,0.0), emission: vec3(2.0,2.0,2.0), ..Default::default() }),
            }),
            Arc::new(Triangle {
                a: vec3(-1.5, 4.95, -0.5),
                b: vec3(-1.5, 4.95,  1.5),
                c: vec3(1.5, 4.95, 1.5),
                material: Arc::new(Lambertian { albedo: vec3(0.0,0.6,0.0), emission: vec3(2.0,2.0,2.0), ..Default::default() }),
            }),
            // Arc::new(Sphere {
            //     center: vec3(0.0,6.0,-1.0),
            //     radius: 1.6,
            //     material: Arc::new(Lambertian { albedo: vec3(0.6,0.3,0.3), emission: vec3(5.0,5.0,5.0) }),
            // }),

        ]),
        point_light_pos: vec3(0.0,1.0,5.0), // for phong shading only
        ambient: vec3(0.1,0.1,0.1), // for phong shading only
    };

    // render and write output
    scene.render_to_image().save_with_format("render.png", ImageFormat::Png).unwrap();

}