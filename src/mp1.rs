// MP1 - based on CS 419 mp1

use std::{ops::Neg, iter::Scan};

// includes
use image::*;
use cgmath::*;

// constants & typedefs
type Vec3 = Vector3<f32>;

// TRAITS
trait Intersectable {

}

// STRUCTS & ENUMS
struct Camera {
    pub eyepoint: Vec3,
    pub view_dir: Vec3,
    pub up: Vec3,
    pub view_plane_dist: f32,
    pub pixel_size: f32,
    pub projection_mode: CameraProjectionMode,
    pub screen_width: u32,
    pub screen_height: u32,
}
pub enum CameraProjectionMode {
    Orthographic,
    Perspective,
}
struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}
struct Scene {
    pub camera: Camera,
    pub objects: Vec<Box<dyn Intersectable>>,
}
struct Sphere {
    pub center: Vec3,
}

// IMPL
impl Camera {
    // generate camera ray given pixel coordinates
    pub fn generate_ray(&self, screen_x: u32, screen_y: u32) -> Ray {
        let cam_space_pixel_center = Vec3 {
            x: self.pixel_size*(screen_x as f32 - 0.5*(self.screen_width as f32) + 0.5),
            y: self.pixel_size*(0.5 + 0.5*(self.screen_height as f32) - screen_y as f32),
            z: -self.view_plane_dist,
        }.normalize();
        // create ray with direction still in camera space
        let mut ray = Ray {
            origin: match self.projection_mode {
                CameraProjectionMode::Orthographic => Vec3 { x: cam_space_pixel_center.x, y: cam_space_pixel_center.y, z: 0.0 },
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
        return ray;
    }
}
impl Scene {
    pub fn render_to_image(&self) -> DynamicImage {
        let mut img = DynamicImage::new_rgb8(self.camera.screen_width, self.camera.screen_height);
        for x in 0..self.camera.screen_width {
            for y in 0..self.camera.screen_height {
                let cam_ray = self.camera.generate_ray(x, y);
                let color = Rgba::from_channels(
                    (cam_ray.direction.x as f32 * 255.9999) as u8,
                    (cam_ray.direction.y as f32 * 255.9999) as u8,
                    (cam_ray.direction.z as f32 * 255.9999) as u8,
                    0);
                img.put_pixel(x, y, color);
            }
        }
        return img;
    }
}


// FUNCTIONS
pub fn run() {
    let scene = Scene {
        camera: Camera {
            eyepoint: Vec3::zero(),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            view_plane_dist: 1.0,
            pixel_size: 0.05,
            projection_mode: CameraProjectionMode::Perspective,
            screen_width: 400,
            screen_height: 400,
        },
        objects: Vec::new(),
    };

    scene.render_to_image().save_with_format("mp1.png", ImageFormat::Png);

}