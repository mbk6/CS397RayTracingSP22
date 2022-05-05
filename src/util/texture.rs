// TEXTURE - implements texture loading and sampling

#![allow(dead_code)]

use image::*;
use cgmath::*;

use super::tracing::*;


#[derive(Debug, Clone)]
pub struct Texture {
    img: DynamicImage
}
impl Texture {
    pub fn load_from_file(file_name: &str) -> Option<Texture> {
        if let Ok(img) = image::open(file_name) {
            Some(Texture {
                img: img,
            })
        }
        else {
            None
        }
    }
    pub fn sample(&self, uv: Vec2) -> Color {
        // nearest neighbor for now:
        let mut x = u32::min((uv.x.clamp(0.0, 0.999)*self.img.width() as f32) as u32, self.img.width()-1);
        let mut y = u32::min(((1.0-uv.y.clamp(0.0, 0.999))*self.img.height() as f32) as u32, self.img.height()-1);
        let pxl = self.img.get_pixel(x,y).to_rgb();
        vec3(pxl[0] as f32/255.0, pxl[1] as f32/255.0, pxl[2] as f32/255.0)
        //vec3(1.0,0.0,1.0)
    }
}


