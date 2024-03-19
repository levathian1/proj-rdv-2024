#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int envmap_width, envmap_height;
std::vector<Vec3f> envmap;

struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const Vec2f &a, const Vec3f &color, const float &spec) : albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : albedo(1,0), diffuse_color(), specular_exponent() {}
    Vec2f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;

    Sphere(const Vec3f &c, const float &r) : center(c), radius(r) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

struct Cone{
    Vec3f centre;
    float radius;
    float height; 
    Material m;

    Cone(const Vec3f &c, const float &r, const float &h, const Material m): centre(c), radius(r), height(h), m(m){}

// readapt from circle version
     bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = orig - centre;
        float tan_theta = (radius) / (height);

        float a = dir.x * dir.x + dir.y * dir.y - tan_theta * dir.z * dir.z;
        float b = 2 * (L.x * dir.x + L.y * dir.y - tan_theta * L.z * dir.z);
        float c = L.x * L.x + L.y * L.y - tan_theta * L.z * L.z;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
            return false;

// Simplify this in landing code to save a precious line
        float sqrt_discriminant = sqrt(discriminant);
        float t1 = (-b + sqrt_discriminant) / (2 * a);
        float t2 = (-b - sqrt_discriminant) / (2 * a);

        float t_min = std::min(t1, t2);
        float t_max = std::max(t1, t2);

        float intersection_distance = L.z + t_min * dir.z;

        if (t_min > 0 && intersection_distance >= 0 && intersection_distance <= height) {
            t0 = t_min;
            return true;
        }

        if (t_max > 0 && intersection_distance >= 0 && intersection_distance <= height) {
            t0 = t_max;
            return true;
        }

        return false;
    }
};

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const Cone &cone, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
        float dist_i;
        if (cone.ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - cone.centre).normalize();
            material = cone.m;
        }
    
    return spheres_dist<1000;
}

Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Cone &cone, const std::vector<Light> &lights) {
    Vec3f point, N;
    Material material;

    if (!scene_intersect(orig, dir, cone, point, N, material)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();

        float light_distance = (lights[i].position - point).norm();

// Replacing 1e-3 seems to solve weird checkered spot on the cone, as if there was a blind spot not reached correctly when calculating things
// TODO: look into reason why
        Vec3f shadow_orig = light_dir*N < 0 ? point - (N * 2) : point + (N * 2); // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, cone, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity* material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1];
}

void render(const Cone &cone, const std::vector<Light> &lights ) {
    const int width    = 1024;
    const int height   = 768;
    const int fov      = M_PI/2.;
    std::vector<Vec3f> framebuffer(width*height);

    #pragma omp parallel for
    for (size_t j = 0; j<height; j++) {
        for (size_t i = 0; i<width; i++) {
            float x =  (2*(i + 0.5)/(float)width  - 1)*tan(fov/2.)*width/(float)height;
            float y = -(2*(j + 0.5)/(float)height - 1)*tan(fov/2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i+j*width] = cast_ray(Vec3f(0,0,0), dir, cone, lights);
        }
    }
    // std::ofstream ofs; // save the framebuffer to file
    // ofs.open("./out.ppm");
    // ofs << "P6\n" << width << " " << height << "\n255\n";
    // for (size_t i = 0; i < height*width; ++i) {
    //     for (size_t j = 0; j<3; j++) {
    //         ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
    //     }
    // }
    // ofs.close();

    std::vector<unsigned char> pixmap(width*height*3);
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            pixmap[i*3+j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    stbi_write_jpg("out.jpg", width, height, 3, pixmap.data(), 100);
}

int main() {

    int n = -1;
    unsigned char *pixmap = stbi_load("envmap.jpg", &envmap_width, &envmap_height, &n, 0);
    if (!pixmap || 3!=n) {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;
    }
    envmap = std::vector<Vec3f>(envmap_width*envmap_height);
    for (int j = envmap_height-1; j>=0 ; j--) {
        for (int i = 0; i<envmap_width; i++) {
            envmap[i+j*envmap_width] = Vec3f(pixmap[(i+j*envmap_width)*3+0], pixmap[(i+j*envmap_width)*3+1], pixmap[(i+j*envmap_width)*3+2])*(1/255.);
        }
    }
    stbi_image_free(pixmap);

    Material      ivory(Vec2f(0.6,  0.3), Vec3f(0.4, 0.4, 0.3),   50.);
    Material red_rubber(Vec2f(0.9,  0.1), Vec3f(0.3, 0.1, 0.1),   10.);
    Cone cone(Vec3f(0, 3, -16), 2, 1, ivory);
    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));
    lights.push_back(Light(Vec3f( 30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));

    render(cone, lights);

    return 0;
}

