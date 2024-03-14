#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const Vec3f &color) : diffuse_color(color) {}
    Material() : diffuse_color() {}
    Vec3f diffuse_color;
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
        float tan_theta2 = (radius * radius) / (height * height);

        float a = dir.x * dir.x + dir.y * dir.y - tan_theta2 * dir.z * dir.z;
        float b = 2 * (L.x * dir.x + L.y * dir.y - tan_theta2 * L.z * dir.z);
        float c = L.x * L.x + L.y * L.y - tan_theta2 * L.z * L.z;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
            return false;

        float sqrt_discriminant = sqrt(discriminant);
        float t1 = (-b + sqrt_discriminant) / (2 * a);
        float t2 = (-b - sqrt_discriminant) / (2 * a);

        float t_min = std::min(t1, t2);
        float t_max = std::max(t1, t2);

        float intersection_distance = L.z + t_min * dir.z; // Distance from the cone's base to the intersection point along the axis

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

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Cone &cone, const std::vector<Light> &lights) {
    Vec3f point, N;
    Material material;

    if (!scene_intersect(orig, dir, cone, point, N, material)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }

    float diffuse_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
    }
    return material.diffuse_color * diffuse_light_intensity;
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

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height*width; ++i) {
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main() {
    Material      ivory(Vec3f(0.4, 0.4, 0.3));
    Material red_rubber(Vec3f(0.3, 0.1, 0.1));
    Cone cone(Vec3f(-3, 0, -16), 2, 1, ivory);
    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));

    render(cone, lights);

    return 0;
}

