#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

const int WIDTH = 1024;
const int HEIGHT = 768;
const int FOV = M_PI/2;

// (0, 0, z) is centered in 2d space
struct Sphere{
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


void save_to_file(const char* filename, std::vector<Vec3f> framebuffer){
    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm", std::ios::binary);
    ofs << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    for (size_t i = 0; i < HEIGHT*WIDTH; ++i) {
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
        }
    }
    return spheres_dist<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres) {
    Vec3f point, N;
    if (!scene_intersect(orig, dir, spheres, point, N)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }
    return Vec3f(0.4, 0.4, 0.3);
}

void render(const std::vector<Sphere> &spheres) {
    std::vector<Vec3f> framebuffer(WIDTH*HEIGHT);

    for (size_t j = 0; j<HEIGHT; j++) {
        for (size_t i = 0; i<WIDTH; i++) {
            float x =  (2*(i + 0.5)/(float)WIDTH - 1)*tan(FOV/2.)*WIDTH/(float)HEIGHT;
            float y = -(2*(j + 0.5)/(float)HEIGHT - 1)*tan(FOV/2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i+j*WIDTH] = cast_ray(Vec3f(0,0,0), dir, spheres);
        }
    }

    save_to_file("file.ppm", framebuffer);
}

int main() {
    Sphere sphere1(Vec3f(0, -3, -16), 2);
    Sphere sphere2(Vec3f(0, 0, -16), 2);
    Sphere sphere3(Vec3f(0, 3, -16), 2);

    std::vector<Sphere> spheres;

    spheres.push_back(sphere1);
    spheres.push_back(sphere2);
    spheres.push_back(sphere3);

    render(spheres);
    return 0;
}