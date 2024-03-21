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

const int WIDTH = 1024;
const int HEIGHT = 768;
const int FOV = M_PI/2;

int envmap_width, envmap_height;
std::vector<Vec3f> envmap;

struct Material {
    Material(const Vec2f &a, const Vec3f &color, const float &spec) : albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : albedo(1,0), diffuse_color(), specular_exponent() {}
    Vec2f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

// (0, 0, z) is centered in 2d space
struct Sphere{
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}

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

// TODO: redocument maths behind this for posteriority and to check understanding
struct Cone{
    Vec3f centre;
    float radius;
    float height; 
    Material material;

    Cone(const Vec3f &c, const float &r, const float &h, const Material &m): centre(c), radius(r), height(h), material(m){}

// readapt from circle version
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = orig - centre; // if ordered same way as test file, nothing is displayed anymore

// Going back to the original formulas i tried (without using tan angle with known rad & height), seems to work fine
        float a = dir.x * dir.x + dir.y * dir.y - dir.z * dir.z;
        float b = 2 * (L.x * dir.x + L.y * dir.y - L.z * dir.z);
        float c = L.x * L.x + L.y * L.y - L.z * L.z;

        float d = b * b - 4 * a * c;
        if (d < 0)
            return false;

        float t1 = (-b + sqrt(d)) / (2 * a);
        float t2 = (-b - sqrt(d)) / (2 * a);

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

struct Cylinder{
    Vec3f centre;
    float radius;
    float height;
    Material material;

    Cylinder(const Vec3f &c, const float &r, const float &h, const Material &m) : centre(c), radius(r), height(h), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {

        Vec3f L = orig - centre; // if ordered same way as test file, nothing is displayed anymore

        float a = dir.x * dir.x + dir.y * dir.y;
        float b = 2 * (dir.x * L.x + dir.y * L.y);
        float c = L.x * L.x + L.y * L.y - radius * radius;

        float d = b * b - 4 * a * c;
        if (d < 0)
            return false;

        float t1 = (-b + sqrt(d)) / (2 * a);
        float t2 = (-b - sqrt(d)) / (2 * a);

        float t;
        if (t1 > t2)t = t2; else t = t1;

        float z = L.z + t * dir.z;

        if (z > 0 && z >= 0 && z <= height) {
            return true;
        }

        return false;
    }

};

struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

// Perlin noise algorithm implementation from wikipedia with amendment to interp weights

float interpolate(float a0, float a1, float w) {
    // You may want clamping by inserting:
     if (0.0 > w) return a0;
     if (1.0 < w) return a1;

    return (a1 - a0) * w + a0;
    /* // Use this cubic interpolation [[Smoothstep]] instead, for a smooth appearance:
     * return (a1 - a0) * (3.0 - w * 2.0) * w * w + a0;
     *
     * // Use [[Smootherstep]] for an even smoother result with a second derivative equal to zero on boundaries:
     * return (a1 - a0) * ((w * (w * 6.0 - 15.0) + 10.0) * w * w * w) + a0;
     */
}

typedef struct {
    float x, y;
} vector2;

/* Create pseudorandom direction vector
 */
vector2 randomGradient(int ix, int iy) {
    // No precomputed gradients mean this works for any number of grid coordinates
    const unsigned w = 8 * sizeof(unsigned);
    const unsigned s = w / 2; // rotation width
    unsigned a = ix, b = iy;
    a *= 3284157443; b ^= a << s | a >> w-s;
    b *= 1911520717; a ^= b << s | b >> w-s;
    a *= 2048419325;
    float random = a * (3.14159265 / ~(~0u >> 1)); // in [0, 2*Pi]
    vector2 v;
    v.x = cos(random); v.y = sin(random);
    return v;
}

// Computes the dot product of the distance and gradient vectors.
float dotGridGradient(int ix, int iy, float x, float y) {
    // Get gradient from integer coordinates
    vector2 gradient = randomGradient(ix, iy);

    // Compute the distance vector
    float dx = x - (float)ix;
    float dy = y - (float)iy;

    // Compute the dot-product
    return (dx*gradient.x + dy*gradient.y);
}

// Compute Perlin noise at coordinates x, y
float noise(float x, float y) {
    // Determine grid cell coordinates
    int x0 = (int)floor(x);
    int x1 = x0 + 1;
    int y0 = (int)floor(y);
    int y1 = y0 + 1;

    

    // Determine interpolation weights
    // Could also use higher order polynomial/s-curve here
    // Fixing values here instead of original to force positive values at all times
    float sx = 0.7;
    float sy = 0.3;

    // std::cout << x << " " << x0 << "\n";

    // Interpolate between grid point gradients
    float n0, n1, ix0, ix1, value;

    n0 = dotGridGradient(x0, y0, x, y);
    n1 = dotGridGradient(x1, y0, x, y);
    ix0 = interpolate(n0, n1, sx);

    n0 = dotGridGradient(x0, y1, x, y);
    n1 = dotGridGradient(x1, y1, x, y);
    ix1 = interpolate(n0, n1, sx);

    value = interpolate(ix0, ix1, sy);
    return value; // Will return in range -1 to 1. To make it in range 0 to 1, multiply by 0.5 and add 0.5
}



void save_to_file(const char* filename, std::vector<Vec3f> framebuffer){
    std::ofstream ofs; // save the framebuffer to file
    ofs.open(filename, std::ios::binary);
    ofs << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    for (size_t i = 0; i < HEIGHT*WIDTH; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}


float signed_distance(const Vec3f &p, float sphere_radius) {
    Vec3f s = Vec3f(p).normalize(sphere_radius);
    float displacement = sin(16*s.x)*sin(16*s.y)*sin(16*s.z)*0.2;
    return sqrt(p.x*p.x+p.y*p.y+p.z*p.z) - (sphere_radius + displacement);
}


bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Cone> &cones, const Cylinder cylinder, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    float cones_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            Vec3f light_dir = (Vec3f(10, 10, 10) - hit).normalize();
            float dist = signed_distance(dir, spheres[i].radius);
            float displacement = (sin(16*hit.x)*sin(16*hit.y)*sin(16*hit.z) + 1.)/2.;
            N = (hit - spheres[i].center);
            N.x += displacement*dist; N.y += displacement*dist; N.z += displacement*dist    ;
            N.normalize();
            material = spheres[i].material;
        }
    }
    for (size_t i=0; i < cones.size(); i++) {
        float dist_i;
        
        if (cones[i].ray_intersect(orig, dir, dist_i) && dist_i < cones_dist) {
            cones_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - cones[i].centre).normalize();
            material = cones[i].material;
        }
    }
    // float cylinder_dist = std::numeric_limits<float>::max();
    //     float dist_i;
    //     if (cylinder.ray_intersect(orig, dir, dist_i) && dist_i < cylinder_dist) {
    //         cylinder_dist = dist_i;
    //         hit = orig + dir*dist_i;
    //         N = (hit - cylinder.centre).normalize();
    //         material = cylinder.material;
    //     }
    
    return spheres_dist < 1000 || cones_dist < 1000;
}


// shadow calc for cone seems to be fragmented in parts 
// TODO: figure out origin
// TODO: go over course on it again, some part of the old calc goofs up on the cone specifically
Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Cone> &cones, const Cylinder cylinder, const std::vector<Light> &lights) {
    Vec3f point, N;
    Material material;

    if (!scene_intersect(orig, dir, spheres, cones, cylinder, point, N, material)) {
        float p1 = pow(dir.x, 2);
        float p2 = pow(dir.z, 2);
        float th = atan2f(dir.z, dir.x);
        float l = (envmap_height/2)*(1-sin(atan2f(dir.y, sqrtf(p1+p2))));
        float c = (envmap_width/2)*(1-(-th/M_PI));
        float align = floor(envmap_width*l+1000);
        return envmap[c+floor(l)*envmap_width];
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
        if (scene_intersect(shadow_orig, light_dir, spheres, cones, cylinder, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity* material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Cone> &cones, const Cylinder cylinder, const std::vector<Light> &lights) {
    std::vector<Vec3f> framebuffer(WIDTH*HEIGHT);
    std::vector<Vec3f> perlin(WIDTH*HEIGHT*3);


    std::cout << "print";


    for (size_t j = 0; j<HEIGHT; j++) {
        for (size_t i = 0; i<WIDTH; i++) {
            float x =  (2*(i + 0.5)/(float)WIDTH - 1)*tan(FOV/2.)*WIDTH/(float)HEIGHT;
            float y = -(2*(j + 0.5)/(float)HEIGHT - 1)*tan(FOV/2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i+j*WIDTH] = cast_ray(Vec3f(0,0,0), dir, spheres, cones, cylinder, lights);
            float noise_val = noise(i, j);
            if(noise_val > 0.5){
                framebuffer[i+j*WIDTH] = Vec3f(255, 255, 255);
                perlin[i+j*WIDTH] = Vec3f(255, 255, 255);
            }
        }
    }

    save_to_file("file.ppm", framebuffer);

    std::vector<unsigned char> pixmap(WIDTH*HEIGHT*3);
    for (size_t i = 0; i < HEIGHT*WIDTH; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            pixmap[i*3+j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
            // pixmap[i*3+j] = (unsigned char)(0);
        }
    }

    save_to_file("pog.ppm", perlin);
    stbi_write_jpg("out.jpg", WIDTH, HEIGHT, 3, pixmap.data(), 100);
}

int main() {

    int n = -1;
    unsigned char *pixmap = stbi_load("envmap5.jpg", &envmap_width, &envmap_height, &n, 0);
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

    Material ivory(Vec2f(0.6,  0.3), Vec3f(0.7, 0.72, 0.7),   50.);
    Material red_rubber(Vec2f(0.9,  0.1), Vec3f(0.3, 0.1, 0.1),   10.);
    Material black(Vec2f(0.6,  0.3), Vec3f(0.0, 0.0, 0.0), 50.);

    Sphere sphere1(Vec3f(0, -2, -16), 3, ivory);
    Sphere sphere2(Vec3f(0, 2, -16), 2.5, ivory);
    Sphere sphere3(Vec3f(0, 5, -16), 2, ivory);
    Sphere lefteye(Vec3f(-0.5, 5.5, -15), 0.8, black);
    Sphere righteye(Vec3f(0.5, 5.5, -15), 0.8, black);
   
    Sphere b1(Vec3f(0, 1.5, -13), 0.2, black);
    Sphere b2(Vec3f(0, 3, -13), 0.2, black);
    Sphere b3(Vec3f(0, 0, -13), 0.2, black);
    Sphere b4(Vec3f(0, -1.5, -13), 0.2, black);
    Sphere b5(Vec3f(0, -3, -13), 0.2, black);
   // Sphere b6(Vec3f(2.7, 2.5, -15), 0.8, ivory);
    //Sphere b7(Vec3f(-2.7, 2.5, -15), 0.8, ivory);


// Weird checker board shadowing on cone seems to be caused by incorrect lighting
// Temp fix is to render cone larger and move it about to keep similar appearance while having it catch more light than previous
// TODO: find nice lighting to go back to smaller cone in the original spot if possible
    Cone cone(Vec3f(0, 11, -32), 1, 1, red_rubber);

    std::vector<Sphere> spheres;
    std::vector<Cone> cones;
    std::vector<Light>  lights;

    spheres.push_back(sphere1);
    spheres.push_back(sphere2);
    spheres.push_back(sphere3);
    spheres.push_back(lefteye);
    spheres.push_back(righteye);
    spheres.push_back(b1);
    spheres.push_back(b2);
    spheres.push_back(b3);
    spheres.push_back(b4);
    spheres.push_back(b5);
   // spheres.push_back(b6);
    //spheres.push_back(b7);
    



    cones.push_back(cone);

    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));
    lights.push_back(Light(Vec3f( 30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f( 10, 30, -10), 1.8));
    lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));

    Cylinder cylinder(Vec3f(0, 0, -16), 0.5, 20, ivory);

    render(spheres, cones, cylinder, lights);
    return 0;
}