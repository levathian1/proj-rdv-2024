#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

const int WIDTH = 1024;
const int HEIGHT = 768;

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

void render() {
    std::vector<Vec3f> framebuffer(WIDTH*HEIGHT);

    for (size_t j = 0; j<HEIGHT; j++) {
        for (size_t i = 0; i<WIDTH; i++) {
            framebuffer[i+j*WIDTH] = Vec3f(j/float(HEIGHT),i/float(WIDTH), 0);
        }
    }

    save_to_file("file.ppm", framebuffer);
}

int main() {
    render();
    return 0;
}