#include "rtweekend.h"

#include "camera.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"

#include <vector>
#include <future>
#include <algorithm>
#include <execution>
#include <iostream>
#include <memory>
#include <random>

int main()
{
    hittable_list world;

    auto ground_material = make_shared<metal>(color(0.5, 0.5, 0.5), 0.05);
    world.add(make_shared<sphere>(point3(0, -3000, 0), 3000, ground_material));

    int counter = 0;
    const int iterCount = (30 * 30); // assuming this is the total number of iterations

    std::vector<std::pair<int, int>> positions;
    positions.reserve(iterCount);

    for (int a = -15; a < 15; ++a) {
        for (int b = -15; b < 15; ++b) {
            positions.emplace_back(a, b);
        }
    }

    std::mutex vector_mutex;
    std::mutex log_mutex;

    std::for_each(std::execution::par, positions.begin(), positions.end(), [&](const std::pair<int, int>& pos) {
        int a = pos.first;
        int b = pos.second;
        auto choose_mat = random_double();
        const auto radius = random_double(0.1, 0.3);
        point3 center(a + 0.9 * random_double(), radius, b + 0.9 * random_double());

        if ((center - point3(4, radius, 0)).length() > 0.9) {
            {
                std::scoped_lock lock(log_mutex);
                std::clog << "\rGenerating world: iteration #" << std::to_string(++counter) << "/" << std::to_string(iterCount) << std::flush;
            }

            std::shared_ptr<material> sphere_material;

            if (choose_mat < 0.75) {
                // diffuse
                auto albedo = color::random() * color::random();
                sphere_material = std::make_shared<lambertian>(albedo);
            } else if (choose_mat < 0.90) {
                // metal
                auto albedo = color::random(0.5, 1);
                auto fuzz = random_double(0, 0.5);
                sphere_material = std::make_shared<metal>(albedo, fuzz);
            } else {
                // glass
                sphere_material = std::make_shared<dielectric>(1.5);
            }

            {
                std::scoped_lock lock(vector_mutex);
                world.add(std::make_shared<sphere>(center, radius, sphere_material));
            }
        }
    });

    std::clog << "\rDone Generating world.                 \n";

    auto material1 = make_shared<dielectric>(1.5);
    world.add(make_shared<sphere>(point3(0,1,0), 1.0, material1));

    auto material2 = make_shared<lambertian>(color(0.118, 0.376, 0.478));
    world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
    world.add(make_shared<sphere>(point3(4,1,0), 1.0, material3));

    camera cam;

    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 3840;
    cam.samples_per_pixel = 500;
    cam.max_depth = 50;

    cam.vfov = 20;
    cam.lookfrom = point3(13, 1.7, 3);
    cam.lookat = point3(0, 0.2, 0);
    cam.vup = vec3(0, 1, 0);

    cam.defocus_angle = 0.6;
    cam.focus_dist = 10.0;

    cam.render(world);
}