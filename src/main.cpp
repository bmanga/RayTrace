#include <vector>
#include <optional>

#include <SFML/Graphics.hpp>
#include <eigen3/Eigen/Core>
#include <range/v3/view.hpp>
#include <range/v3/algorithm.hpp>
#include <fmt/format.h>


using Vec3f = Eigen::Vector3f;
using Color3f = Eigen::Vector3f;
using SphereIdx = size_t;

struct RayBase
{
    Vec3f origin;
    Vec3f direction;
};

struct NormalizedRay : RayBase
{
};

struct Ray : RayBase
{
    NormalizedRay normalized() const
    {
        return {origin, direction.normalized() };
    }
};

struct SphereGeometry
{
    Vec3f origin;
    float squared_radius; // radius * radius, which is what we usually work on
};

struct SphereProperty
{
    Color3f surface_color;
    Color3f emission_color;
    float transparency;
    float reflectivity;
};

std::optional<std::pair<float, float>> ray_intersection(SphereGeometry sphere, NormalizedRay ray)
{
    Vec3f VOC = sphere.origin - ray.origin;
    float OA = VOC.dot(ray.direction);

    if (OA < 0) {
        return std::nullopt;
    }
    float OC2 = VOC.dot(VOC);
    float AC2 = OC2 - OA * OA;

    if (AC2 > sphere.squared_radius) {
        return std::nullopt;
    }
    float AP2 = sphere.squared_radius - AC2;
    float AP = std::sqrt(AP2);

    return {{OA - AP, OA + AP}};
}

struct IntersectionResult
{
    SphereIdx sphere_idx;
    std::pair<float, float> intersection_points;
    float closest_positive_intersection;

    bool operator <(const IntersectionResult &rhs) const {
        return closest_positive_intersection < rhs.closest_positive_intersection;
    }
    bool is_ray_origin_inside() const
    {
        // If the closest point is not the first, it means that the first
        // point was negative, and hence the origin was inside.
        return closest_positive_intersection != intersection_points.first;
    }
};

std::optional<IntersectionResult> find_ray_intersection(const std::vector<SphereGeometry> &spheres, NormalizedRay ray)
{
    IntersectionResult min = {SphereIdx(-1), {}, std::numeric_limits<float>::max()};

    for (auto [idx, sphere] : ranges::view::zip(ranges::view::indices, spheres)) {
        auto maybe_intersection = ray_intersection(sphere, ray);
        if (maybe_intersection.has_value()) {
            auto intersections = maybe_intersection.value();
            IntersectionResult tentative = {SphereIdx(idx), intersections, intersections.first < 0
                                                                           ? intersections.second
                                                                           : intersections.first   };
            min = std::min(min, tentative);
        }
    }

    if (min.sphere_idx == SphereIdx(-1)) {
        return std::nullopt;
    }
    return min;
}


#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif


using Vec3f = Eigen::Vector3f;


float mix(const float &a, const float &b, const float &mix)
{
    return b * mix + a * (1 - mix);
}

Color3f trace(
        const std::vector<SphereGeometry> &sphere_geometries,
        const std::vector<SphereProperty> &sphere_properties,
        NormalizedRay ray,
        Color3f background,
        int depth, int max_depth)
{

    auto maybe_sphere_intersect = find_ray_intersection(sphere_geometries, ray);
    if (!maybe_sphere_intersect) {
        // Return background color if there are no intersections.
        return background;
    }

    IntersectionResult intersection = maybe_sphere_intersect.value();
    unsigned idx = intersection.sphere_idx;
    float tnear = intersection.closest_positive_intersection;

    const SphereGeometry geometry = sphere_geometries[idx];
    const SphereProperty properties = sphere_properties[idx];

    Color3f surfaceColor = {0, 0, 0};
    Vec3f phit = ray.origin + ray.direction * tnear; // point of intersection
    Vec3f nhit = (phit - geometry.origin).normalized(); // normal at the intersection point

    // If the normal and the view direction are not opposite to each other
    // reverse the normal direction. That also means we are inside the sphere so set
    // the inside bool to true. Finally reverse the sign of IdotN which we want
    // positive.
    float bias = 1e-4; // add some bias to the point from which we will be tracing
    bool inside = false;
    if (intersection.is_ray_origin_inside()) nhit = -nhit, inside = true;
    if ((properties.transparency > 0 || properties.reflectivity > 0) && depth < max_depth) {
        float cosi = ray.direction.dot(nhit);
        // change the mix value to tweak the effect
        float fresneleffect = mix(pow(1 + cosi, 3), 1, 0.1);
        // compute reflection direction (not need fargcto normalize because all vectors
        // are already normalized)
        Vec3f refldir = ray.direction - nhit * 2 * cosi;
        refldir.normalize();
        Color3f reflection = trace(sphere_geometries, sphere_properties,
                                   NormalizedRay{phit + nhit * bias, refldir}, background, depth + 1, max_depth);
        Color3f refraction = {0, 0, 0};
        // if the sphere is also transparent compute refraction ray (transmission)
        if (properties.transparency) {
            float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
            float k = 1 - eta * eta * (1 - cosi * cosi);
            Vec3f refrdir = (ray.direction * eta + nhit * (eta *  -cosi - sqrt(k))).normalized();
            refraction = trace(sphere_geometries, sphere_properties,
                               NormalizedRay{phit - nhit * bias, refrdir}, background, depth + 1, max_depth);
        }
        // the result is a mix of reflection and refraction (if the sphere is transparent)
        surfaceColor = (
                         reflection * fresneleffect +
                         refraction * (1 - fresneleffect) * properties.transparency
                       ).cwiseProduct(properties.surface_color);
    }
    else {
        //return surfaceColor + sphere->emissionColor;
        // it's a diffuse object, no need to raytrace any further
        for (unsigned i = 0; i < sphere_geometries.size(); ++i) {
            if (sphere_properties[i].emission_color.x() > 0) {
                // this is a light
                float transmission = 1.f;
                Vec3f lightDirection = (sphere_geometries[i].origin - phit).normalized();
                for (unsigned j = 0; j < sphere_geometries.size(); ++j) {
                    if (i != j) {
                        if (ray_intersection(sphere_geometries[j], NormalizedRay{phit + nhit * bias, lightDirection})) {
                            transmission = 0.f;
                            break;
                        }
                    }
                }
                surfaceColor += (properties.surface_color * transmission *
                                   std::max(float(0), nhit.dot(lightDirection))
                                 ).cwiseProduct(sphere_properties[i].emission_color);
            }
        }
    }
    return surfaceColor + properties.emission_color;
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a sphere, we return the color of the
// sphere at the intersection point, else we return the background color.
//[/comment]
void render(uint8_t *buffer,
            const std::vector<SphereGeometry> &sphere_geometries,
            const std::vector<SphereProperty> &sphere_properties,
            Color3f background, int max_depth)
{
    unsigned width = 640, height = 480;
    Color3f *image = new Color3f[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    float fov = 30, aspectratio = width / float(height);
    float angle = tan(M_PI * 0.5 * fov / 180.);
    // Trace rays
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x, ++pixel) {
            float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
            float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
            Vec3f raydir(xx, yy, -1);
            raydir.normalize();
            *pixel = trace(sphere_geometries, sphere_properties,
                           NormalizedRay{Vec3f(0, 0, 0), raydir}, background, 0, max_depth);
        }
    }

    for (unsigned i = 0; i < width * height; ++i) {
            unsigned buffer_idx = i * 4;
            buffer[buffer_idx] =(unsigned char)(std::min(float(1), image[i].x()) * 255);
            buffer[buffer_idx + 1] = (unsigned char)(std::min(float(1), image[i].y()) * 255);
            buffer[buffer_idx + 2] = (unsigned char)(std::min(float(1), image[i].z()) * 255);
            buffer[buffer_idx + 3] = 128;
    }
    delete [] image;
}

//[comment]
// In the main function, we will create the scene which is composed of 5 spheres
// and 1 light (which is also a sphere). Then, once the scene description is complete
// we render that scene, by calling the render() function.
//[/comment]
int main(int argc, char **argv)
{
    int max_depth = 5;
    if (argc > 1) {
        max_depth = atol(argv[1]);
    }

    sf::RenderWindow window(sf::VideoMode(640, 480), "SFML ");

    std::vector<SphereGeometry> sphere_geometries;
    std::vector<SphereProperty> sphere_properties;

    // surface_color, emission_color, transparency, reflectivity;


    sphere_geometries.push_back({Vec3f( 0.0, -10'004, -20), 10'000 * 10'000});
    sphere_properties.push_back({Color3f(0.20, 0.20, 0.20),{0, 0, 0}, 0.0, 0.0});

    // position, radius, surface color, reflectivity, transparency, emission color
    sphere_geometries.push_back({Vec3f( 0.0, 0, -20), 4 * 4});
    sphere_properties.push_back({Color3f(1, 0.32, 0.36),{0, 0, 0}, 0.5, 1.0});

    sphere_geometries.push_back({Vec3f( 5.0, -1, -15), 2 * 2});
    sphere_properties.push_back({Color3f(0.90, 0.76, 0.46),{0, 0, 0}, 0, 1.0});

    sphere_geometries.push_back({Vec3f( 5.0, 0, -25), 3 * 3});
    sphere_properties.push_back({Color3f(0.65, 0.77, 0.97),{0, 0, 0}, 0.0, 1.0});


    sphere_geometries.push_back({Vec3f( -5.5, 0, -15), 3 * 3});
    sphere_properties.push_back({Color3f(0.90, 0.90, 0.90), {0, 0, 0}, 0, 1.f});

    // light
    sphere_geometries.push_back({Vec3f( 0.0,     20, -30),     9});
    sphere_properties.push_back({Color3f(0.00, 0.00, 0.00), Color3f{3, 3, 3}, 0, 0.0});

    //render(sphere_geometries, sphere_properties, Color3f{2, 2, 2}, max_depth);

    uint8_t *buffer = new uint8_t[640 * 480 * 4];
    sf::Texture rendered;
    rendered.create(640, 480);

    bool toggle = false;
    while (window.isOpen()) {
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
                toggle = !toggle;
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
                sphere_geometries[1].origin += Vec3f(0, 0.1, 0);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
                sphere_geometries[1].origin += Vec3f(0, -0.1, 0);
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
                sphere_geometries[1].origin += Vec3f(-0.1, 0, 0);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
                sphere_geometries[1].origin += Vec3f(0.1, 0, 0);
            }


            render(buffer, sphere_geometries, sphere_properties, toggle ? Color3f{2, 2, 2} : Color3f{1, 2, 3}, max_depth);
            rendered.update(buffer);
            sf::Sprite sprite(rendered);

            window.clear();
            window.draw(sprite);
            window.display();
        }
    }

    delete []buffer;
    return 0;
}