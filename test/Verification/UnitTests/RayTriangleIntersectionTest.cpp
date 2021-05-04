#include "RayTriangleIntersectionTest.hpp"
#include <RayTracingStep/RayTracingStepCPU/IntersectionTests.hpp>

std::string ldplab::verification::RayTriangleIntersectionTest::input2String(
    const RayTriangleIntersectionTestInput& i)
{
    std::stringstream ss;
    ss << "tri = (" <<
        "[" << i.tri.a.x << ", " << i.tri.a.y << ", " << i.tri.a.z << "], " <<
        "[" << i.tri.b.x << ", " << i.tri.b.y << ", " << i.tri.b.z << "], " <<
        "[" << i.tri.c.x << ", " << i.tri.c.y << ", " << i.tri.c.z << "]) - ";
    ss << "ray = (" <<
        "[" <<
        i.ray.origin.x << ", " <<
        i.ray.origin.y << ", " <<
        i.ray.origin.z << "], " <<
        "[" <<
        i.ray.direction.x << ", " <<
        i.ray.direction.y << ", " <<
        i.ray.direction.z << "])";
    return ss.str();
}

ldplab::verification::IntersectionOutput
    ldplab::verification::RayTriangleIntersectionTest::getResult(
        const RayTriangleIntersectionTestInput& input)
{
    IntersectionOutput output;
    output.intersects = rtscpu::IntersectionTest::intersectRayTriangle(
        input.ray,
        input.tri,
        output.intersection_distance);
    return output;
}

std::vector<std::tuple<
    std::string, 
    ldplab::verification::RayTriangleIntersectionTestInput, 
    ldplab::verification::IntersectionOutput>> 
    ldplab::verification::RayTriangleIntersectionTest::getTests()
{
    std::vector<std::tuple<
        std::string,
        RayTriangleIntersectionTestInput,
        IntersectionOutput>> tests;

    Triangle t;
    Ray r;
    r.intensity = 0;

    t.a = Vec3(0, 0, 0);
    t.b = Vec3(1, 0, 0);
    t.c = Vec3(0, 1, 0);
    r.origin = Vec3(0, 0, 0.5);
    r.direction = glm::normalize(Vec3(0, 0, -1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-01",
        RayTriangleIntersectionTestInput { r, t },
        IntersectionOutput{ true, 0.5 }
    ));

    r.origin = Vec3(0, 0, -0.5);
    r.direction = glm::normalize(Vec3(0, 0, 1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-02",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ true, 0.5 }
    ));

    r.origin = Vec3(0, 0, 0);
    r.direction = glm::normalize(Vec3(1, 1, 0));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-03",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ false, 0 }
    ));

    r.origin = Vec3(0.1, 0.1, 0.1);
    r.direction = glm::normalize(Vec3(0, 1, -1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-04",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ true, 0.141421356 }
    ));

    r.origin = Vec3(0.1, 0.1, 0);
    r.direction = glm::normalize(Vec3(0, 1, -1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-05",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ false, 0 }
    ));

    t.a = Vec3(0, 0, 1);
    t.b = Vec3(1, 0, 0);
    t.c = Vec3(0, 1, 0);
    r.origin = Vec3(0, 0, 0);
    r.direction = glm::normalize(Vec3(1, 1, 1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-06",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ true, std::sqrt(3.0) / 3.0 }
    ));

    r.origin = Vec3(0, 0, 0);
    r.direction = glm::normalize(Vec3(0, 0, 1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-07",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ true, 1 }
    ));

    r.origin = Vec3(0, 0, 0);
    r.direction = glm::normalize(Vec3(-1, -1, 1));
    tests.emplace_back(std::make_tuple(
        "ray-triangle-intersection-08",
        RayTriangleIntersectionTestInput{ r, t },
        IntersectionOutput{ false, 0 }
    ));

    return tests;
}

double ldplab::verification::RayTriangleIntersectionTest::getEpsilon(
    const RayTriangleIntersectionTestInput& value)
{
    //const double d1 = glm::length(value.ray.origin - value.tri.a);
    //const double d2 = glm::length(value.ray.origin - value.tri.b);
    //const double d3 = glm::length(value.ray.origin - value.tri.c);
    //const double mxd = std::max(d1, std::max(d2, d3));

    // Return arbitrarily selected constant for now
    return 1e-7;
}
