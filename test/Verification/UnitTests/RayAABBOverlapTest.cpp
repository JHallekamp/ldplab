#include "RayAABBOverlapTest.hpp"

#include <RayTracingStep/RayTracingStepCPU/IntersectionTests.hpp>

std::string ldplab::verification::RayAABBOverlapTest::input2String(
    const RayAABBOverlapTestInput& i)
{
    std::stringstream ss;
    ss << "ray = (" <<
        "[" <<
        i.ray.origin.x << ", " <<
        i.ray.origin.y << ", " <<
        i.ray.origin.z << "], " <<
        "[" <<
        i.ray.direction.x << ", " <<
        i.ray.direction.y << ", " <<
        i.ray.direction.z << "]) - ";
    ss << "aabb = (" <<
        "[" <<
        i.aabb.min.x << ", " <<
        i.aabb.min.y << ", " <<
        i.aabb.min.z << "], " <<
        "[" <<
        i.aabb.max.x << ", " <<
        i.aabb.max.y << ", " <<
        i.aabb.max.z << "])";
    return ss.str();
}

ldplab::verification::IntersectionOutput 
    ldplab::verification::RayAABBOverlapTest::getResult(
        const RayAABBOverlapTestInput& input)
{
    IntersectionOutput output;
    output.intersects = rtscpu::IntersectionTest::overlapRayAABB(
        input.ray, 
        input.aabb, 
        output.intersection_distance);
    return output;
}

std::vector<std::tuple<
    std::string, 
    ldplab::verification::RayAABBOverlapTestInput, 
    ldplab::verification::IntersectionOutput>> 
    ldplab::verification::RayAABBOverlapTest::getTests()
{
    std::vector<std::tuple<
        std::string,
        ldplab::verification::RayAABBOverlapTestInput,
        ldplab::verification::IntersectionOutput>> tests;
    
    AABB aabb;
    Ray ray;
    ray.intensity = 0;

    aabb.min = Vec3(0, 0, 0);
    aabb.max = Vec3(1, 1, 1);
    ray.origin = Vec3(0, 0, 0);
    ray.direction = glm::normalize(Vec3(1, 0, 1));
    tests.emplace_back(std::make_tuple(
        "ray-aabb-overlap-01",
        RayAABBOverlapTestInput{ ray, aabb },
        IntersectionOutput{ true, 0 }
    ));

    ray.origin = Vec3(0.5, 0.5, 0.5);
    ray.direction = glm::normalize(Vec3(1, 0, 0));
    tests.emplace_back(std::make_tuple(
        "ray-aabb-overlap-02",
        RayAABBOverlapTestInput{ ray, aabb },
        IntersectionOutput{ true, 0 }
    ));

    ray.origin = Vec3(-1, 0.5, 0.5);
    ray.direction = glm::normalize(Vec3(1, 0, 0));
    tests.emplace_back(std::make_tuple(
        "ray-aabb-overlap-03",
        RayAABBOverlapTestInput{ ray, aabb },
        IntersectionOutput{ true, 1 }
    ));

    ray.origin = Vec3(-1, 0, 0);
    ray.direction = glm::normalize(Vec3(1, 0, 0));
    tests.emplace_back(std::make_tuple(
        "ray-aabb-overlap-04",
        RayAABBOverlapTestInput{ ray, aabb },
        IntersectionOutput{ true, 1 }
    ));

    ray.origin = Vec3(-1, 0, -1e-10);
    ray.direction = glm::normalize(Vec3(1, 0, 0));
    tests.emplace_back(std::make_tuple(
        "ray-aabb-overlap-05",
        RayAABBOverlapTestInput{ ray, aabb },
        IntersectionOutput{ false, 0 }
    ));

    return tests;
}

double ldplab::verification::RayAABBOverlapTest::getEpsilon(
    const RayAABBOverlapTestInput&)
{
    return 1e-7;
}
