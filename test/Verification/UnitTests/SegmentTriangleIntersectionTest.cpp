#include "SegmentTriangleIntersectionTest.hpp"
#include <RayTracingStep/RayTracingStepCPU/IntersectionTests.hpp>

std::string 
    ldplab::verification::SegmentTriangleIntersectionTest::input2String(
        const SegmentTriangleIntersectionTestInput& i)
{
    std::stringstream ss;
    ss << "tri = (" <<
        "[" << i.tri.a.x << ", " << i.tri.a.y << ", " << i.tri.a.z << "], " <<
        "[" << i.tri.b.x << ", " << i.tri.b.y << ", " << i.tri.b.z << "], " <<
        "[" << i.tri.c.x << ", " << i.tri.c.y << ", " << i.tri.c.z << "]) - ";
    ss << "seg = (" <<
        "[" <<
        i.seg_ori.x << ", " <<
        i.seg_ori.y << ", " <<
        i.seg_ori.z << "], " <<
        "[" <<
        i.seg_end.x << ", " <<
        i.seg_end.y << ", " <<
        i.seg_end.z << "])";
    return ss.str();
}

ldplab::verification::IntersectionOutput 
    ldplab::verification::SegmentTriangleIntersectionTest::getResult(
        const SegmentTriangleIntersectionTestInput& input)
{
    IntersectionOutput output;
    output.intersects = rtscpu::IntersectionTest::intersectSegmentTriangle(
        input.seg_ori,
        input.seg_end,
        input.tri,
        output.intersection_distance);
    return output;
}

std::vector<std::tuple<
    std::string, 
    ldplab::verification::SegmentTriangleIntersectionTestInput, 
    ldplab::verification::IntersectionOutput>> 
    ldplab::verification::SegmentTriangleIntersectionTest::getTests()
{
    std::vector<std::tuple<
        std::string,
        ldplab::verification::SegmentTriangleIntersectionTestInput,
        ldplab::verification::IntersectionOutput>> tests;

    return tests;
}

double ldplab::verification::SegmentTriangleIntersectionTest::getEpsilon(
    const SegmentTriangleIntersectionTestInput&)
{
    // Right now, just return a const epsilon
    return 1e-7;
}
