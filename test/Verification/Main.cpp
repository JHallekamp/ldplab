#include "UnitTests/RayTriangleIntersectionTest.hpp"
#include "UnitTests/RayAABBOverlapTest.hpp"
#include "UnitTests/SegmentTriangleIntersectionTest.hpp"

int main()
{
    ldplab::verification::RayTriangleIntersectionTest t1;
    t1.runTests();

    ldplab::verification::SegmentTriangleIntersectionTest t2;
    t2.runTests();

    ldplab::verification::RayAABBOverlapTest t3;
    t3.runTests();

    std::cin.get();
    return 0;
}