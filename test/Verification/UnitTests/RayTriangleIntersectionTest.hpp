#ifndef WWU_LDPLAB_VERIFICATON_UNIT_TESTS_RAY_TRIANGLE_INTERSECTION_TEST_HPP
#define WWU_LDPLAB_VERIFICATON_UNIT_TESTS_RAY_TRIANGLE_INTERSECTION_TEST_HPP

#include "IntersectionCommon.hpp"
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace verification
    {
        /** @brief Unit test input for ray triangle intersection unit test. */
        struct RayTriangleIntersectionTestInput
        {
            Ray ray;
            Triangle tri;
        };

        /** @brief Unit test ray triangle intersection tests. */
        class RayTriangleIntersectionTest : 
            public IIntersectionUnitTest<RayTriangleIntersectionTestInput>
        {
        public:
            bool setup() override { return true; }
            void release() override { }
        protected:
            std::string input2String(
                const RayTriangleIntersectionTestInput&) override;
            IntersectionOutput getResult(
                const RayTriangleIntersectionTestInput&);
            std::vector<std::tuple<
                std::string,
                RayTriangleIntersectionTestInput,
                IntersectionOutput>>
                getTests() override;
            double getEpsilon(
                const RayTriangleIntersectionTestInput&) override;
        };
    }
}

#endif