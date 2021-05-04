#ifndef WWU_LDPLAB_VERIFICATON_UNIT_TESTS_SEGMENT_TRIANGLE_INTERSECTION_TEST_HPP
#define WWU_LDPLAB_VERIFICATON_UNIT_TESTS_SEGMENT_TRIANGLE_INTERSECTION_TEST_HPP

#include "IntersectionCommon.hpp"
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace verification
    {
        /** @brief Unit test input for ray triangle intersection unit test. */
        struct SegmentTriangleIntersectionTestInput
        {
            Vec3 seg_ori;
            Vec3 seg_end;
            Triangle tri;
        };

        /** @brief Unit test ray triangle intersection tests. */
        class SegmentTriangleIntersectionTest :
            public IIntersectionUnitTest<SegmentTriangleIntersectionTestInput>
        {
        public:
            bool setup() override { return true; }
            void release() override { }
        protected:
            std::string input2String(
                const SegmentTriangleIntersectionTestInput&) override;
            IntersectionOutput getResult(
                const SegmentTriangleIntersectionTestInput&);
            std::vector<std::tuple<
                std::string,
                SegmentTriangleIntersectionTestInput,
                IntersectionOutput>>
                getTests() override;
            double getEpsilon(
                const SegmentTriangleIntersectionTestInput&) override;
        };
    }
}

#endif