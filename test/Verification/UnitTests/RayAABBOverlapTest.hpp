#ifndef WWU_LDPLAB_VERIFICATON_UNIT_TESTS_RAY_AABB_OVERLAP_TEST_HPP
#define WWU_LDPLAB_VERIFICATON_UNIT_TESTS_RAY_AABB_OVERLAP_TEST_HPP

#include "IntersectionCommon.hpp"
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace verification
    {
        struct RayAABBOverlapTestInput
        {
            Ray ray;
            AABB aabb;
        };

        class RayAABBOverlapTest :
            public IIntersectionUnitTest<RayAABBOverlapTestInput>
        {
        public:
            bool setup() override { return true; }
            void release() override { }
        protected:
            std::string input2String(const RayAABBOverlapTestInput&) override;
            IntersectionOutput getResult(
                const RayAABBOverlapTestInput&) override;
            std::vector<std::tuple<
                std::string,
                RayAABBOverlapTestInput,
                IntersectionOutput>>
                getTests() override;
            double getEpsilon(const RayAABBOverlapTestInput&) override;
        };
    }
}

#endif