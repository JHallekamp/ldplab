#ifndef WWU_LDPLAB_VERIFICATON_UNIT_TESTS_INTERSECTION_COMMON_HPP
#define WWU_LDPLAB_VERIFICATON_UNIT_TESTS_INTERSECTION_COMMON_HPP

#include "../ITest.hpp"

namespace ldplab
{
    namespace verification
    {
        /** @brief Output structure for intersection tests. */
        struct IntersectionOutput
        {
            bool intersects;
            double intersection_distance;
        };

        /** @brief An intersection unit test. */
        template <typename IT>
        class IIntersectionUnitTest : public IUnitTest<IT, IntersectionOutput>
        {
        public:
            virtual ~IIntersectionUnitTest() { }
        protected:
            /** 
             * @brief Inherited via ldplab::verification::IUnitTest 
             * @details Uses squared distance between expected and resulting
             *          intersection distance as error measurement, if the
             *          expected boolean result of the intersection test 
             *          matches the real result.
             */
            double getError(
                const IntersectionOutput& expectation,
                const IntersectionOutput& result) override
            {
                if (expectation.intersects != result.intersects)
                    return 1000.0;
                else
                {
                    const double absolute_error =
                        result.intersection_distance -
                        expectation.intersection_distance;
                    return absolute_error * absolute_error;
                }
            }
        };
    }
}

#endif