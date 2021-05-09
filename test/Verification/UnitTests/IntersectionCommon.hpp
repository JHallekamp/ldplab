#ifndef WWU_LDPLAB_VERIFICATON_UNIT_TESTS_INTERSECTION_COMMON_HPP
#define WWU_LDPLAB_VERIFICATON_UNIT_TESTS_INTERSECTION_COMMON_HPP

#include "../ITest.hpp"
#include <sstream>

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
            std::string output2String(const IntersectionOutput& o) override
            {
                std::stringstream ss;
                ss << "(";
                if (o.intersects)
                    ss << "isec = hit";
                else
                    ss << "isec = miss";
                ss << ", dist = " << o.intersection_distance << ")";
                return ss.str();
            }
            /** 
             * @brief Inherited via ldplab::verification::IUnitTest 
             * @details Uses distance between expected and resulting
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
                else if (result.intersects)
                {
                    const double absolute_error =
                        result.intersection_distance -
                        expectation.intersection_distance;
                    return absolute_error;
                }
                else
                    return 0;
            }
        };
    }
}

#endif