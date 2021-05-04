#ifndef WWU_LDPLAB_VERIFICATON_ITEST_HPP
#define WWU_LDPLAB_VERIFICATON_ITEST_HPP

#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ldplab
{
    namespace verification
    {
        /** @brief Interface for generic testing. */
        class ITest
        {
        public:
            virtual ~ITest() { release(); }
            /** @brief Called to setup the test. */
            virtual bool setup() = 0;
            /** @brief Called to deinitialize the test. */
            virtual void release() = 0;
            /** 
             * @brief Called to run the tests. 
             * @returns A map containing the names of inidivdual tests and 
             *          their result.
             */
            virtual std::map<std::string, bool> runTests() = 0;
        };

        /** @brief Interface for unit tests. */
        template <typename InType, typename OutType>
        class IUnitTest : public ITest
        {
        public:
            virtual ~IUnitTest() { }
            /** @brief Inherited via ldplab::verification::ITest. */
            std::map<std::string, bool> runTests() override
            {
                std::vector<std::tuple<std::string, InType, OutType>> tests =
                    getTests();
                std::map<std::string, bool> results;
                for (size_t i = 0; i < tests; ++i)
                {
                    const std::string& test_name = tests[i][0];
                    const InType& input = tests[i][1];
                    const OutType& expected_output = tests[i][2];
                    const OutType result = getResult(input);
                    const double epsilon = getEpsilon(input);
                    const double error = getError(expected_output, result);
                    const bool has_passed = (error <= epsilon);
                    std::cout << "test " << test_name << ": ";
                    if (has_passed)
                        std::cout << "test " << test_name << ": passed ";
                    else
                        std::cout << "test " << test_name << ": failed ";
                    std::cout << "(error = " << error << ")" << std::endl;
                    results[test_name] = has_passed;
                }
                return results;
            }
        protected:
            /**
             * @brief Returns a vector containing tests.
             * @detials Each test consists of a triple. The first element is a
             *          string containing the test name. The second is an 
             *          InType instance, which contains the test input.
             *          The third element contains an OutType instance, that
             *          is the expected result.
             */
            virtual std::vector<std::tuple<std::string, InType, OutType>> 
                getTests() = 0;
            /** @brief Computes the OutType value for an InType instance. */
            virtual OutType getResult(const InType&) = 0;
            /** 
             * @brief Calculates the error between an expected and a resulting
             *        OutType instance.
             */
            virtual double getError(
                const OutType& expected, 
                const OutType& result)
            { return (expected == result ? 0 : 1); }
            /**
             * @brief Returns an epsilon for when resulting values are still
             *        considered near enough to the expected value, so that
             *        the test passes.
             * @details The epsilon can depend on the InType value.
             */
            virtual double getEpsilon(const InType& value)
            { return 0; }
        };
    }
}

#endif