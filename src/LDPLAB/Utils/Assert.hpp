#ifndef WWU_LDPLAB_UTILS_ASSERT_HPP
#define WWU_LDPLAB_UTILS_ASSERT_HPP

#ifndef LDPLAB_BUILD_OPTION_ENABLE_ASSERT
#   define LDPLAB_ASSERT(expression) 
#else // LDPLAB_BUILD_OPTION_ENABLE_ASSERT
#   ifdef NDEBUG
#       define NDEBUG
#       include <cassert>
#       define LDPLAB_ASSERT(expression) assert(expression)
#       undef NDEBUG
#   else // NDEBUG
#       include <cassert>
#       define LDPLAB_ASSERT(expression) assert(expression)
#   endif // NDEBUG
#endif // LDPLAB_BUILD_OPTION_ENABLE_ASSERT

#endif