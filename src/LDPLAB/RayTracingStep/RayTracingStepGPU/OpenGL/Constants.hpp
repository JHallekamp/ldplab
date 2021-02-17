#ifndef WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP
#define WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP

#include <cstdint>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        namespace constant
        {
            /** @brief The local work group size of the glsl shaders. */
            constexpr size_t glsl_local_group_size = 64;
        }
    }
}

#endif