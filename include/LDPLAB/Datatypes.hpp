#ifndef WWU_LDPLAB_DATATYPES_HPP
#define WWU_LDPLAB_DATATYPES_HPP

#include <glm/glm.hpp>

namespace ldplab
{
#ifdef LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION
    /** @brief Internal floating point type. */
    typedef float real_t;
    /** @brief Structure modeling a 2 dimensional vector. */
    typedef glm::vec2 Vec2;
    /** @brief Structure modeling a 3 dimensional vector. */
    typedef glm::vec3 Vec3;
    /** @brief Structure modeling a 4 dimensional vector. */
    typedef glm::vec4 Vec4;
    /** @brief Structure modeling a 3x3 matrix. */
    typedef glm::mat3 Mat3;
    /** @brief Structure modeling a 4x4 matrix. */
    typedef glm::mat4 Mat4;
#else
    /** @brief Internal floating point type. */
    typedef double real_t;
    /** @brief Structure modeling a 2 dimensional vector. */
    typedef glm::dvec2 Vec2;
    /** @brief Structure modeling a 3 dimensional vector. */
    typedef glm::dvec3 Vec3;
    /** @brief Structure modeling a 4 dimensional vector. */
    typedef glm::dvec4 Vec4;
    /** @brief Structure modeling a 3x3 matrix. */
    typedef glm::dmat3 Mat3;
    /** @brief Structure modeling a 4x4 matrix. */
    typedef glm::dmat4 Mat4;
#endif // LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION
}

#endif