#ifndef WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP
#define WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP

#include <cstdint>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        namespace constant
        {
            namespace glsl_local_group_sizes
            {
                /**
                 * @brief The local work group size of the glsl shader that 
                 *        resets the intersection and interaction buffers.
                 * @details The corresponding shader file is
                 *          ldplab_cs_reset_output_and_intersection_buffer.glsl
                 */
                constexpr size_t reset_output_and_intersection_buffer = 128;
                /**
                 * @brief The local work group size of the glsl shader that
                 *        tests the ray bounding sphere intersections.
                 * @details The corresponding shader file is
                 *          ldplab_cs_ray_bounding_sphere_intersection_test_brute_force.glsl
                 */
                constexpr size_t ray_bounding_sphere_intersection_test_brute_force = 128;
                /**
                 * @brief The local work group size of the glsl shader that
                 *        tests rod particle inner particle propagation.
                 * @details The corresponding shader file is
                 *          ldplab_cs_linear_index_gradient_rod_particle_propagation.glsl
                 */
                constexpr size_t linear_index_gradient_rod_particle_propagation = 64;
                /**
                 * @brief The local work group size of the glsl shader for the
                 *        rod particle intersection test.
                 * @details The corresponding shader file is
                 *          ldplab_cs_rod_particle_intersection_test.glsl
                 */
                constexpr size_t rod_particle_intersection_test = 64;
                /**
                 * @brief The local work group size of the glsl shader for the
                 *        interaction of unpolarized light with 1d linear index
                 *        gradient particle material.
                 * @details The corresponding shader file is
                 *          ldplab_cs_unpolarized_light_1d_linear_index_gradient_interaction.glsl
                 */
                constexpr size_t unpolarized_light_1d_linear_index_gradient_interaction = 128;
            }
        }
    }
}

#endif