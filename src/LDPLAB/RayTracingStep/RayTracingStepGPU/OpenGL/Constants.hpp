#ifndef WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP
#define WWU_LDPLAB_RTSGPU_OGL_CONSTANTS_HPP

#include <cstdint>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        namespace constant
        {
            namespace glsl_local_group_size
            {
                /**
                 * @brief The local work group size of the glsl shader that 
                 *        resets the intersection and interaction buffers.
                 * @details The corresponding shader file is
                 *          CSResetOutputAndIntersectionBuffer.glsl
                 */
                constexpr size_t reset_output_and_intersection_buffer = 128;
                /**
                 * @brief The local work group size of the glsl shader that 
                 *        gathers the output data.
                 * @details The corresponding shader file is
                 *          CSGatherOutput.glsl
                 */
                constexpr size_t gather_output = 256;
                /**
                 * @brief The local work group size of the glsl shader that 
                 *        counts invalid and world space rays.
                 * @details The corresponding shader file is
                 *          CSCountRayBufferState.glsl
                 */
                constexpr size_t count_ray_buffer_state = 256;
                /**
                 * @brief The local work group size of the glsl shader that
                 *        tests the ray bounding sphere intersections.
                 * @details The corresponding shader file is
                 *          CSRayBoundingSphereIntersectionTestBruteForce.glsl
                 */
                constexpr size_t ray_bounding_sphere_intersection_test_brute_force = 128;
                /**
                 * @brief The local work group size of the glsl shader that
                 *        tests rod particle inner particle propagation.
                 * @details The corresponding shader file is
                 *          CSLinearIndexGradientRodParticlePropagation.glsl
                 */
                constexpr size_t linear_index_gradient_rod_particle_propagation = 64;
                /**
                 * @brief The local work group size of the glsl shader for the
                 *        rod particle intersection test.
                 * @details The corresponding shader file is
                 *          CSRodParticleIntersectionTest.glsl
                 */
                constexpr size_t rod_particle_intersection_test = 64;
                /**
                 * @brief The local work group size of the glsl shader for the
                 *        interaction of unpolarized light with 1d linear index
                 *        gradient particle material.
                 * @details The corresponding shader file is
                 *          CSUnpolarizedLight1DLinearIndexGradientInteraction.glsl
                 */
                constexpr size_t unpolarized_light_1d_linear_index_gradient_interaction = 128;
            }

            namespace glsl_shader_name
            {
                constexpr const char* count_ray_buffer_state =
                    "CSCountRayBufferState";
                constexpr const char* gather_output =
                    "CSGatherOutput";
                constexpr const char* linear_index_gradient_rod_particle_propagation =
                    "CSLinearIndexGradientRodParticlePropagation";
                constexpr const char* ray_bounding_sphere_intersection_test_brute_force =
                    "CSRayBoundingSphereIntersectionTestBruteForce";
                constexpr const char* reset_output_and_intersection_buffer =
                    "CSResetOutputAndIntersectionBuffer";
                constexpr const char* rod_particle_intersection_test =
                    "CSRodParticleIntersectionTest";
                constexpr const char* unpolarized_light_1d_linear_index_gradient_interaction =
                    "CSUnpolarizedLight1DLinearIndexGradientInteraction";
            }

            constexpr const char* glsl_shader_subdirectory = "glsl/";
            constexpr const char* glsl_shader_extension = ".glsl";
        }
    }
}

#endif