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
                constexpr size_t count_ray_buffer_state_pre_stage = 256;
                constexpr size_t count_ray_buffer_state_post_stage = 256;
                constexpr size_t count_ray_buffer_state_reduction_stage = 256;
                constexpr size_t gather_output_pre_stage = 256;
                constexpr size_t gather_output_post_stage = 256;
                constexpr size_t gather_output_reduction_stage = 256;
                constexpr size_t linear_index_gradient_rod_particle_propagation = 64;
                constexpr size_t ray_bounding_sphere_intersection_test_brute_force = 128;
                constexpr size_t reset_output_and_intersection_buffer = 128;
                constexpr size_t rod_particle_intersection_test = 64;
                constexpr size_t unpolarized_light_1d_linear_index_gradient_interaction = 128;
            }

            namespace glsl_shader_name
            {
                constexpr const char* count_ray_buffer_state_pre_stage =
                    "CSCountRayBufferStatePreStage";
                constexpr const char* count_ray_buffer_state_post_stage =
                    "CSCountRayBufferStatePostStage";
                constexpr const char* count_ray_buffer_state_reduction_stage =
                    "CSCountRayBufferStateReduceStage";
                constexpr const char* gather_output_pre_stage =
                    "CSGatherOutputPreStage";
                constexpr const char* gather_output_post_stage =
                    "CSGatherOutputPostStage";
                constexpr const char* gather_output_reduction_stage =
                    "CSGatherOutputReduceStage";
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