#include "SharedShaders.hpp"

#include "Constants.hpp"
#include "Data.hpp"
#include "Context.hpp"
#include "../../../ExperimentalSetup/Particle.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/ComputeHelper.hpp"
#include "../../../Utils/Profiler.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>

ldplab::rtsgpu_ogl::SharedShaders::SharedShaders(
    std::shared_ptr<Context> ctx)
    :
    m_context{ ctx },
    m_cs_count_ray_buffer_state_post_stage{ },
    m_cs_count_ray_buffer_state_pre_stage{ },
    m_cs_count_ray_buffer_state_reduction_stage{ },
    m_cs_reset_output_and_intersection{ }
{
}

bool ldplab::rtsgpu_ogl::SharedShaders::createShaderByName(
    const char* name, std::shared_ptr<ComputeShader>& shader)
{
    shader = m_context->ogl->createComputeShaderFromFile(name,
        m_context->parameters.shader_base_directory + 
            constant::glsl_shader_subdirectory +
            name + 
            constant::glsl_shader_extension);
    return (shader != nullptr);
}

bool ldplab::rtsgpu_ogl::SharedShaders::initShaders()
{
    // Create shader
    if (!createShaderByName(constant::glsl_shader_name::count_ray_buffer_state_post_stage,
            m_cs_count_ray_buffer_state_post_stage.shader))
        return false;
    if (!createShaderByName(constant::glsl_shader_name::count_ray_buffer_state_pre_stage,
        m_cs_count_ray_buffer_state_pre_stage.shader))
        return false;
    if (!createShaderByName(constant::glsl_shader_name::count_ray_buffer_state_reduction_stage,
        m_cs_count_ray_buffer_state_reduction_stage.shader))
        return false;
    if (!createShaderByName(constant::glsl_shader_name::reset_output_and_intersection_buffer,
        m_cs_reset_output_and_intersection.shader))
        return false;

    // Create SSBOs
    m_cs_count_ray_buffer_state_post_stage.ssbo_output =
        m_context->ogl->createShaderStorageBuffer(sizeof(int32_t) * 2);
    m_cs_count_ray_buffer_state_pre_stage.ssbo_temp =
        m_context->ogl->createShaderStorageBuffer(sizeof(int32_t) * 2 *
            m_context->parameters.number_rays_per_buffer);
    if (m_cs_count_ray_buffer_state_post_stage.ssbo_output == nullptr ||
        m_cs_count_ray_buffer_state_pre_stage.ssbo_temp == nullptr)
        return false;

    // Compute work group size
    m_cs_count_ray_buffer_state_post_stage.num_work_groups = 
        ComputeHelper::getNumWorkGroups(
            m_context->parameters.number_rays_per_buffer,
            constant::glsl_local_group_size::count_ray_buffer_state_post_stage);
    m_cs_count_ray_buffer_state_pre_stage.num_work_groups =
        ComputeHelper::getNumWorkGroups(
            m_context->parameters.number_rays_per_buffer,
            constant::glsl_local_group_size::count_ray_buffer_state_pre_stage);
    m_cs_reset_output_and_intersection.num_work_groups =
        ComputeHelper::getNumWorkGroups(
            m_context->parameters.number_rays_per_buffer,
            constant::glsl_local_group_size::reset_output_and_intersection_buffer);

    // Get uniform locations
    m_cs_count_ray_buffer_state_pre_stage.uniform_num_particles =
        m_cs_count_ray_buffer_state_pre_stage.shader->getUniformLocation(
            "num_particles");
    m_cs_count_ray_buffer_state_pre_stage.uniform_num_rays_per_buffer =
        m_cs_count_ray_buffer_state_pre_stage.shader->getUniformLocation(
            "num_rays_per_buffer");

    m_cs_count_ray_buffer_state_reduction_stage.uniform_buffer_size =
        m_cs_count_ray_buffer_state_reduction_stage.shader->getUniformLocation(
            "buffer_size");
    m_cs_count_ray_buffer_state_reduction_stage.uniform_source_offset =
        m_cs_count_ray_buffer_state_reduction_stage.shader->getUniformLocation(
            "source_offset");

    m_cs_count_ray_buffer_state_pre_stage.shader->use();
    glUniform1ui(m_cs_count_ray_buffer_state_pre_stage.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));
    glUniform1ui(m_cs_count_ray_buffer_state_pre_stage.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));

    // Get uniform locations
    m_cs_reset_output_and_intersection.uniform_num_rays_per_buffer =
        m_cs_reset_output_and_intersection.shader->getUniformLocation("num_rays_per_buffer");

    m_cs_reset_output_and_intersection.shader->use();
    glUniform1ui(m_cs_reset_output_and_intersection.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));

    // Done
    return true;
}

void ldplab::rtsgpu_ogl::SharedShaders::updateRayBufferState(RayBuffer& buffer)
{
    // Pre stage
    LDPLAB_PROFILING_START(update_ray_buffer_state_pre_stage);
    m_cs_count_ray_buffer_state_pre_stage.shader->use();
    buffer.ssbo.particle_index->bindToIndex(0);
    m_cs_count_ray_buffer_state_pre_stage.ssbo_temp->bindToIndex(1);
    m_cs_count_ray_buffer_state_pre_stage.shader->execute(
        m_cs_count_ray_buffer_state_pre_stage.num_work_groups);
    LDPLAB_PROFILING_STOP(update_ray_buffer_state_pre_stage);

    // Reduction stage
    LDPLAB_PROFILING_START(update_ray_buffer_state_reduction_stage);
    m_cs_count_ray_buffer_state_reduction_stage.shader->use();
    m_cs_count_ray_buffer_state_pre_stage.ssbo_temp->bindToIndex(0);
    size_t num_threads = m_context->parameters.number_rays_per_buffer;
    GLuint buffer_size, source_offset;
    do
    {
        buffer_size = static_cast<GLuint>(num_threads);
        num_threads = num_threads / 2 + num_threads % 2;
        source_offset = static_cast<GLuint>(num_threads);
        glUniform1ui(
            m_cs_count_ray_buffer_state_reduction_stage.uniform_buffer_size,
            buffer_size);
        glUniform1ui(
            m_cs_count_ray_buffer_state_reduction_stage.uniform_source_offset,
            source_offset);
        m_cs_count_ray_buffer_state_reduction_stage.shader->execute(
            ComputeHelper::getNumWorkGroups(num_threads,
                constant::glsl_local_group_size::count_ray_buffer_state_reduction_stage));
    } while (num_threads > 1);
    LDPLAB_PROFILING_STOP(update_ray_buffer_state_reduction_stage);

    // Post stage
    LDPLAB_PROFILING_START(update_ray_buffer_state_post_stage);
    m_cs_count_ray_buffer_state_post_stage.shader->use();
    m_cs_count_ray_buffer_state_pre_stage.ssbo_temp->bindToIndex(0);
    m_cs_count_ray_buffer_state_post_stage.ssbo_output->bindToIndex(1);
    m_cs_count_ray_buffer_state_post_stage.shader->execute(
        m_cs_count_ray_buffer_state_post_stage.num_work_groups);
    LDPLAB_PROFILING_STOP(update_ray_buffer_state_post_stage);

    // Download data
    LDPLAB_PROFILING_START(update_ray_buffer_state_data_download);
    int32_t output_data[2] = { 0 };
    m_cs_count_ray_buffer_state_post_stage.ssbo_output->download(output_data);
    LDPLAB_PROFILING_STOP(update_ray_buffer_state_data_download);

    // Update ray buffer state
    buffer.active_rays = output_data[0];
    buffer.world_space_rays = output_data[1];
}

void ldplab::rtsgpu_ogl::SharedShaders::resetOutputAndIntersectionBuffer(
    OutputBuffer& output, IntersectionBuffer& intersection)
{
    // Bind shaders
    LDPLAB_PROFILING_START(reset_output_and_intersection_shader_binding);
    m_cs_reset_output_and_intersection.shader->use();
    LDPLAB_PROFILING_STOP(reset_output_and_intersection_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(reset_output_and_intersection_ssbo_binding);
    intersection.ssbo.particle_index->bindToIndex(0);
    output.ssbo.output_per_ray->bindToIndex(1);
    LDPLAB_PROFILING_STOP(reset_output_and_intersection_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(reset_output_and_intersection_shader_execution);
    m_cs_reset_output_and_intersection.shader->execute(
        m_cs_reset_output_and_intersection.num_work_groups);
    LDPLAB_PROFILING_STOP(reset_output_and_intersection_shader_execution);
}

void ldplab::rtsgpu_ogl::SharedShaders::uploadRayBufferData(RayBuffer& buffer)
{
    // Upload data
    LDPLAB_PROFILING_START(ray_buffer_data_upload);
    buffer.ssbo.particle_index->upload(buffer.particle_index_data);
    buffer.ssbo.ray_properties->upload(buffer.ray_properties_data);
    LDPLAB_PROFILING_STOP(ray_buffer_data_upload);
}
