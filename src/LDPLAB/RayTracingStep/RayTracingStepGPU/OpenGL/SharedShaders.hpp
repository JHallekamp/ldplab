#ifndef WWU_LDPLAB_RTSGPU_OGL_SHARED_SHADERS_HPP
#define WWU_LDPLAB_RTSGPU_OGL_SHARED_SHADERS_HPP

#include "Data.hpp"
#include "../../RayTracingStepGPUOpenGLInfo.hpp"

#include <memory>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /** 
         * @brief Contains shared shader ressources and common shader methods. 
         */
        class SharedShaders
        {
        public:
            SharedShaders(std::shared_ptr<Context> ctx);
            /** @brief Compiles a shader by name. */
            bool createShaderByName(const char* name,
                std::shared_ptr<ComputeShader>& shader);
            /** @brief Initializes shared shaders. */
            bool initShaders();
            /** @brief Executes ray buffer state update shader. */
            void updateRayBufferState(RayBuffer& buffer);
            /** @brief Executes reset output and intersection buffer shader. */
            void resetOutputAndIntersectionBuffer(
                OutputBuffer& output, IntersectionBuffer& intersection);
            /** @brief Uploads ray buffer data. */
            void uploadRayBufferData(RayBuffer& buffer);
        private:
            /** @brief Contains count ray buffer shader data. */
            struct CountRayBufferStateShader {
                std::shared_ptr<ComputeShader> shader;
                std::shared_ptr<ShaderStorageBuffer> ssbo_temp;
                std::shared_ptr<ShaderStorageBuffer> ssbo_output;
                GLint uniform_num_particles;
                GLint uniform_num_rays_per_buffer;
                size_t num_work_groups;
            } m_cs_count_ray_buffer_state;
            /** @brief Contains reset output and intersection buffer shader. */
            struct ResetOutputAndIntersectionShader {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_num_rays_per_buffer;
                size_t num_work_groups;
            } m_cs_reset_output_and_intersection;
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif