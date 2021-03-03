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
            /** 
             * @brief Executes ray buffer state update shader.
             * @details Performs a check of the buffer index i against the 
             *          the given threshold value t. If i >= t, then the count
             *          is increased.
             * @returns The final counter state.
             */
            void countRayBufferIndexState(
                size_t threshold1,
                size_t threshold2,
                RayBuffer& buffer, 
                size_t& count1,
                size_t& count2);
            /**
             * @brief Executes ray buffer state update shader.
             * @details Performs a check of the buffer index i against the
             *          the given threshold value t. If i >= t, then the count
             *          is increased.
             * @returns The final counter state.
             */
            void countRayBufferIndexState(
                size_t threshold,
                RayBuffer& buffer1, 
                RayBuffer& buffer2,
                size_t& count1,
                size_t& count2);
            /** @brief Executes reset output and intersection buffer shader. */
            void resetOutputAndIntersectionBuffer(
                OutputBuffer& output, IntersectionBuffer& intersection);
            /** @brief Uploads ray buffer data. */
            void uploadRayBufferData(RayBuffer& buffer);
        private:
            /** Counts buffer indices - after pre stage has been executed. */
            void countBufferIndexState();
        private:
            /** @brief Contains count ray buffer shader data. */
            struct CountRayBufferStateShaderPreStage {
                std::shared_ptr<ComputeShader> shader;
                std::shared_ptr<ShaderStorageBuffer> ssbo_temp;
                GLint uniform_num_rays_per_buffer;
                GLint uniform_second_ray_buffer;
                GLint uniform_threshold1;
                GLint uniform_threshold2;
                size_t num_work_groups;
            } m_cs_count_ray_buffer_state_pre_stage;
            /** @brief Contains count ray buffer shader data. */
            struct CountRayBufferStateShaderPostStage {
                std::shared_ptr<ComputeShader> shader;
                std::shared_ptr<ShaderStorageBuffer> ssbo_output;
                int32_t output_buffer[2];
                size_t num_work_groups;
            } m_cs_count_ray_buffer_state_post_stage;
            /** @brief Contains count ray buffer shader data. */
            struct CountRayBufferStateShaderReductionStage {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_source_offset;
                GLint uniform_buffer_size;
            } m_cs_count_ray_buffer_state_reduction_stage;
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