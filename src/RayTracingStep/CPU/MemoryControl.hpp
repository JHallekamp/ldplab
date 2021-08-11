#ifndef WWU_LDPLAB_RTSCPU_MEMORY_CONTROL_HPP
#define WWU_LDPLAB_RTSCPU_MEMORY_CONTROL_HPP

#include <memory>
#include <vector>

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief The memory control is responsible for creating, assigning and
         *        destroying ray buffers.
         */
        class MemoryControl
        {
        public:
            struct StageDependentData
            {
                std::shared_ptr<void> bounding_volume_intersection_data;
                std::shared_ptr<void> initial_stage_data;
                std::shared_ptr<void> inner_particle_propagation_data;
                std::shared_ptr<void> particle_intersection_data;
                std::shared_ptr<void> surface_interaction_data;
            };
        public:
            MemoryInfo getMemoryInfo (
                const RayTracingStepCPUInfo& info, 
                size_t thread_idx);
            bool allocateBuffers(
                const RayTracingStepCPUInfo& info,
                const ExperimentalSetup& setup,
                StageDependentData&& stage_dependent_data);
            /**
             * @brief Provides the initial batch buffer (depth 0).
             * @returns The root buffer of the buffer tree.
             */
            RayBuffer& initialBuffer() 
            { return getRayBuffer(0); }
            OutputBuffer& getOutputBuffer() 
            { return m_output_buffer; }
            RayBuffer& getRayBuffer(size_t depth)
            { return m_ray_buffers[depth]; }
            IntersectionBuffer& getIntersectionBuffer(size_t depth)
            { return m_intersection_buffers[0]; }
            size_t getNumRayBuffers() 
            { return m_ray_buffers.size(); }
            size_t getNumIntersectionBuffers() 
            { return m_intersection_buffers.size(); }
            StageDependentData& getStageDependentData()
            { return m_stage_dependent_data; }
            void resetOutputBuffer();
        private:
            StageDependentData m_stage_dependent_data;

            std::vector<RayBuffer> m_ray_buffers;
            std::vector<Ray> m_ray_data;
            std::vector<int32_t> m_ray_index_data;
            std::vector<double> m_ray_min_bounding_sphere_distance_data;

            std::vector<IntersectionBuffer> m_intersection_buffers;
            std::vector<Vec3> m_intersection_point_data;
            std::vector<Vec3> m_intersection_normal_data;
            std::vector<int32_t> m_intersection_particle_index_data;

            OutputBuffer m_output_buffer;
            std::vector<Vec3> m_output_force_data;
            std::vector<Vec3> m_output_torque_data;
        };
    }
}

#endif