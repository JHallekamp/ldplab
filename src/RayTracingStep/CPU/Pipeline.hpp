#ifndef WWU_LDPLAB_RTSCPU_PIPELINE_HPP
#define WWU_LDPLAB_RTSCPU_PIPELINE_HPP

#include <LDPLAB/RayTracingStep/CPU/IBoundingVolumeIntersection.hpp>
#include <LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp>
#include <LDPLAB/RayTracingStep/CPU/IInitialStage.hpp>
#include <LDPLAB/RayTracingStep/CPU/IInnerParticlePropagation.hpp>
#include <LDPLAB/RayTracingStep/CPU/IParticleIntersection.hpp>
#include <LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepOutput.hpp>

#include "../../Utils/ThreadPool.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        // Prototype
        class MemoryControl;
        class RayTracingStepCPU;

        class Pipeline : public utils::ThreadPool::IJob
        {
            friend RayTracingStepCPU;
        public:
            Pipeline(
                const RayTracingStepCPU& owner,
                const RayTracingStepCPUInfo& info,
                const SimulationParameter& simulation_parameter,
                InterfaceMapping&& interface_mapping,
                std::unique_ptr<ExperimentalSetup>&& setup,
                std::vector<MemoryControl>&& memory_controls,
                std::vector<std::shared_ptr<IGenericGeometry>>&& geometries,
                std::unique_ptr<IBoundingVolumeIntersection>&& bvi,
                std::unique_ptr<IInitialStage>&& is,
                std::unique_ptr<IInnerParticlePropagation>&& ipp,
                std::unique_ptr<IParticleIntersection>&& pi,
                std::unique_ptr<ISurfaceInteraction>&& si);
            void stepSetup(const SimulationState& sim_state);
            void finalizeOutput(RayTracingStepOutput& output);
            /** @brief Inherited via IJob */
            void execute(size_t job_id, size_t batch_size) override;
        private:
            ldplab::Mat3 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order); 
            void processBatch(
                RayBuffer& buffer,
                MemoryControl& mem_control);
        private:
            const RayTracingStepCPU& m_owner;
            RayTracingStepCPUInfo m_info;
            SimulationParameter m_sim_params;
            InterfaceMapping m_interface_mapping;
            std::unique_ptr<ExperimentalSetup> m_setup;
            
            std::vector<MemoryControl> m_memory_controls;
            std::vector<std::shared_ptr<IGenericGeometry>> m_generic_geometries;
            std::vector<ParticleTransformation> m_particle_transformations;

            std::unique_ptr<IBoundingVolumeIntersection> m_stage_bvi;
            std::unique_ptr<IInitialStage> m_stage_is;
            std::unique_ptr<IInnerParticlePropagation> m_stage_ipp;
            std::unique_ptr<IParticleIntersection> m_stage_pi;
            std::unique_ptr<ISurfaceInteraction> m_stage_si;
        };
    }
}

#endif