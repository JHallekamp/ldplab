#ifndef WWU_LDPLAB_RTSCPU_IMPL_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_INITIAL_STAGE_HPP

#include <LDPLAB/RayTracingStep/CPU/IInitialStage.hpp>

#include <mutex>

namespace ldplab
{
    namespace rtscpu
    {
        class InitialStageBoundingSpheresHomogenousLight :
            public IInitialStage
        {
        public:
            InitialStageBoundingSpheresHomogenousLight(
                std::vector<LightSource>&& light_sources,
                double number_rays_per_unit);
            void stepSetup(
                const ExperimentalSetup& setup,
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping,
                const std::vector<ParticleTransformation>& particle_transformation) override;
            bool execute(
                RayBuffer& initial_batch_buffer,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) override;
        private:
            struct Projection
            {
                Vec2 center;
                double radius;
                size_t light_index;
                double depth;
                // Pointers to overlapping projections
                std::vector<Projection*> overlaps;
            };
            enum class CreateRay
            {
                yes,
                no_overlap,
                no_outside_projection
            };
        private:
            bool projLightOverlap(
                const Vec2& center,
                const double radius,
                const LightSource& light_source) const;
            CreateRay hasToCreateRay(
                const Projection& projection,
                const LightSource& light_source) const;
            void advBatchCreationLight(size_t& li);
            void advBatchCreationParticle(size_t& pi);
        private:
            // Projections for each particle
            std::vector<std::vector<Projection>> m_projections_per_particle;
            std::vector<LightSource> m_light_sources;
            size_t m_batch_creation_particle_index;
            size_t m_batch_creation_light_index;
            bool m_batch_creation_particle_initialized;
            double m_rasterization_x;
            double m_rasterization_y;
            bool m_rasterization_up;
            bool m_rasterization_right;
            double m_rasterization_step_size;
            std::mutex m_mutex;
        };

        class InitialStageBoundingSpheresHomogenousPolarizedLight :
            public IInitialStage
        {
        public:
            struct Polarization
            {
                Vec4 stokes_parameter;
                Vec3 direction;
                bool is_particle_system;
            };
            struct PolarizationBuffer
            {
                PolarizationBuffer(size_t buffer_index, size_t size)
                    :
                    polarization_data{ nullptr },
                    buffer_index{ buffer_index },
                    size{ size }
                { }
                Polarization* polarization_data;
                const size_t buffer_index;
                const size_t size;
            };
            struct PolarizationData
            {
                std::vector<PolarizationBuffer> polarization_buffers;
                std::vector<Polarization> polarization_data;
            };
        public:
            InitialStageBoundingSpheresHomogenousPolarizedLight(
                std::vector<LightSource>&& light_sources,
                double number_rays_per_unit);
            void stepSetup(
                const ExperimentalSetup& setup,
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping,
                const std::vector<ParticleTransformation>& particle_transformation) override;
            bool execute(
                RayBuffer& initial_batch_buffer,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) override;
        private:
            struct Projection
            {
                Vec2 center;
                double radius;
                size_t light_index;
                double depth;
                // Pointers to overlapping projections
                std::vector<Projection*> overlaps;
            };
            enum class CreateRay
            {
                yes,
                no_overlap,
                no_outside_projection
            };
        private:
            bool projLightOverlap(
                const Vec2& center,
                const double radius,
                const LightSource& light_source) const;
            CreateRay hasToCreateRay(
                const Projection& projection,
                const LightSource& light_source) const;
            void advBatchCreationLight(size_t& li);
            void advBatchCreationParticle(size_t& pi);
        private:
            // Projections for each particle
            std::vector<std::vector<Projection>> m_projections_per_particle;
            std::vector<LightSource> m_light_sources;
            size_t m_batch_creation_particle_index;
            size_t m_batch_creation_light_index;
            bool m_batch_creation_particle_initialized;
            double m_rasterization_x;
            double m_rasterization_y;
            bool m_rasterization_up;
            bool m_rasterization_right;
            double m_rasterization_step_size;
            std::mutex m_mutex;
        };
    }
}

#endif