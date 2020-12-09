#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include "RayTracingStepCPU.hpp"
#include "Data.hpp"

#include "..\..\Geometry.hpp"
#include "..\..\ExperimentalSetup\Particle.hpp"

#include <mutex>

namespace ldplab
{
    namespace rtscpu
    {
        class Context
        {
            friend class RayTracingStepFactory;
        public:
            // Initial ray buffer
            void initialRayBufferReset(size_t num_rays);
            RayBuffer initialRayBufferGetBuffer();
        private:
            std::vector<Particle> m_particles;
            // Configuration
            size_t m_ray_buffer_size;
            // Ray buffers
            std::vector<Ray> m_initial_rays;
            size_t m_initial_rays_offset;
            size_t m_initial_rays_count;
            std::vector<Ray> m_batch_rays;
            size_t m_batch_rays_offset;
            size_t m_batch_rays_count;
            std::mutex m_ray_buffer_queue_mutex;
        };
    }
}

#endif