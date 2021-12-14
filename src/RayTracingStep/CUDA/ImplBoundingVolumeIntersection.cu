#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplBoundingVolumeIntersection.hpp"

#include <LDPLAB/Constants.hpp>

#include "IntersectionTests.hpp"

namespace sphere_brutforce
{
    __global__ void bvIntersectionKernel(
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        double* ray_min_bv_dist_buffer,
        size_t num_rays_per_batch,
        ldplab::rtscuda::BoundingSphere* bounding_volumes,
        ldplab::Mat3* w2p_transformation,
        ldplab::Vec3* w2p_translation,
        size_t num_particles);
}

ldplab::rtscuda::BoundingSphere::BoundingSphere()
    :
    center{ Vec3(0, 0, 0) },
    radius{ 0 }
{ }

ldplab::rtscuda::BoundingSphere::BoundingSphere(
    const BoundingVolumeSphere& cpy)
    :
    center{ cpy.center },
    radius{ cpy.radius }
{ }

ldplab::rtscuda::BoundingSphereIntersectionBruteforce::
    BoundingSphereIntersectionBruteforce(
        std::vector<DeviceBuffer<BoundingSphere>>&& bounding_spheres_per_device)
    :
    m_bounding_spheres_per_device{ std::move(bounding_spheres_per_device) }
{ }

void ldplab::rtscuda::BoundingSphereIntersectionBruteforce::stepSetup(
    const SimulationState& simulation_state,
    SharedStepData& shared_data,
    DeviceContext& device_context)
{
    const size_t device_id = device_context.groupId();
    if (device_id != 0)
    {
        // Reuse data on device 0
        m_bounding_spheres_per_device[device_id].uploadExt(
            m_bounding_spheres_per_device[0].getHostBuffer());
    }
    else
    {
        BoundingSphere* spheres = 
            m_bounding_spheres_per_device[device_id].getHostBuffer();
        for (size_t i = 0; i < shared_data.experimental_setup.particles.size(); ++i)
        {
            // Get the particle instance for particle i using the interface mapping
            const ParticleInstance& particle_instance =
                simulation_state.particle_instances.find(
                    shared_data.interface_mapping.particle_index_to_uid.at(i))->second;
            // Get the bounding sphere in pspace
            spheres[i] = *static_cast<BoundingVolumeSphere*>(
                shared_data.experimental_setup.particles[i].bounding_volume.get());
            // Translate bounding volume center to world space
            const auto& p2w_transformation =
                shared_data.per_device_data[device_id].particle_transformation_buffers.
                p2w_transformation_buffer.getHostBuffer()[i];
            const auto& p2w_translation =
                shared_data.per_device_data[device_id].particle_transformation_buffers.
                p2w_translation_buffer.getHostBuffer()[i];
            spheres[i].center = 
                p2w_transformation * spheres[i].center + p2w_translation;
        }
        // Upload
        m_bounding_spheres_per_device[device_id].upload();
    }    
}

void ldplab::rtscuda::BoundingSphereIntersectionBruteforce::execute(
    StreamContext& smctx,
    size_t ray_buffer_index,
    size_t num_rays)
{
    using namespace sphere_brutforce;
    const size_t block_size = 128;
    const size_t grid_size = num_rays / block_size + (num_rays % block_size ? 1 : 0);
    bvIntersectionKernel<<<grid_size, block_size, 0, smctx.cudaStream()>>>(
        smctx.rayDataBuffers().particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().origin_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().direction_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(ray_buffer_index),
        num_rays,
        m_bounding_spheres_per_device[smctx.deviceGroup()].getDeviceBuffer(),
        smctx.particleTransformationBuffers().w2p_transformation_buffer.getDeviceBuffer(),
        smctx.particleTransformationBuffers().w2p_translation_buffer.getDeviceBuffer(),
        smctx.simulationParameter().num_particles);
}

__global__ void sphere_brutforce::bvIntersectionKernel(
    int32_t* ray_index_buffer,
    ldplab::Vec3* ray_origin_buffer,
    ldplab::Vec3* ray_direction_buffer,
    double* ray_min_bv_dist_buffer,
    size_t num_rays_per_batch,
    ldplab::rtscuda::BoundingSphere* bounding_volumes,
    ldplab::Mat3* w2p_transformation,
    ldplab::Vec3* w2p_translation,
    size_t num_particles)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    // Check if the ray already is in a particle space or is invalid
    if (ray_index_buffer[ri] < static_cast<int32_t>(num_particles))
        return;
    double min_dist = -1.0;
    int32_t min_idx = -1;

    Vec3 ray_origin = ray_origin_buffer[ri];
    Vec3 ray_direction = ray_direction_buffer[ri];

    // Check each bounding volume sequentially for intersections
    for (size_t i = 0; i < num_particles; ++i)
    {
        BoundingSphere bsphere = bounding_volumes[i];
        double isec_dist_min, isec_dist_max;
        if (IntersectionTest::intersectRaySphere(
            ray_origin,
            ray_direction,
            bsphere.center,
            bsphere.radius,
            isec_dist_min,
            isec_dist_max))
        {
            if (isec_dist_min < 0)
                continue;
            if ((isec_dist_max < min_dist || min_dist < 0) &&
                isec_dist_max > ray_min_bv_dist_buffer[ri])
            {
                min_dist = isec_dist_max;
                min_idx = i;
            }
        }
    }
    // Check if the ray hits a particle bounding sphere
    if (min_idx >= 0)
    {
        // Ray hits particle with index min_idx
        ray_index_buffer[ri] = min_idx;
        ray_min_bv_dist_buffer[ri] = min_dist +
            constant::intersection_tests::epsilon;
        // Transform ray from world to particle space
        ray_origin_buffer[ri] = w2p_transformation[min_idx] *
            (ray_origin + w2p_translation[min_idx]);
        ray_direction_buffer[ri] = glm::normalize(
            w2p_transformation[min_idx] * ray_direction);
    }
    else
    {
        // Ray exits the scene
        ray_index_buffer[ri] = -1;
    }
}

#endif