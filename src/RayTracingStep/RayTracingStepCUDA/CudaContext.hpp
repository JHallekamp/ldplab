#ifndef WWU_LDPLAB_RTSCUDA_CUDA_CONTEXT_HPP
#define WWU_LDPLAB_RTSCUDA_CUDA_CONTEXT_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <LDPLAB/SimulationState.hpp>
#include <memory>

#include "CudaData.hpp"
#include "CudaResource.hpp"
#include "Data.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Holds any resources acquired for the use of CUDA. */
        struct CudaContext
        {
            /** @brief Creates the context and uploads neccessary data. */
            bool create(
                const RayTracingStepCUDAInfo& info,
                const ExperimentalSetup& setup);
            /** @brief Updates the context and uploads neccessary data. */
            bool update(const SimulationState& input);
            /** @brief Ray buffer storage resources. */
            struct
            {
                /** @brief 2D array holding index arrays per buffer. */
                CudaPitchedPtr<int32_t> index_buffers;
                /** @brief 2D array of origin vector arrays per buffers. */
                CudaPitchedPtr<Vec3> origin_buffers;
                /** @brief 2D array of direction vector arrays per buffer. */
                CudaPitchedPtr<Vec3> direction_buffers;
                /** @brief 2D array of intensity arrays per buffer. */
                CudaPitchedPtr<double> intensity_buffers;
                /** 
                 * @brief 2D array of minimum bounding volume distance arrays
                 *        per buffer.
                 */
                CudaPitchedPtr<double> min_bounding_volume_distance_buffers;
            } ray_buffer_data;
            /** @brief Intersection buffer storage resources. */
            struct
            {
                /** @brief Array of particle indices. */
                CudaLinearArray<int32_t> indices;
                /** @brief Array of intersection points. */
                CudaLinearArray<Vec3> intersection_points;
                /** @brief Array of intersection normals. */
                CudaLinearArray<Vec3> intersection_normal;
            } intersection_buffer_data;
            /** @brief Output buffer storage resources. */
            struct
            {
                /** @brief Array containing force vectors per particle. */
                CudaLinearArray<Vec3> force_per_particle;
                /** @brief Array containing torque vectors per particle. */
                CudaLinearArray<Vec3> torque_per_particle;
            } output_buffer_data;
            /** @brief Transformation data. */
            struct
            {
                /** 
                 * @brief Array containing world to particle space 
                 *        translation vectors per particle. 
                 */
                CudaLinearArray<Vec3> w2p_translation_vectors;
                /**
                 * @brief Array containing world to particle space
                 *        rotation and scale matrices per particle.
                 */
                CudaLinearArray<Mat3> w2p_rotation_scale_matrices;
                /**
                 * @brief Array containing particle to world space
                 *        translation vectors per particle.
                 */
                CudaLinearArray<Vec3> p2w_translation_vectors;
                /**
                * @brief Array containing particle to world space
                *        scale and rotation matrices per particle.
                */
                CudaLinearArray<Mat3> p2w_scale_rotation_matrices;
            } transformation_data;
            /** @brief Contains bounding volume data. */
            std::shared_ptr<IBoundingVolumeCudaData> bounding_volume_data;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_CUDA_CONTEXT_HPP