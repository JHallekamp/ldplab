#ifndef WWU_LDPLAB_RTSCUDA_CUDA_DATA_HPP
#define WWU_LDPLAB_RTSCUDA_CUDA_DATA_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/Geometry.hpp>
#include "CudaResource.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /**
         * @brief Interface for structures containing data of bounding
         *        volumes.
         */
        struct IBoundingVolumeCudaData
        {
            virtual ~IBoundingVolumeCudaData() { }
            /** @brief The type of the bounding volume data. */
            enum class Type { spheres };
            /** @brief Returns the type of the bounding volumes. */
            virtual Type type() const = 0;
        };

        /** @brief Contains the data of the transformed bounding spheres. */
        struct BoundingSphereCudaData : public IBoundingVolumeCudaData
        {
            Type type() const override { return Type::spheres; }
            /** @brief Array holding bounding sphere centers per particle. */
            CudaLinearArray<Vec3> bounding_sphere_centers;
            /** @brief Array of bounding sphere radii per particle. */
            CudaLinearArray<double> bounding_sphere_radii;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_CUDA_DATA_HPP