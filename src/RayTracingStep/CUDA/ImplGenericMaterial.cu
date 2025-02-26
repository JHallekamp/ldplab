#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplGenericMaterial.hpp"

#include <LDPLAB/RayTracingStep/CUDA/DeviceResource.hpp>

namespace linear_one_directional
{
    using namespace ldplab;
    using namespace rtscuda;
    __device__ double indexOfRefraction(
        const Vec3& position,
        const void* particle_material);
    __device__ IGenericMaterial::indexOfRefraction indexOfRefractionFp = indexOfRefraction;
}

ldplab::rtscuda::GenericMaterialLinearOneDirectional::
GenericMaterialLinearOneDirectional(
    DeviceBuffer<MaterialLinearOneDirectionalData>&& material_data)
    :
    m_material_data{ std::move(material_data) }
{ }

void* ldplab::rtscuda::GenericMaterialLinearOneDirectional::getDeviceData()
{
    return m_material_data.getDeviceBuffer();
}

ldplab::rtscuda::IGenericMaterial::indexOfRefraction
ldplab::rtscuda::GenericMaterialLinearOneDirectional::
getDeviceIndexOfRefractionFunction()
{
    using namespace linear_one_directional;
    indexOfRefraction kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        indexOfRefractionFp,
        sizeof(indexOfRefractionFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

__device__ double linear_one_directional::indexOfRefraction(
    const Vec3& position, 
    const void* particle_material)
{
    MaterialLinearOneDirectionalData mt =
        *static_cast<const MaterialLinearOneDirectionalData*>(particle_material);
    return mt.index_of_refraction_minus_partial_dot +
        glm::dot(mt.direction_times_gradient, position);
}

#endif
