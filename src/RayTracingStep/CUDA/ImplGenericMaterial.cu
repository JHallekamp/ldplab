#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplGenericMaterial.hpp"

namespace linear_one_directional
{
    using namespace ldplab;
    using namespace rtscuda;
    double indexOfRefraction(
        const Vec3& position,
        const void* particle_material);
    IGenericMaterial::indexOfRefraction indexOfRefractionFp = indexOfRefraction;
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
    indexOfRefraction kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        linear_one_directional::indexOfRefractionFp,
        sizeof(linear_one_directional::indexOfRefractionFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

double linear_one_directional::indexOfRefraction(
    const Vec3& position, 
    const void* particle_material)
{
    MaterialLinearOneDirectionalData mt =
        *static_cast<const MaterialLinearOneDirectionalData*>(particle_material);
    return mt.index_of_refraction + mt.gradient *
        glm::dot(mt.direction, (position - mt.origin));
}

#endif
