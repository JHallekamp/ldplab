#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "GenericParticleMaterial.hpp"

#include "../../Utils/Log.hpp"

std::shared_ptr<ldplab::rtscuda::GenericParticleMaterial> 
    ldplab::rtscuda::GenericParticleMaterial::create(
        std::shared_ptr<IParticleMaterial> particle_material)
{
    std::shared_ptr<ldplab::rtscuda::GenericParticleMaterial> material;
    if (particle_material->type() == IParticleMaterial::Type::homogenous)
        material = std::make_shared<ParticleHomogeneousMaterial>();
    else if (particle_material->type() == IParticleMaterial::Type::linear_one_directional)
        material = std::make_shared<ParticleLinearOneDirectionalMaterial>();
    else
    {
        // Type not supported.
        LDPLAB_LOG_ERROR("RTSCUDA generic particle material: "
            "Could not create resource, unsupported material type");
        return nullptr;
    }
    // Allocate resource
    if (!material->allocate(particle_material))
        return nullptr;
    return material;
}

ldplab::rtscuda::GenericParticleMaterialData 
    ldplab::rtscuda::ParticleHomogeneousMaterial::getData()
{
    GenericParticleMaterialData data;
    data.type = GenericParticleMaterialData::TYPE_HOMOGENOUS;
    data.data = m_data.getResource();
    return data;
}

bool ldplab::rtscuda::ParticleHomogeneousMaterial::allocate(
    std::shared_ptr<IParticleMaterial> particle_material)
{
    // Allocate
    if (!m_data.allocate(1))
        return false;
    // Upload data
    ParticleMaterialHomogenous* material = 
        static_cast<ParticleMaterialHomogenous*>(particle_material.get());
    Data data;
    data.index_of_refraction = material->index_of_refraction;
    if (!m_data.upload(&data))
        return false;
    return true;
}

ldplab::rtscuda::GenericParticleMaterialData 
    ldplab::rtscuda::ParticleLinearOneDirectionalMaterial::getData()
{
    GenericParticleMaterialData data;
    data.type = GenericParticleMaterialData::TYPE_LINEAR_ONE_DIRECTIONAL;
    data.data = m_data.getResource();
    return data;
}

bool ldplab::rtscuda::ParticleLinearOneDirectionalMaterial::allocate(
    std::shared_ptr<IParticleMaterial> particle_material)
{
    // Allocate
    if (!m_data.allocate(1))
        return false;
    // Upload data
    ParticleMaterialLinearOneDirectional* material =
        static_cast<ParticleMaterialLinearOneDirectional*>(
            particle_material.get());
    Data data;
    data.index_of_refraction = material->index_of_refraction;
    data.gradient = material->gradient;
    data.origin = material->origin;
    data.direction = material->direction;
    if (!m_data.upload(&data))
        return false;
    return true;
}

#endif