#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL

#include "Data.hpp"
#include "Context.hpp"
#include "../../../Utils/Log.hpp"

void ldplab::rtsogl::RodParticleData::uploadSSBO()
{
    ssbo.rod_particles->upload(rod_particles_data.data());
}

void ldplab::rtsogl::ParticleMaterialLinearOneDirectionalData::uploadSSBO()
{
    ssbo.material->upload(material_data.data());
}

void ldplab::rtsogl::BoundingSphereData::uploadSSBO()
{
    ssbo.sphere_properties->upload(sphere_properties_data.data());
}

#endif