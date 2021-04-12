#ifndef LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL

#include "Data.hpp"
#include "Context.hpp"
#include "../../../Utils/Log.hpp"

void ldplab::rtsgpu_ogl::RodParticleData::uploadSSBO()
{
    ssbo.rod_particles->upload(rod_particles_data.data());
}

void ldplab::rtsgpu_ogl::ParticleMaterialLinearOneDirectionalData::uploadSSBO()
{
    ssbo.material->upload(material_data.data());
}

void ldplab::rtsgpu_ogl::BoundingSphereData::uploadSSBO()
{
    ssbo.sphere_properties->upload(sphere_properties_data.data());
}

#endif