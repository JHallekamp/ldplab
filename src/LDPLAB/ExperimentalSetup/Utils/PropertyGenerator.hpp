#ifndef WWU_LDPLAB_PROPERTY_GENERATOR_HPP
#define WWU_LDPLAB_PROPERTY_GENERATOR_HPP

#include "..\..\Geometry.hpp"

namespace ldplab
{
    struct RodParticleGeometry;
    Vec3 getRodParticleCenterOfMass(const RodParticleGeometry& geometry);
}

#endif