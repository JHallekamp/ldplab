#ifndef WWU_LDPLAB_PROPERTY_GENERATOR_HPP
#define WWU_LDPLAB_PROPERTY_GENERATOR_HPP

#include "..\..\Geometry.hpp"

namespace ldplab
{
    // Prototype
    struct RodParticleGeometry;
    struct Particle;

    Vec3 getRodParticleCenterOfMass(const RodParticleGeometry& geometry);
    Particle getRodParticle(
        const double A,
        const double l,
        const double kappa,
        const double np,
        const double nu,
        const Vec3 position,
        const Vec3 orientation);
}

#endif