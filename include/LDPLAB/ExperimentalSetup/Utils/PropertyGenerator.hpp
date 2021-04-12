#ifndef WWU_LDPLAB_PROPERTY_GENERATOR_HPP
#define WWU_LDPLAB_PROPERTY_GENERATOR_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    // Prototype
    struct RodParticleGeometry;
    struct Particle;

    Vec3 getRodParticleCenterOfMass(const RodParticleGeometry& geometry);
    Particle getRodParticleConstArea(
        const double A,
        const double l,
        const double kappa,
        const double np,
        const double nu,
        const Vec3 position,
        const Vec3 orientation);

    Particle getRodParticleConstVolume(
        const double V,
        const double l,
        const double kappa,
        const double np,
        const double nu,
        const Vec3 position,
        const Vec3 orientation);

    Particle getRodParticle(
        const double R,
        const double L,
        const double kappa,
        const double np,
        const double nu,
        const Vec3 position,
        const Vec3 orientation);

    Particle getSphereParticle(
        const double V,
        const double np,
        const double nu,
        const Vec3 position,
        const Vec3 orientation);
}

#endif