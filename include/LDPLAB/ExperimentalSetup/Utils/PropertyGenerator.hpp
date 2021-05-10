#ifndef WWU_LDPLAB_PROPERTY_GENERATOR_HPP
#define WWU_LDPLAB_PROPERTY_GENERATOR_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    // Prototype
    struct RodParticleGeometry;
    struct Particle;

    class PropertyGenerator
    {
    public:
        static Vec3 getRodParticleCenterOfMass(
            const RodParticleGeometry& geometry);
        static Particle getRodParticleConstRadius(
            const double R,
            const double l,
            const double kappa,
            const double np,
            const double nu,
            const double gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getRodParticleConstVolume(
            const double V,
            const double l,
            const double kappa,
            const double np,
            const double nu,
            const double gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getRodParticle(
            const double R,
            const double L,
            const double kappa,
            const double np,
            const double nu,
            const double gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getSphereParticleByRadius(
            const double V,
            const double np,
            const double nu,
            const double gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getSphereParticleByVolume(
            const double R,
            const double np,
            const double nu,
            const double gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
    };
}

#endif