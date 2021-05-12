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
            const real_t R,
            const real_t l,
            const real_t kappa,
            const real_t np,
            const real_t nu,
            const real_t gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getRodParticleConstVolume(
            const real_t V,
            const real_t l,
            const real_t kappa,
            const real_t np,
            const real_t nu,
            const real_t gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getRodParticle(
            const real_t R,
            const real_t L,
            const real_t kappa,
            const real_t np,
            const real_t nu,
            const real_t gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getSphereParticleByRadius(
            const real_t V,
            const real_t np,
            const real_t nu,
            const real_t gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
        static Particle getSphereParticleByVolume(
            const real_t R,
            const real_t np,
            const real_t nu,
            const real_t gradient_direction,
            const Vec3 position,
            const Vec3 orientation);
    };
}

#endif