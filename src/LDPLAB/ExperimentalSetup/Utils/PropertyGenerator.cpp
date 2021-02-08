#include "PropertyGenerator.hpp"

#include "..\..\Geometry.hpp"
#include "..\Particle.hpp"

#include <cmath>

constexpr double const_pi()
{
    return 3.14159265358979323846264338327950288419716939937510;
}

ldplab::Vec3 ldplab::getRodParticleCenterOfMass(
    const ldplab::RodParticleGeometry& geometry)
{
    Vec3 rs(0, 0, 0);
    rs.z = geometry.cylinder_length / 2 +
        (geometry.cylinder_radius * geometry.kappa) / 2 +
        (geometry.cylinder_radius * geometry.kappa * geometry.kappa *
            geometry.kappa) / 6;
    return rs;
}

ldplab::Particle ldplab::getRodParticle(
    const double A,
    const double l,
    const double kappa,
    const double np,
    const double nu,
    const Vec3 position,
    const Vec3 orientation)
{
    const double R = std::sqrt(A / (2 * const_pi() * ((1 + kappa) + 2 * l)));
    const double L = l * 2 * R;
    const double h = kappa * R;
    const double delta_n = nu / (L + h) / 2;
    const double bounding_sphere_radius = 
        std::sqrt(std::pow((L + h) / 2, 2.0) + R * R);

    ldplab::Particle particle;
    particle.position = position;
    particle.orientation = orientation;
    particle.geometry =
        std::make_shared<ldplab::RodParticleGeometry>(
            R,
            L,
            kappa);
    particle.bounding_volume =
        std::make_shared<ldplab::BoundingVolumeSphere>(
            Vec3(0, 0, (L+h)/2),
            bounding_sphere_radius);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            delta_n,
            Vec3(0, 0, (L+h)/2),
            Vec3(0, 0, 1));
    particle.centre_of_mass = ldplab::getRodParticleCenterOfMass(
        *(ldplab::RodParticleGeometry*)particle.geometry.get());

    return particle;
}


