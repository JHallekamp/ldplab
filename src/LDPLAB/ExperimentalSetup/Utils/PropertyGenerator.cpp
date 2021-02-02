#include "PropertyGenerator.hpp"

#include "..\..\Geometry.hpp"
#include "..\ParticleGeometry.hpp"


ldplab::Vec3 ldplab::getRodParticleCenterOfMass(
    const ldplab::RodParticleGeometry& geometry)
{
    Vec3 rs(0, 0, 0);
    rs.z = -(geometry.cylinder_length / 2) +
        (geometry.cylinder_length * geometry.cylinder_length * geometry.kappa) /
        geometry.cylinder_radius +
        (geometry.cylinder_radius * geometry.kappa) / 2 +
        (geometry.cylinder_length * geometry.kappa * geometry.kappa) / 2 -
        (geometry.cylinder_radius * geometry.kappa * geometry.kappa *
            geometry.kappa) / 6;
    return rs;
}