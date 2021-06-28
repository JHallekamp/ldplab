#include <LDPLAB/ExperimentalSetup/Utils/PropertyGenerator.hpp>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/ExperimentalSetup/Particle.hpp>

#include <cmath>

constexpr double const_pi()
{
    return 3.14159265358979323846264338327950288419716939937510;
}

ldplab::Vec3 ldplab::PropertyGenerator::getRodParticleCenterOfMass(
    const ldplab::RodParticleGeometry& geometry)
{
    Vec3 rs(0, 0, 0);
    rs.z = geometry.cylinder_length / 2 +
        (geometry.cylinder_radius * geometry.kappa) / 2 +
        (geometry.cylinder_radius * geometry.kappa * geometry.kappa *
            geometry.kappa) / 6;
    return rs;
}

ldplab::Vec3 ldplab::PropertyGenerator::getCapParticleCenterOfMass(const CapParticleGeometry& geometry)
{
    return Vec3(0,0,geometry.radius-geometry.cap_hight/2);
}

ldplab::Particle ldplab::PropertyGenerator::getRodParticleConstRadius(
    const double R,
    const double l,
    const double kappa,
    const double np,
    const double delta_n,
    const double gradient_direction,
    const Vec3 position,
    const Vec3 orientation)
{
    const double L = l * 2 * R;
    const double h = kappa * R;
    const double nu = delta_n / (L + h);
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
            nu,
            Vec3(0, 0, (L+h)/2),
            Vec3(0, 0, gradient_direction));
    particle.centre_of_mass = 
        ldplab::PropertyGenerator::getRodParticleCenterOfMass(
            *(ldplab::RodParticleGeometry*)particle.geometry.get());
    return particle;
}

ldplab::Particle ldplab::PropertyGenerator::getRodParticleConstVolume(
    const double V,
    const double l,
    const double kappa,
    const double np,
    const double delta_n,
    const double gradient_direction,
    const Vec3 position,
    const Vec3 orientation)
{
    const double R = std::pow(V/(l * 2.0 * const_pi()),1.0/3.0);
    const double L = l * 2 * R;
    const double h = kappa * R;
    const double nu = delta_n / (L + h);
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
            Vec3(0, 0, (L + h) / 2),
            bounding_sphere_radius);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            nu,
            Vec3(0, 0, (L + h) / 2),
            Vec3(0, 0 , gradient_direction));
    particle.centre_of_mass = 
        ldplab::PropertyGenerator::getRodParticleCenterOfMass(
            *(ldplab::RodParticleGeometry*)particle.geometry.get());

    return particle;
}


ldplab::Particle ldplab::PropertyGenerator::getRodParticle(
    const double R, 
    const double L, 
    const double kappa, 
    const double np, 
    const double delta_n,
    const double gradient_direction,
    const Vec3 position, 
    const Vec3 orientation)
{
    const double h = kappa * R;
    const double nu = delta_n / (L + h);
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
            Vec3(0, 0, (L + h) / 2),
            bounding_sphere_radius);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            nu,
            Vec3(0, 0, (L + h) / 2),
            Vec3(0, 0 , gradient_direction));
    particle.centre_of_mass = 
        ldplab::PropertyGenerator::getRodParticleCenterOfMass(
            *(ldplab::RodParticleGeometry*)particle.geometry.get());
    return particle;
}

ldplab::Particle ldplab::PropertyGenerator::getSphereParticleByVolume(
    const double V,
    const double np,
    const double delta_n,
    const double gradient_direction,
    const Vec3 position,
    const Vec3 orientation)
{
    const double R = std::pow(V * 3.0 / (const_pi()*4.0), (1.0 / 3.0));
    const double nu = delta_n / (2*R);

    ldplab::Particle particle;
    particle.position = position;
    particle.orientation = orientation;
    particle.geometry =
        std::make_shared<ldplab::SphericalParticleGeometry>(R);
    particle.bounding_volume =
        std::make_shared<ldplab::BoundingVolumeSphere>(
            Vec3(0, 0, 0),
            R + R*1e-4);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            nu,
            Vec3(0, 0, 0),
            Vec3(0, 0 , gradient_direction));
    particle.centre_of_mass = Vec3(0,0,0);
    return particle;
}

ldplab::Particle ldplab::PropertyGenerator::getCapParticleByRadius(
    const double R,
    const double kappa,
    const double np,
    const double delta_n,
    const double gradient_direction, 
    const Vec3 position, 
    const Vec3 orientation)
{
    const double nu = delta_n / (2 * R * kappa);

    ldplab::Particle particle;
    particle.position = position;
    particle.orientation = orientation;
    particle.geometry =
        std::make_shared<ldplab::CapParticleGeometry>(R,2*R*kappa);
    particle.bounding_volume =
        std::make_shared<ldplab::BoundingVolumeSphere>(
            Vec3(0, 0, 0),
            R + R * 1e-6);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            nu,
            Vec3(0, 0, R * (1-kappa)),
            Vec3(0, 0, gradient_direction));
    particle.centre_of_mass = Vec3(0, 0, R * (1 - kappa));
    return particle;
}

ldplab::Particle ldplab::PropertyGenerator::getSphereParticleByRadius(
    const double R,
    const double np,
    const double delta_n,
    const double gradient_direction,
    const Vec3 position,
    const Vec3 orientation)
{
    const double nu = delta_n / (2 * R);

    ldplab::Particle particle;
    particle.position = position;
    particle.orientation = orientation;
    particle.geometry =
        std::make_shared<ldplab::SphericalParticleGeometry>(R);
    particle.bounding_volume =
        std::make_shared<ldplab::BoundingVolumeSphere>(
            Vec3(0, 0, 0),
            R + R * 1e-4);
    particle.material =
        std::make_shared<ldplab::ParticleMaterialLinearOneDirectional>(
            np,
            nu,
            Vec3(0, 0, 0),
            Vec3(0, 0 , gradient_direction));
    particle.centre_of_mass =
        ldplab::PropertyGenerator::getCapParticleCenterOfMass(
            *(ldplab::CapParticleGeometry*)particle.geometry.get());
    return particle;
}