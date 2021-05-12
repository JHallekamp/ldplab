#ifndef WWU_LDPLAB_PARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_PARTICLE_GEOMETRY_HPP

#include <vector>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    struct IParticleGeometry
    {
        enum class Type { sphere, rod_particle, triangle_mesh };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IParticleGeometry.
         */
        virtual ~IParticleGeometry() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::sphere: return "sphere";
            case Type::rod_particle: return "rod_particle";
            case Type::triangle_mesh: return "triangle_mesh";
            default: return "unknown_type";
            }
        }
    };

    /** 
     *  @brief Geometry of an rod like particle. The particle combines a 
     *         cylinder with a half sphere as a cap. At the bottom the same 
     *         half is subtracted.
     */
    struct RodParticleGeometry : public IParticleGeometry
    {
        RodParticleGeometry(real_t cylinder_radius,
            real_t cylinder_length,
            real_t kappa)
            :
            cylinder_radius{ cylinder_radius },
            cylinder_length{ cylinder_length },
            kappa{ kappa },
            l{ cylinder_length/2/cylinder_radius }
        {}
        /** @brief Radius of the cylinder. */
        real_t cylinder_radius;
        /** @brief Length of the cylinder. */
        real_t cylinder_length;
        /** @brief Height of the cap in units of the cylinder radius. */
        real_t kappa;
        /** @brief */
        real_t l;
        Type type() const override 
        { return IParticleGeometry::Type::rod_particle; }
    };

    /**
     *  @brief Geometry of an spherical particle.
     */
    struct SphericalParticleGeometry : public IParticleGeometry
    {
        SphericalParticleGeometry(real_t radius)
            :
            radius{ radius }
        {}
        /** @brief Radius of the sphere. */
        real_t radius;
        Type type() const override
        {
            return IParticleGeometry::Type::sphere;
        }
    };

    /**
     * @brief Geometry of an mash based particle. 
     */
    struct TriangleMeshParticleGeometry : public IParticleGeometry     
    {
        TriangleMeshParticleGeometry(std::vector<Triangle> mesh)
            :
            mesh{ mesh }
        {}
        /** @brief Triangular mesh defining the particle */
        std::vector<Triangle> mesh;
        Type type() const override
        {
            return IParticleGeometry::Type::triangle_mesh;
        }
    };
    
}

#endif