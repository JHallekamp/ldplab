#ifndef WWU_LDPLAB_PARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_PARTICLE_GEOMETRY_HPP

namespace ldplab
{
    struct IParticleGeometry
    {
        enum class Type { sphere, rod_particle };
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
        RodParticleGeometry(double cylinder_radius,
            double cylinder_length,
            double volume,
            double kappa)
            :
            cylinder_radius{ cylinder_radius },
            cylinder_length{ cylinder_length },
            volume{ volume },
            kappa{ kappa },
            l{ 0 }
        {}
        /** @brief Radius of the cylinder. */
        double cylinder_radius;
        /** @brief Length of the cylinder. */
        double cylinder_length;
        /** @brief Volume of the particle.*/
        double volume;
        /** @brief Height of the cap in units of the cylinder radius. */
        double kappa;
        /** @brief */
        double l;
        Type type() const override 
        { return IParticleGeometry::Type::rod_particle; }
    };
}

#endif