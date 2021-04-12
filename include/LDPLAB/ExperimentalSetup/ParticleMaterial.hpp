#ifndef WWU_LDPLAB_IPARTICLE_MATERIAL_HPP
#define WWU_LDPLAB_IPARTICLE_MATERIAL_HPP

#include "../Geometry.hpp"
#include <glm/glm.hpp>

namespace ldplab
{
	struct IParticleMaterial
	{
		enum class Type { homogenous, linear_one_directional };
		/**
		 * @brief The destructor is virtual since classes inherit from 
		 *        IParticleMaterial.
		 */
		virtual ~IParticleMaterial() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::homogenous: return "homogenous";
            case Type::linear_one_directional: return "linear_one_directional";
            default: return "unknown_type";
            }
        }
	};

	struct ParticleMaterialHomogenous : public IParticleMaterial
	{
        ParticleMaterialHomogenous(double index_of_refraction)
            :
            index_of_refraction{ index_of_refraction }
        {}
		Type type() const override { return Type::homogenous; }
		/**
		 * @brief The index of refraction of the hole particle. 
		 */
		double index_of_refraction;
	};

	struct ParticleMaterialLinearOneDirectional : public IParticleMaterial
	{
        ParticleMaterialLinearOneDirectional(double index_of_refraction,
            double gradient,
            Vec3 origin,
            Vec3 direction)
            :
            index_of_refraction{ index_of_refraction },
            gradient{ gradient },
            origin{ origin },
            direction{ direction },
            direction_times_gradient{ direction * gradient },
            index_of_refraction_minus_partial_dot{ 
                index_of_refraction - glm::dot(gradient * direction, origin) }
        {}
		Type type() const override { return Type::linear_one_directional; }
		/**
		 * @brief Calculating the index of refraction at given position in the
		 *        particle.
		 * @param[in] position Location to evaluate the index of refraction 
		 *                     given in the particle coordinate system.
		 * @returns The index of refraction at given position.
		 * @warning There is no checking if the position is inside the 
		 *           particle.
		 */
		inline double indexOfRefraction(const Vec3& position) const { 
		    return index_of_refraction + 
		    	gradient * glm::dot(direction, (position - origin));
            //return index_of_refraction_minus_partial_dot +
            //    glm::dot(direction_times_gradient, position);
        }
		/**
		 * @brief The index of refraction at the origin of the linear index 
		 *        change.
		 */
		double index_of_refraction;
		/**
		 * @brief The gradient of linear index of refraction change. 
		 */
		double gradient;
		/**
		 * @brief The origin of the index change.
		 * @detail The origin is given in the particle coordinate system.
		 */
		Vec3 origin;
		/**
		 * @brief The direction in which the index of refraction changes.
		 * @detail The direction is given in the particle coordinate system.
		 */
		Vec3 direction;
        
        Vec3 direction_times_gradient;
        double index_of_refraction_minus_partial_dot;
	};
}

#endif