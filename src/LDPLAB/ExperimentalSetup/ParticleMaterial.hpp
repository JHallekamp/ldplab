#ifndef WWU_LDPLAB_IPARTICLE_MATERIAL_HPP
#define WWU_LDPLAB_IPARTICLE_MATERIAL_HPP

#include "../Geometry.hpp"
#include "../../../libs/glm/glm.hpp"

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
	};

	struct ParticleMaterialHomogenous : public IParticleMaterial
	{
		Type type() const override { return Type::homogenous; }
		/**
		 * @brief The index of refraction of the hole particle. 
		 */
		double index_of_refraction;
	};

	struct ParticleMaterialLinearOneDirectional : public IParticleMaterial
	{
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
		double indexOfRefraction(const Vec3& position) const { 
			return index_of_refraction + 
				gradient * glm::dot(direction, (position - origin)); }
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
	};
}

#endif