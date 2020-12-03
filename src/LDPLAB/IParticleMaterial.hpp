#ifndef WWU_LDPLAB_IPARTICLE_MATERIAL_HPP
#define WWU_LDPLAB_IPARTICLE_MATERIAL_HPP

namespace ldplab
{
	class IParticleMaterial
	{
	public:
		enum class Type { homogenous };
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from 
		 *        IParticleMaterial.
		 */
		virtual ~IParticleMaterial() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
	};
}

#endif