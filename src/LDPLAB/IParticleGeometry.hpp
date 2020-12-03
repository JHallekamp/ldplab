#ifndef WWU_LDPLAB_IPARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_IPARTICLE_GEOMETRY_HPP

namespace ldplab
{
	class IParticleGeometry
	{
	public:
		enum class Type { shpere };
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        IParticleGeometry.
		 */
		virtual ~IParticleGeometry() { }
		/** @brief Returns the type of the instance. */
		virtual Type type() const = 0;
	};
}

#endif