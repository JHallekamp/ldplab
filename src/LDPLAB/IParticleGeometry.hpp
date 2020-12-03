#ifndef WWU_LDPLAB_IPARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_IPARTICLE_GEOMETRY_HPP

namespace ldplab
{
	class IParticleGeometry
	{
	private:
		enum class Type { shpere } type;
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from
		 *        IParticleGeometry.
		 */
		virtual ~IParticleGeometry() { }
		Type getType() { return type; }
	};
}

#endif