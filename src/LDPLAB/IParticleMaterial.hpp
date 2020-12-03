#ifndef WWU_LDPLAB_IPARTICLE_MATERIAL_HPP
#define WWU_LDPLAB_IPARTICLE_MATERIAL_HPP

namespace ldplab
{
	class IParticleMaterial
	{
	private:
		enum class Type { homogenous } type;
	public:
		/**
		 * @brief The destructor is virtual since classes inherit from 
		 *        IParticleMaterial.
		 */
		virtual ~IParticleMaterial() { }
		Type getType() { return type; }
	};
}

#endif