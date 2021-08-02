#ifndef WWU_LDPLAB_RTSCUDA_I_GENERIC_MATERIAL_HPP
#define WWU_LDPLAB_RTSCUDA_I_GENERIC_MATERIAL_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

namespace ldplab
{
    namespace rtscuda
    {
        class IGenericMaterial
        {
        public:
             /**
              * @brief Function pointer type for a function that computes the 
              *        index of refraction of a particle material at a given
              *        position in the particle coordinate system.
              * @param[in] position Position in particle coordinate system.
              * @param[in] particle_material The implementation dependent
              *                              particle material data.
              * @return index of refraction
              */
            typedef double (*indexOfRefraction)(
                const Vec3& position,
                const void* particle_material);
        };
    }
}

#endif
#endif