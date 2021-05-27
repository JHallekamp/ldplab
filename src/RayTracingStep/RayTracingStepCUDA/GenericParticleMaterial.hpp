#ifndef WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_MATERIAL_HPP
#define WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_MATERIAL_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/ParticleMaterial.hpp>
#include <LDPLAB/Geometry.hpp>
#include <memory>

#include "CudaResource.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Device resource holding generic particle material data. */
        struct GenericParticleMaterialData
        {
            /** @brief The type of the bounding volume. */
            enum Type { TYPE_HOMOGENOUS, TYPE_LINEAR_ONE_DIRECTIONAL } type;
            /** @brief Pointer to the bounding resource. */
            void* data;
        };

        /** @brief Container baseclass for generic particle materials. */
        class GenericParticleMaterial
        {
        public:
            virtual ~GenericParticleMaterial() { }
            /** @brief Creates a generic bounding volume instance. */
            static std::shared_ptr<GenericParticleMaterial> create(
                std::shared_ptr<IParticleMaterial> particle_material);
            /** @brief Provides the resource data. */
            virtual GenericParticleMaterialData getData() = 0;
        protected:
            /** @brief Allocates the device resource. */
            virtual bool allocate(
                std::shared_ptr<IParticleMaterial> particle_material) = 0;
        };

        /** @brief Container holding homogeneous particle material data. */
        class ParticleHomogeneousMaterial : public GenericParticleMaterial
        {
        public:
            struct Data
            {
                double index_of_refraction;
            };
        public:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleMaterial */
            GenericParticleMaterialData getData() override;
        protected:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleMaterial */
            bool allocate(
                std::shared_ptr<IParticleMaterial> particle_material) override;
        private:
            CudaPtr<Data> m_data;
        };

        /** 
         * @brief Container holding linear one directional particle material 
         *        data. 
         */
        class ParticleLinearOneDirectionalMaterial : 
            public GenericParticleMaterial
        {
        public:
            struct Data
            {
                inline __device__ double indexOfRefraction(
                    const Vec3& position) const;
                double index_of_refraction;
                double gradient;
                Vec3 origin;
                Vec3 direction;
            };
        public:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleMaterial */
            GenericParticleMaterialData getData() override;
        protected:
            /** @brief Inherited via ldplab::rtscuda::GenericParticleMaterial */
            bool allocate(
                std::shared_ptr<IParticleMaterial> particle_material) override;
        private:
            CudaPtr<Data> m_data;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_GENERIC_PARTICLE_MATERIAL_HPP