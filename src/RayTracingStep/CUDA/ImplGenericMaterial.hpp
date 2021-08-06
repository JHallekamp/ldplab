#ifndef WWU_LDPLAB_RTSCUDA_IMPL_GENERIC_MATERIAL_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_GENERIC_MATERIAL_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IGenericMaterial.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        struct MaterialLinearOneDirectionalData
        {
            double index_of_refraction;
            double gradient;
            Vec3 origin;
            Vec3 direction;
        };

        class GenericMaterialLinearOneDirectional :
            public IGenericMaterial
        {
        public:
            GenericMaterialLinearOneDirectional(
                DeviceBuffer<MaterialLinearOneDirectionalData>&& material_data);
            void* getDeviceData() override;
            indexOfRefraction getDeviceIndexOfRefractionFunction() override;
        private:
            DeviceBuffer<MaterialLinearOneDirectionalData> m_material_data;
        };
    }
}

#endif
#endif