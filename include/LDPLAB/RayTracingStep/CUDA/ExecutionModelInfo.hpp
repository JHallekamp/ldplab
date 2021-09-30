#ifndef WWU_LDPLAB_RTSCUDA_PARALLEL_INFO_HPP
#define WWU_LDPLAB_RTSCUDA_PARALLEL_INFO_HPP

#include <map>

namespace ldplab
{
    namespace rtscuda
    {
        /** 
         * @brief Baseclass for different types of cuda RTS execution model
         *        info.
         */
        struct IExecutionModelInfo
        {
            enum class Type { 
                auto_construction, 
                explicit_configuration };
            virtual ~IExecutionModelInfo() = default;
            virtual Type type() const = 0;
        };

        struct ExecutionModelAutoConstructionInfo : IExecutionModelInfo
        {
            Type type() const override { return Type::auto_construction; }
            size_t num_streams_per_device = 1;
            enum class DeviceModel {
                /** @brief Selects device with lowest id */
                single_device,
                /** @brief Distributes streams equally over all devices. */
                distribute_equally
            } device_model = DeviceModel::distribute_equally;
            // Ctors
            ExecutionModelAutoConstructionInfo() = default;
            ExecutionModelAutoConstructionInfo(
                size_t streams_per_device)
                :
                num_streams_per_device{ streams_per_device },
                device_model{ DeviceModel::distribute_equally }
            { }
            ExecutionModelAutoConstructionInfo(
                size_t streams_per_device, 
                DeviceModel device_model)
                :
                num_streams_per_device{ streams_per_device },
                device_model{ device_model }
            { }
        };

        struct ExecutionModelExplicitConfigInfo : IExecutionModelInfo
        {
            Type type() const override { return Type::explicit_configuration; }
            size_t num_streams;
            size_t num_device_groups;
            std::map<size_t, size_t> map_stream_to_device_group;
            std::map<size_t, int> map_device_group_to_device_id;
        };

    }
}

#endif