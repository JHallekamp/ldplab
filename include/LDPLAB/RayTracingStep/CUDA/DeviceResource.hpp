#ifndef WWU_LDPLAB_RTSCUDA_DEVICE_RESOURCE_HPP
#define WWU_LDPLAB_RTSCUDA_DEVICE_RESOURCE_HPP

#include <memory>
#include <vector>
#include <cuda_runtime.h>
#include <LDPLAB/UID.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Interface for device resources. */
        class IDeviceResource
        {
        public:
            virtual ~IDeviceResource() { }
            /** @brief Frees the resource. */
            virtual bool free() = 0;
            /** @brief Checks if the resource is allocated. */
            virtual bool allocated() = 0;
            /** @brief The resource UID. */
            UID<IDeviceResource> uid() const;
        protected:
            virtual const char* resourceTypeName() const = 0;
            bool checkCudaAllocationError(
                cudaError_t cuda_return_value,
                size_t num_bytes) const;
            bool checkCudaFreeError(
                cudaError_t cuda_return_value,
                void* device_ptr,
                size_t num_bytes) const;
            bool checkCudaMemsetError(
                cudaError_t cuda_return_value,
                void* device_offset_ptr,
                void* device_base_ptr,
                size_t num_bytes,
                int value) const;
            bool checkCudaUploadError(
                cudaError_t cuda_return_value,
                void* device_offset_ptr,
                void* device_base_ptr,
                void* host_address,
                size_t num_bytes) const;
            bool checkCudaDownloadError(
                cudaError_t cuda_return_value,
                void* device_offset_ptr,
                void* device_base_ptr,
                void* host_address,
                size_t num_bytes) const;
        private:
            /** @brief Resource uid. */
            UID<IDeviceResource> m_resource_uid;
        };
        
        /**
         * @brief Simple wrapper around multiple device buffers and potential
         *        host buffers used for up- and downloading data.
         * @tparam basetype Type of the underlying data elements.
         */
        template <typename basetype>
        class DeviceBufferRange : public IDeviceResource
        {
        public:
            DeviceBufferRange();
            DeviceBufferRange(const DeviceBufferRange<basetype>& other) = delete;
            DeviceBufferRange(DeviceBufferRange<basetype>&& other);
            virtual ~DeviceBufferRange() { free(); }
            bool allocated() override;
            bool free() override;
            /**
             * @brief Allocates memory.
             * @details Allocates num_device_buffers on the device and 
             *          additionally num_host_buffers in host memory (used for
             *          host-device data exchange), each measuring buffer_size
             *          basetype elements.
             * @param[in] buffer_size The number of basetype elements per 
             *                        allocated buffer.
             * @param[in] num_device_buffers Number of buffers allocated in
             *                               device memory.
             * @param[in] num_host_buffers Number of buffers allocated in
             *                             host memory.
             * @returns false, if an error occurred.
             */
            bool allocate(
                size_t buffer_size,
                size_t num_device_buffers,
                size_t num_host_buffers);
            /** @brief Provides the number of buffers in the device memory. */
            size_t deviceSize() const;
            /** @brief Provides the number of buffers in the host memory. */
            size_t hostSize() const;
            /** @brief Provides the number of elements inside each buffer. */
            size_t bufferSize() const;
            /**
             * @brief Provides a pointer into device memory that points to the
             *        specified device buffer.
             * @param[in] index The index of the requested device buffer.
             * @returns Device memory pointer to the requested buffer.
             */
            basetype* getDeviceBuffer(size_t index) const;
            /**
             * @brief Provides a pointer into host memory that points to the
             *        specified host buffer.
             * @param[in] index The index of the requested host buffer.
             * @returns Host memory pointer to the requested buffer.
             */
            basetype* getHostBuffer(size_t index);
            /**
             * @brief Provides the device buffer range pointer.
             * @details The device buffer range is an array in device memory,
             *          which contains the pointer to the device buffers as
             *          elements.
             * @returns Device memory pointer to the device buffer range.
             */
            basetype** getDeviceBufferRange();
            /**
             * @brief Memsets a specific device buffer.
             * @param[in] val Value to which the memory is set. 
             * @param[in] buffer_index The index of the device buffer that is
             *                         set.
             * @returns false, if an error occurred.
             */
            bool memset(int val, size_t buffer_index);
            /**
             * @brief Memsets all device buffers.
             * @param[in] val Value to which the memory is set.
             * @returns false, if an error occurred.
             */
            bool memset(int val);
            /**
             * @brief Uploads data from a host buffer to a device buffer.
             * @param[in] target_device_buffer The buffer index of the target
             *                                 buffer in device memory.
             * @param[in] source_host_buffer The buffer index of the source
             *                               buffer in host memory.
             * @returns false, if an error occurred.
             */
            bool upload(
                size_t target_device_buffer, 
                size_t source_host_buffer);
            /**
             * @brief Uploads data from a given host pointer to a device buffer.
             * @param[in] target_device_buffer The buffer index of the target
             *                                 buffer in device memory.
             * @param[in] source_ptr Source buffer pointer on the host.
             * @returns false, if an error occurred.
             */
            bool uploadExt(
                size_t target_device_buffer,
                basetype* source_ptr);
            /**
             * @brief Uploads data from a host buffer to a device buffer.
             * @param[in] target_device_buffer The buffer index of the target
             *                                 buffer in device memory.
             * @param[in] source_host_buffer The buffer index of the source
             *                               buffer in host memory.
             * @param[in] element_offset Element offset inside both buffers.
             * @param[in] element_count Number of uploaded elements.
             * @returns false, if an error occurred.
             */
            bool upload(
                size_t target_device_buffer,
                size_t source_host_buffer,
                size_t element_offset,
                size_t element_count);
            /**
             * @brief Downloads data from a device buffer to a host buffer.
             * @param[in] target_host_buffer The buffer index of the target
             *                               buffer in host memory.
             * @param[in] source_device_buffer The buffer index of the source
             *                                 buffer in device memory.
             * @returns false, if an error occurred.
             */
            bool download(
                size_t target_host_buffer,
                size_t source_device_buffer);
            /**
             * @brief Downloads data from a device buffer to a host buffer.
             * @param[in] target_host_buffer The buffer index of the target
             *                               buffer in host memory.
             * @param[in] source_device_buffer The buffer index of the source
             *                                 buffer in device memory.
             * @param[in] element_offset Element offset inside both buffers.
             * @param[in] element_count Number of downloaded elements.
             * @returns false, if an error occurred.
             */
            bool download(
                size_t target_host_buffer,
                size_t source_device_buffer,
                size_t element_offset,
                size_t element_count);
        protected:
            virtual const char* resourceTypeName() const override
            {
                return "DeviceBufferRange";
            }
        private:
            void** m_device_range_ptr;
            std::vector<void*> m_device_buffer_ptr_range;
            std::vector <std::vector<basetype>> m_host_buffer_ptr_range;
            size_t m_buffer_size;
        };

        /**
         * @brief Simple implementation of a device buffer range containing 
         *        just a single buffer.
         */
        template <typename basetype>
        class DeviceBuffer : public DeviceBufferRange<basetype>
        {
        private:
            using baseclass = DeviceBufferRange<basetype>;
        public:
            /**
             * @brief Allocates the buffer on device memory.
             * @param[in] buffer_size The size of the allocated buffer in
             *                        elements.
             * @param[in] host_buffer Indicates whether an equivalent buffer
             *                        is allocated in host memory (required for
             *                        up- and download capability).
             * @returns false, if an error occurred.
             */
            bool allocate(size_t buffer_size, bool host_buffer = true);
            /** 
             * @brief Returns true, if the device buffer as an equivalent in
             *        host memory.
             */
            bool hasHostBuffer() const { return (baseclass::hostSize() > 0); }
            basetype* getDeviceBuffer() const { return baseclass::getDeviceBuffer(0); }
            basetype* getHostBuffer() { return baseclass::getHostBuffer(0); }
            bool memset(int val) { return baseclass::memset(val); }
            bool upload() { return baseclass::upload(0, 0); }
            bool uploadExt(basetype* host_buffer) { return baseclass::uploadExt(0, host_buffer); }
            bool upload(size_t element_offset, size_t element_count);
            bool download() { return baseclass::download(0, 0); }
            bool download(size_t element_offset, size_t element_count);
        protected:
            virtual const char* resourceTypeName() const override
            {
                return "DeviceBuffer";
            }
        private:
            using baseclass::allocate;
            using baseclass::deviceSize;
            using baseclass::hostSize;
            using baseclass::getDeviceBuffer;
            using baseclass::getHostBuffer;
            using baseclass::getDeviceBufferRange;
            using baseclass::memset;
            using baseclass::upload;
            using baseclass::uploadExt;
            using baseclass::download;
        };

        /**
         * @brief Simple wrapper around multiple device buffers using pinnend
         *        memory on the host.
         */
        template<typename basetype>
        class DeviceBufferRangePinnend : public IDeviceResource
        {
        public:
            DeviceBufferRangePinnend();
            DeviceBufferRangePinnend(const DeviceBufferRangePinnend<basetype>& other) = delete;
            DeviceBufferRangePinnend(DeviceBufferRangePinnend<basetype>&& other);
            virtual ~DeviceBufferRangePinnend() { free(); }
            bool allocated() override { return m_device_buffer_ptr_range.size() > 0; }
            bool free() override;
            /**
             * @brief Allocates memory.
             * @details Allocates num_device_buffers on the device and
             *          additionally num_host_buffers in host memory (used for
             *          host-device data exchange), each measuring buffer_size
             *          basetype elements.
             * @param[in] buffer_size The number of basetype elements per
             *                        allocated buffer.
             * @param[in] num_device_buffers Number of buffers allocated in
             *                               device memory.
             * @param[in] num_host_buffers Number of buffers allocated in
             *                             host memory.
             * @returns false, if an error occurred.
             */
            bool allocate(
                size_t buffer_size,
                size_t num_device_buffers,
                size_t num_host_buffers);
            /** @brief Provides the number of buffers in the device memory. */
            size_t deviceSize() const { return m_device_buffer_ptr_range.size(); }
            /** @brief Provides the number of buffers in the host memory. */
            size_t hostSize() const { return m_pinnend_host_buffer_ptr_range.size(); }
            /** @brief Provides the number of elements inside each buffer. */
            size_t bufferSize() const { return m_buffer_size; }
            /**
             * @brief Provides a pointer into device memory that points to the
             *        specified device buffer.
             * @param[in] index The index of the requested device buffer.
             * @returns Device memory pointer to the requested buffer.
             */
            basetype* getDeviceBuffer(size_t index) const 
            { return static_cast<basetype*>(m_device_buffer_ptr_range[index]); }
            /**
             * @brief Provides a pointer into host memory that points to the
             *        specified host buffer.
             * @param[in] index The index of the requested host buffer.
             * @returns Host memory pointer to the requested buffer.
             */
            basetype* getHostBuffer(size_t index)
            { return static_cast<basetype*>(m_pinnend_host_buffer_ptr_range[index]); }
            /**
             * @brief Provides the device buffer range pointer.
             * @details The device buffer range is an array in device memory,
             *          which contains the pointer to the device buffers as
             *          elements.
             * @returns Device memory pointer to the device buffer range.
             */
            basetype** getDeviceBufferRange() 
            { return static_cast<basetype*>(m_device_range_ptr); }
            /**
             * @brief Uploads data from a host buffer to a device buffer.
             * @param[in] target_device_buffer The buffer index of the target
             *                                 buffer in device memory.
             * @param[in] source_host_buffer The buffer index of the source
             *                               buffer in host memory.
             * @param[in] stream The stream used for the async call.
             * @returns false, if an error occurred.
             */
            bool uploadAsync(
                size_t target_device_buffer,
                size_t source_host_buffer,
                cudaStream_t stream);
            /**
             * @brief Uploads data from a host buffer to a device buffer.
             * @param[in] target_device_buffer The buffer index of the target
             *                                 buffer in device memory.
             * @param[in] source_host_buffer The buffer index of the source
             *                               buffer in host memory.
             * @param[in] element_offset Element offset inside both buffers.
             * @param[in] element_count Number of uploaded elements.
             * @param[in] stream The stream used for the async call.
             * @returns false, if an error occurred.
             */
            bool uploadAsync(
                size_t target_device_buffer,
                size_t source_host_buffer,
                size_t element_offset,
                size_t element_count,
                cudaStream_t stream);
            /**
             * @brief Downloads data from a device buffer to a host buffer.
             * @param[in] target_host_buffer The buffer index of the target
             *                               buffer in host memory.
             * @param[in] source_device_buffer The buffer index of the source
             *                                 buffer in device memory.
             * @param[in] stream The stream used for the async call.
             * @returns false, if an error occurred.
             */
            bool downloadAsync(
                size_t target_host_buffer,
                size_t source_device_buffer,
                cudaStream_t stream);
            /**
             * @brief Downloads data from a device buffer to a host buffer.
             * @param[in] target_host_buffer The buffer index of the target
             *                               buffer in host memory.
             * @param[in] source_device_buffer The buffer index of the source
             *                                 buffer in device memory.
             * @param[in] element_offset Element offset inside both buffers.
             * @param[in] element_count Number of downloaded elements.
             * @param[in] stream The stream used for the async call.
             * @returns false, if an error occurred.
             */
            bool downloadAsync(
                size_t target_host_buffer,
                size_t source_device_buffer,
                size_t element_offset,
                size_t element_count,
                cudaStream_t stream);
        protected:
            virtual const char* resourceTypeName() const override
            {
                return "DeviceBufferRangePinnend";
            }
        private:
            void** m_device_range_ptr;
            std::vector<void*> m_device_buffer_ptr_range;
            std::vector<void*> m_pinnend_host_buffer_ptr_range;
            size_t m_buffer_size;
        };

        /**
         * @brief Simple implementation of a device buffer range containing
         *        just a single buffer.
         */
        template <typename basetype>
        class DeviceBufferPinnend : public DeviceBufferRangePinnend<basetype>
        {
        private:
            using baseclass = DeviceBufferRangePinnend<basetype>;
        public:
            /**
             * @brief Allocates the buffer on device memory.
             * @param[in] buffer_size The size of the allocated buffer in
             *                        elements.
             * @param[in] host_buffer Indicates whether an equivalent buffer
             *                        is allocated in host memory (required for
             *                        up- and download capability).
             * @returns false, if an error occurred.
             */
            bool allocate(size_t buffer_size, bool host_buffer = true);
            /**
             * @brief Returns true, if the device buffer as an equivalent in
             *        host memory.
             */
            bool hasHostBuffer() const { return (baseclass::hostSize() > 0); }
            basetype* getDeviceBuffer() const { return baseclass::getDeviceBuffer(0); }
            basetype* getHostBuffer() { return baseclass::getHostBuffer(0); }
            bool uploadAsync(cudaStream_t stream);
            bool uploadAsync(size_t element_offset, size_t element_count, cudaStream_t stream);
            bool downloadAsync(cudaStream_t stream);
            bool downloadAsync(size_t element_offset, size_t element_count, cudaStream_t stream);
        protected:
            virtual const char* resourceTypeName() const override
            {
                return "DeviceBufferPinnend";
            }
        private:
            using baseclass::allocate;
            using baseclass::deviceSize;
            using baseclass::hostSize;
            using baseclass::getDeviceBuffer;
            using baseclass::getHostBuffer;
            using baseclass::getDeviceBufferRange;
            using baseclass::uploadAsync;
            using baseclass::downloadAsync;
        };

        template<typename basetype>
        inline bool DeviceBuffer<basetype>::allocate(
            size_t buffer_size, 
            bool host_buffer)
        {
            return baseclass::allocate(buffer_size, 1, host_buffer ? 1 : 0);
        }

        template<typename basetype>
        inline bool DeviceBuffer<basetype>::upload(
            size_t element_offset, 
            size_t element_count)
        {
            return baseclass::upload(0, 0, element_offset, element_count);
        }

        template<typename basetype>
        inline bool DeviceBuffer<basetype>::download(
            size_t element_offset, 
            size_t element_count)
        {
            return baseclass::download(0, 0, element_offset, element_count);
        }

        template<typename basetype>
        inline DeviceBufferRange<basetype>::DeviceBufferRange()
            :
            m_buffer_size{ 0 },
            m_device_buffer_ptr_range{ },
            m_host_buffer_ptr_range{ } ,
            m_device_range_ptr{ nullptr }
        { }

        template<typename basetype>
        inline DeviceBufferRange<basetype>::DeviceBufferRange(
            DeviceBufferRange<basetype>&& other)
        {
            if (allocated())
                free();
            m_buffer_size = other.m_buffer_size;
            m_device_buffer_ptr_range = std::move(other.m_device_buffer_ptr_range);
            m_host_buffer_ptr_range = std::move(other.m_host_buffer_ptr_range);
            m_device_range_ptr = other.m_device_range_ptr;
            other.m_device_range_ptr = nullptr;
            other.m_buffer_size = 0;
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::allocated()
        {
            return (m_device_buffer_ptr_range.size() > 0);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::free()
        {
            bool ret = true;
            if (m_device_range_ptr)
            {
                ret = ret && checkCudaFreeError(
                    cudaFree(m_device_range_ptr),
                    m_device_range_ptr,
                    m_device_buffer_ptr_range.size() * sizeof(void*));
                m_device_range_ptr = nullptr;
            }
            for (size_t i = 0; i < m_device_buffer_ptr_range.size(); ++i)
            {
                if (m_device_buffer_ptr_range[i] != nullptr)
                {
                    ret = ret && checkCudaFreeError(
                        cudaFree(m_device_buffer_ptr_range[i]),
                        m_device_buffer_ptr_range[i],
                        m_buffer_size * sizeof(basetype));
                    m_device_buffer_ptr_range[i] = nullptr;
                }
            }
            m_device_buffer_ptr_range.clear();
            m_host_buffer_ptr_range.clear();
            m_buffer_size = 0;
            return ret;
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::allocate(
            size_t buffer_size, 
            size_t num_device_buffers, 
            size_t num_host_buffers)
        {
            m_buffer_size = buffer_size;
            // Allocate device buffers
            bool ret = true;
            const size_t buffer_byte_size = buffer_size * sizeof(basetype);
            for (size_t i = 0; i < num_device_buffers; ++i)
            {
                m_device_buffer_ptr_range.emplace_back();
                ret = ret && checkCudaAllocationError(
                    cudaMalloc(&m_device_buffer_ptr_range.back(), buffer_byte_size),
                    buffer_byte_size);
            }
            // Allocate device buffer range
            const size_t range_byte_size = num_device_buffers * sizeof(void*);
            ret = ret && checkCudaAllocationError(
                cudaMalloc(&m_device_range_ptr, range_byte_size),
                range_byte_size);
            ret = ret && checkCudaUploadError(
                cudaMemcpy(
                    m_device_range_ptr,
                    m_device_buffer_ptr_range.data(),
                    num_device_buffers,
                    cudaMemcpyHostToDevice),
                m_device_range_ptr,
                m_device_range_ptr,
                m_device_buffer_ptr_range.data(),
                range_byte_size);
            // Allocate host mem
            for (size_t i = 0; i < num_host_buffers; ++i)
            {
                m_host_buffer_ptr_range.emplace_back();
                m_host_buffer_ptr_range.back().resize(m_buffer_size);
            }
            return ret;
        }

        template<typename basetype>
        inline size_t DeviceBufferRange<basetype>::deviceSize() const
        {
            return m_device_buffer_ptr_range.size();
        }

        template<typename basetype>
        inline size_t DeviceBufferRange<basetype>::hostSize() const
        {
            return m_host_buffer_ptr_range.size();
        }

        template<typename basetype>
        inline size_t DeviceBufferRange<basetype>::bufferSize() const
        {
            return m_buffer_size;
        }

        template<typename basetype>
        inline basetype* DeviceBufferRange<basetype>::getDeviceBuffer(size_t index) const
        {
            return static_cast<basetype*>(m_device_buffer_ptr_range[index]);
        }

        template<typename basetype>
        inline basetype* DeviceBufferRange<basetype>::getHostBuffer(size_t index)
        {
            return m_host_buffer_ptr_range[index].data();
        }

        template<typename basetype>
        inline basetype** DeviceBufferRange<basetype>::getDeviceBufferRange()
        {
            return static_cast<basetype*>(m_device_range_ptr);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::memset(
            int val, 
            size_t buffer_index)
        {
            return checkCudaMemsetError(
                cudaMemset(m_device_buffer_ptr_range[buffer_index], val, m_buffer_size),
                m_device_buffer_ptr_range[buffer_index],
                m_device_buffer_ptr_range[buffer_index],
                m_buffer_size * sizeof(basetype),
                val);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::memset(int val)
        {
            bool ret = true;
            for (size_t i = 0; i < m_device_buffer_ptr_range.size(); ++i)
                ret = ret && memset(val, i);
            return ret;
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::upload(
            size_t target_device_buffer, 
            size_t source_host_buffer)
        {
            return upload(target_device_buffer, source_host_buffer, 0, m_buffer_size);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::uploadExt(
            size_t target_device_buffer, 
            basetype* source_ptr)
        {
            void* device_ptr = getDeviceBuffer(target_device_buffer);
            const size_t num_bytes = sizeof(basetype) * m_buffer_size;
            return checkCudaUploadError(
                cudaMemcpy(
                    device_ptr,
                    (void*)source_ptr,
                    num_bytes,
                    cudaMemcpyHostToDevice),
                device_ptr,
                m_device_buffer_ptr_range[target_device_buffer],
                source_ptr,
                num_bytes);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::upload(
            size_t target_device_buffer, 
            size_t source_host_buffer, 
            size_t element_offset, 
            size_t element_count)
        {
            void* device_ptr = static_cast<void*>(
                getDeviceBuffer(target_device_buffer) + element_offset);
            void* host_ptr = static_cast<void*>(
                getHostBuffer(source_host_buffer) + element_offset);
            const size_t num_bytes = sizeof(basetype) * element_count;
            return checkCudaUploadError(
                cudaMemcpy(
                    device_ptr,
                    host_ptr,
                    num_bytes,
                    cudaMemcpyHostToDevice),
                device_ptr,
                m_device_buffer_ptr_range[target_device_buffer],
                host_ptr,
                num_bytes);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::download(
            size_t target_host_buffer, 
            size_t source_device_buffer)
        {
            return download(target_host_buffer, source_device_buffer, 0, m_buffer_size);
        }

        template<typename basetype>
        inline bool DeviceBufferRange<basetype>::download(
            size_t target_host_buffer, 
            size_t source_device_buffer, 
            size_t element_offset, 
            size_t element_count)
        {
            void* host_ptr = static_cast<void*>(
                getHostBuffer(target_host_buffer) + element_offset);
            void* device_ptr = static_cast<void*>(
                getDeviceBuffer(source_device_buffer) + element_offset);
            const size_t num_bytes = sizeof(basetype) * element_count;
            return checkCudaDownloadError(
                cudaMemcpy(
                    host_ptr,
                    device_ptr,
                    num_bytes,
                    cudaMemcpyDeviceToHost),
                device_ptr,
                m_device_buffer_ptr_range[source_device_buffer],
                host_ptr,
                num_bytes);
        }

        template<typename basetype>
        inline DeviceBufferRangePinnend<basetype>::DeviceBufferRangePinnend()
            :
            m_buffer_size{ 0 },
            m_device_buffer_ptr_range{ },
            m_pinnend_host_buffer_ptr_range{ },
            m_device_range_ptr{ nullptr }
        { }

        template<typename basetype>
        inline DeviceBufferRangePinnend<basetype>::DeviceBufferRangePinnend(
            DeviceBufferRangePinnend<basetype>&& other)
        {
            if (allocated())
                free();
            m_device_buffer_ptr_range = 
                std::move(other.m_device_buffer_ptr_range);
            m_pinnend_host_buffer_ptr_range = 
                std::move(other.m_pinnend_host_buffer_ptr_range);
            m_device_range_ptr = other.m_device_range_ptr;
            m_buffer_size = other.m_buffer_size;
            other.m_device_range_ptr = nullptr;
            other.m_buffer_size = 0;
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::free()
        {
            bool ret = true;
            if (m_device_range_ptr)
            {
                ret = ret && checkCudaFreeError(
                    cudaFree(m_device_range_ptr),
                    m_device_range_ptr,
                    m_device_buffer_ptr_range.size() * sizeof(void*));
                m_device_range_ptr = nullptr;
            }
            for (size_t i = 0; i < m_device_buffer_ptr_range.size(); ++i)
            {
                if (m_device_buffer_ptr_range[i] != nullptr)
                {
                    ret = ret && checkCudaFreeError(
                        cudaFree(m_device_buffer_ptr_range[i]),
                        m_device_buffer_ptr_range[i],
                        m_buffer_size * sizeof(basetype));
                    m_device_buffer_ptr_range[i] = nullptr;
                }
            }
            m_device_buffer_ptr_range.clear();
            for (size_t i = 0; i < m_pinnend_host_buffer_ptr_range.size(); ++i)
            {
                if (m_pinnend_host_buffer_ptr_range[i] != nullptr)
                {
                    if (cudaFreeHost(m_pinnend_host_buffer_ptr_range[i]) !=
                        cudaSuccess)
                        ret = false;
                    m_pinnend_host_buffer_ptr_range[i] = nullptr;
                }
            }
            m_pinnend_host_buffer_ptr_range.clear();
            m_buffer_size = 0;
            return ret;
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::allocate(
            size_t buffer_size, 
            size_t num_device_buffers, 
            size_t num_host_buffers)
        {
            m_buffer_size = buffer_size;
            // Allocate device buffers
            bool ret = true;
            const size_t buffer_byte_size = buffer_size * sizeof(basetype);
            for (size_t i = 0; i < num_device_buffers; ++i)
            {
                m_device_buffer_ptr_range.emplace_back();
                ret = ret && checkCudaAllocationError(
                    cudaMalloc(&m_device_buffer_ptr_range.back(), buffer_byte_size),
                    buffer_byte_size);
            }
            // Allocate device buffer range
            const size_t range_byte_size = num_device_buffers * sizeof(void*);
            ret = ret && checkCudaAllocationError(
                cudaMalloc(&m_device_range_ptr, range_byte_size),
                range_byte_size);
            ret = ret && checkCudaUploadError(
                cudaMemcpy(
                    m_device_range_ptr,
                    m_device_buffer_ptr_range.data(),
                    num_device_buffers,
                    cudaMemcpyHostToDevice),
                m_device_range_ptr,
                m_device_range_ptr,
                m_device_buffer_ptr_range.data(),
                range_byte_size);
            // Allocate host mem
            for (size_t i = 0; i < num_host_buffers; ++i)
            {
                void* temp_ptr; 
                if (cudaMallocHost(&temp_ptr, buffer_byte_size) != cudaSuccess)
                    ret = false;
                else
                    m_pinnend_host_buffer_ptr_range.push_back(temp_ptr);
            }
            return ret;
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::uploadAsync(
            size_t target_device_buffer, 
            size_t source_host_buffer, 
            cudaStream_t stream)
        {
            return uploadAsync(
                target_device_buffer,
                source_host_buffer,
                0,
                m_buffer_size,
                stream);
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::uploadAsync(
            size_t target_device_buffer, 
            size_t source_host_buffer, 
            size_t element_offset, 
            size_t element_count, 
            cudaStream_t stream)
        {
            void* device_ptr = static_cast<void*>(
                getDeviceBuffer(target_device_buffer) + element_offset);
            void* host_ptr = static_cast<void*>(
                getHostBuffer(source_host_buffer) + element_offset);
            const size_t num_bytes = sizeof(basetype) * element_count;
            return checkCudaUploadError(
                cudaMemcpyAsync(
                    device_ptr,
                    host_ptr,
                    num_bytes,
                    cudaMemcpyHostToDevice,
                    stream),
                device_ptr,
                m_device_buffer_ptr_range[target_device_buffer],
                host_ptr,
                num_bytes);
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::downloadAsync(
            size_t target_host_buffer, 
            size_t source_device_buffer, 
            cudaStream_t stream)
        {
            return downloadAsync(
                target_host_buffer,
                source_device_buffer,
                0,
                m_buffer_size,
                stream);
        }

        template<typename basetype>
        inline bool DeviceBufferRangePinnend<basetype>::downloadAsync(
            size_t target_host_buffer, 
            size_t source_device_buffer, 
            size_t element_offset, 
            size_t element_count, 
            cudaStream_t stream)
        {
            void* host_ptr = static_cast<void*>(
                getHostBuffer(target_host_buffer) + element_offset);
            void* device_ptr = static_cast<void*>(
                getDeviceBuffer(source_device_buffer) + element_offset);
            const size_t num_bytes = sizeof(basetype) * element_count;
            return checkCudaDownloadError(
                cudaMemcpyAsync(
                    host_ptr,
                    device_ptr,
                    num_bytes,
                    cudaMemcpyDeviceToHost,
                    stream),
                device_ptr,
                m_device_buffer_ptr_range[source_device_buffer],
                host_ptr,
                num_bytes);
        }
        
        template<typename basetype>
        inline bool DeviceBufferPinnend<basetype>::allocate(
            size_t buffer_size, 
            bool host_buffer)
        {
            return baseclass::allocate(buffer_size, 1, host_buffer ? 1 : 0);
        }

        template<typename basetype>
        inline bool DeviceBufferPinnend<basetype>::uploadAsync(cudaStream_t stream)
        {
            return baseclass::uploadAsync(0, 0, stream);
        }
        template<typename basetype>
        inline bool DeviceBufferPinnend<basetype>::uploadAsync(
            size_t element_offset, 
            size_t element_count, 
            cudaStream_t stream)
        {
            return baseclass::uploadAsync(0, 0, element_offset, element_count, stream);
        }

        template<typename basetype>
        inline bool DeviceBufferPinnend<basetype>::downloadAsync(cudaStream_t stream)
        {
            return baseclass::downloadAsync(0, 0, stream);
        }

        template<typename basetype>
        inline bool DeviceBufferPinnend<basetype>::downloadAsync(
            size_t element_offset, 
            size_t element_count, 
            cudaStream_t stream)
        {
            return baseclass::downloadAsync(0, 0, element_offset, element_count, stream);
        }
    }
}

#endif