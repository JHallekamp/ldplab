#ifndef WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP
#define WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstdint>
#include <cuda_runtime.h>
#include <LDPLAB/UID.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Baseclass for CUDA memory resource wrapper classes. */
        class ICudaResource
        {
        public:
            ICudaResource() : m_device_ptr{ 0 }{ }
            ICudaResource(ICudaResource&& other) noexcept;
            virtual ~ICudaResource() { free(); }
            /** @brief Frees the resource. */
            virtual bool free() = 0;
            /** @brief Checks if the resource is allocated. */
            inline bool isAllocated() const { return m_device_ptr != 0; }
            /** @brief Provides the device pointer as raw void pointer. */
            inline void* getResource() const { return m_device_ptr; }
        protected:
            virtual const char* resourceTypeName() const = 0;
            bool hasAllocationError(
                cudaError_t return_value,
                size_t num_bytes) const;
            bool hasFreeError(
                cudaError_t return_value,
                size_t num_bytes) const;
            bool hasMemsetError(
                cudaError_t return_value,
                void* device_offset_address,
                size_t num_bytes,
                int value) const;
            bool hasUploadError(
                cudaError_t return_value,
                void* device_offset_address,
                void* host_address,
                size_t num_bytes) const;
            bool hasDownloadError(
                cudaError_t return_value,
                void* device_offset_address,
                void* host_address,
                size_t num_bytes) const;
        protected:
            /** @brief Pointer to the device memory resource. */
            void* m_device_ptr;
            /** @brief Resource uid. */
            UID<ICudaResource> m_resource_uid;
        };

        /** 
         * @brief Simple wrapper for linear device memory. 
         * @tparam basetype Type of the underlying data elements.  
         */
        template <typename basetype>
        class CudaPtr : public ICudaResource
        {
        public:
            CudaPtr() : m_count{ 0 }{ }
            /** 
             * @brief Allocates device memory for a given number of basetype 
             *        elements.
             * @param[in] count The number of basetype elements that fit the 
             *                  allocated linear array on the device.
             * @returns false, if an error occured.
             */
            bool allocate(size_t count = 1);
            /** @brief Inherited via ldplab::rtscuda::ICudaResource */
            bool free() override;
            /**
             * @brief Sets the allocated device memory to the given value.
             * @param[in] val Value to which the device memory is set.
             * @returns false, if an error occured.
             */
            bool memset(int val) { return memset(val, 0, m_count); }
            /**
             * @brief Sets the allocated device memory to the given value.
             * @param[in] val Value to which the device memory is set.
             * @param[in] offset The memory offset in elements.
             * @param[in] count The number of elements which memory is set to 
             *                  val.
             * @returns false, if an error occured.
             */
            bool memset(int val, size_t offset, size_t count);
            /**
             * @brief Uploads memory from a host source.
             * @param[in] src Pointer to the source memory.
             * @returns false, if an error occured.
             */
            bool upload(basetype* src) { return upload(src, 0, m_count); }
            /**
             * @brief Uploads memory from a host source.
             * @param[in] src Pointer to the source memory.
             * @param[in] offset The offset on the device memory in basetype
             *                   elements.
             * @param[in] count The number of basetype elements that are
             *                  uploaded from src.
             * @returns false, if an error occured.
             */
            bool upload(basetype* src, size_t offset, size_t count);
            /**
             * @brief Downloads memory to a host destination.
             * @param[in] dst Pointer to the destination in host memory.
             * @returns false, if an error occured.
             */
            bool download(basetype* dst) { return download(dst, 0, m_count); }
            /**
             * @brief Downloads memory to a host destination.
             * @param[in] dst Pointer to the destination in host memory.
             * @param[in] offset The offset on the device memory in basetype
             *                   elements.
             * @param[in]count The number of basetype elements that are 
             *                 downloaded to dst.
             * @returns false, if an error occured.
             */
            bool download(basetype* dst, size_t offset, size_t count);
            /** @brief Provides the device resource as basetype pointer. */
            inline basetype* get() const
            { return static_cast<basetype*>(m_device_ptr); }
            /** @brief Provides the number of allocated elements. */
            inline size_t count() const { return m_count; }
        protected:
            /** @brief Inherited via ldplab::rtscuda::ICudaResource */
            virtual const char* resourceTypeName() const override 
            { return "CudaPtr"; }
        private:
            size_t m_count;
        };

        /** 
         * @brief A memory region of undefined internal structure of a given
         *        size.
         */
        class CudaBlob : public CudaPtr<uint8_t>
        {
        protected:
            /** @brief Inherited via ldplab::rtscuda::CudaPtr<uint8_t> */
            const char* resourceTypeName() const override
            { return "CudaBlob"; }
        };

        template<typename basetype>
        inline bool CudaPtr<basetype>::allocate(size_t count)
        {
            const size_t size = sizeof(basetype) * count;
            if (hasAllocationError(cudaMalloc(&m_device_ptr, size), size))
                return false;
            m_count = count;
            return true;
        }

        template<typename basetype>
        inline bool CudaPtr<basetype>::free()
        {
            if (m_device_ptr == NULL)
                return true;
            const size_t size = sizeof(basetype) * m_count;
            if (hasFreeError(cudaFree(m_device_ptr), size))
                return false;
            m_device_ptr = 0;
            m_count = 0;
            return true;
        }

        template<typename basetype>
        inline bool CudaPtr<basetype>::memset(
            int val, 
            size_t offset, 
            size_t count)
        {
            const void* device_ptr = static_cast<void*>(
                static_cast<basetype*>(m_device_ptr) + offset);
            const size_t num_bytes = sizeof(basetype) * count;
            const cudaError_t rv = cudaMemset(
                device_ptr,
                val,
                num_bytes);
            return !hasMemsetError(rv, device_ptr, num_bytes, val);
        }

        template<typename basetype>
        inline bool CudaPtr<basetype>::upload(
            basetype* src, 
            size_t offset, 
            size_t count)
        {
            const void* device_ptr = static_cast<void*>(
                static_cast<basetype*>(m_device_ptr) + offset);
            const void* src_ptr = static_cast<void*>(src);
            const size_t num_bytes = sizeof(basetype) * count;
            const cudaError_t rv = cudaMemcpy(
                device_ptr,
                src_ptr,
                num_bytes,
                cudaMemcpyHostToDevice);
            return !hasUploadError(rv, device_ptr, src_ptr, num_bytes);
        }

        template<typename basetype>
        inline bool CudaPtr<basetype>::download(
            basetype* dst, 
            size_t offset, 
            size_t count)
        {
            const void* dst_ptr = static_cast<void*>(dst);
            const void* device_ptr = static_cast<void*>(
                static_cast<basetype*>(m_device_ptr) + offset);
            const size_t num_bytes = sizeof(basetype) * count;
            const cudaError_t rv = cudaMemcpy(
                dst_ptr,
                device_ptr,
                num_bytes,
                cudaMemcpyDeviceToHost);
            return !hasDownloadError(rv, device_ptr, dst_ptr, num_bytes);
        }
    }
}

#endif //LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP