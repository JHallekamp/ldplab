#ifndef WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP
#define WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstddef>
#include <cuda_runtime.h>

#include "../../Utils/Log.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        template<typename T>
        class CudaPtr
        {
        public:
            CudaPtr() : m_device_ptr{ 0 } { }
            ~CudaPtr() { free(); }
            bool allocate();
            void free();
            bool memset(int val);
            bool upload(const T* src);
            bool download(T* dest);
            inline T* get() { return m_device_ptr; }
        private:
            T* m_device_ptr;
        };

        template<typename T>
        class CudaLinearArray
        {
        public:
            CudaLinearArray() : m_device_ptr{ 0 }, m_size{ 0 } { }
            ~CudaLinearArray() { free(); }
            bool allocate(size_t size);
            void free();
            bool memset(int val);
            bool memset(int val, size_t offset, size_t size);
            bool upload(const T* src);
            bool upload(const T* src, size_t offset, size_t size);
            bool download(T* dst);
            bool download(T* dst, size_t offset, size_t size);
            inline size_t size() { return m_size; }
            inline T* get() { return m_device_ptr; }
        private:
            T* m_device_ptr;
            size_t m_size;
        };

        template<typename T>
        class CudaPitchedPtr
        {
        public:
            CudaPitchedPtr() : 
                m_device_ptr{ 0 }, 
                m_width{ 0 }, 
                m_height{ 0 }, 
                m_pitch{ 0 } 
            { }
            ~CudaPitchedPtr() { free(); }
            bool allocate(size_t width, size_t height);
            void free();
            bool memset(int val);
            bool upload(const T* src);
            bool download(T* dst);
            inline size_t width() { return m_width; }
            inline size_t height() { return m_height; }
            inline size_t pitch() { return m_pitch; }
            inline T* get() { return m_device_ptr; }
        private:
            T* m_device_ptr;
            size_t m_width;
            size_t m_height;
            size_t m_pitch;
        };

        template<typename T>
        inline bool CudaPtr<T>::allocate()
        {
            if (cudaMalloc(&m_device_ptr, sizeof(T)) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to allocate %i "\
                    "bytes of device memory",
                    sizeof(T));
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Allocated %i bytes of memory "\
                "at device address %p",
                sizeof(T),
                (void*) m_device_ptr);
            return true;
        }

        template<typename T>
        inline void CudaPtr<T>::free()
        {
            if (m_device_ptr != 0)
            {
                LDPLAB_LOG_TRACE("RTSCUDA resource: Freed %i bytes of memory "\
                    "at device address %p",
                    sizeof(T),
                    (void*)m_device_ptr);
                cudaFree(m_device_ptr);
                m_device_ptr = 0;
            }
        }

        template<typename T>
        inline bool CudaPtr<T>::memset(int val)
        {
            if (cudaMemset(
                (void*)m_device_ptr, 
                val, 
                sizeof(T)) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed set %i bytes of "\
                    "device memory to value %i at address %p",
                    sizeof(T),
                    val,
                    (void*) m_device_ptr);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Set %i bytes of memory "\
                "at device address %p to value %i",
                sizeof(T),
                (void*)m_device_ptr,
                val);
            return true;
        }

        template<typename T>
        inline bool CudaPtr<T>::upload(const T* src)
        {
            if (cudaMemcpy(
                (void*)m_device_ptr,
                (const void*)src,
                sizeof(T),
                cudaMemcpyHostToDevice) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to upload %i "\
                    "bytes from host address %p to device address %p",
                    sizeof(T),
                    (void*)m_device_ptr,
                    (void*)src);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Uploaded %i bytes from "\
                "host address %p to device address %p",
                sizeof(T),
                (void*)m_device_ptr,
                (void*)src);
            return true;
        }

        template<typename T>
        inline bool CudaPtr<T>::download(T* dest)
        {
            if (cudaMemcpy(
                (void*)src,
                (const void*)m_device_ptr,
                sizeof(T),
                cudaMemcpyDeviceToHost) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to download %i "\
                    "bytes from device address %p to host address %p",
                    sizeof(T),
                    (void*)src,
                    (void*)m_device_ptr);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Downloaded %i bytes from "\
                "device address %p to host address %p",
                sizeof(T),
                (void*)src,
                (void*)m_device_ptr);
            return true;
        }

        template<typename T>
        inline bool CudaLinearArray<T>::allocate(size_t size)
        {
            if (cudaMalloc(&m_device_ptr, sizeof(T) * size) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to allocate %i "\
                    "bytes of device memory",
                    sizeof(T) * size);
                return false;
            }
            m_size = size;
            LDPLAB_LOG_TRACE("RTSCUDA resource: Allocated %i bytes of memory "\
                "at device address %p",
                sizeof(T) * size,
                (void*)m_device_ptr);
            return true;
        }

        template<typename T>
        inline void CudaLinearArray<T>::free()
        {
            if (m_device_ptr != 0)
            {
                LDPLAB_LOG_TRACE("RTSCUDA resource: Freed %i bytes of memory "\
                    "at device address %p",
                    sizeof(T) * m_size,
                    (void*)m_device_ptr);
                cudaFree(m_device_ptr);
                m_device_ptr = 0;
                m_size = 0;
            }
        }

        template<typename T>
        inline bool CudaLinearArray<T>::memset(int val)
        {
            return memset(val, 0, m_size);
        }

        template<typename T>
        inline bool CudaLinearArray<T>::memset(int val, size_t offset, size_t size)
        {
            if (cudaMemset(
                (void*)(m_device_ptr + offset), 
                val, 
                sizeof(T) * size) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed set %i bytes of "\
                    "device memory to value %i at address %p with an offset "\
                    "of %i bytes",
                    sizeof(T) * size,
                    val,
                    (void*)m_device_ptr,
                    sizeof(T) * offset);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Set %i bytes of memory "\
                "at device address %p with an offset of %i bytes to value %i",
                sizeof(T) * size,
                (void*)m_device_ptr,
                sizeof(T) * offset,
                val);
            return true;
        }

        template<typename T>
        inline bool CudaLinearArray<T>::upload(const T* src)
        {
            return upload(src, 0, m_size);
        }

        template<typename T>
        inline bool CudaLinearArray<T>::upload(const T* src, size_t offset, size_t size)
        {
            if (cudaMemcpy(
                (void*)(m_device_ptr + offset), 
                (const void*)src, 
                sizeof(T) * size,
                cudaMemcpyHostToDevice);
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed upload %i bytes "\
                    "from host address %p to device address %p with an offset "\
                    "of %i bytes",
                    sizeof(T) * size,
                    (void*)src,
                    (void*)m_device_ptr,
                    sizeof(T) * offset);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Uploaded %i bytes of from "\
                "host address %p to device address %p with an offset of %i "\
                "bytes",
                sizeof(T) * size,
                (void*)src,
                (void*)m_device_ptr,
                sizeof(T) * offset);
            return true;
        }

        template<typename T>
        inline bool CudaLinearArray<T>::download(T* dst)
        {
            return download(dst, 0, m_size);
        }

        template<typename T>
        inline bool CudaLinearArray<T>::download(T* dst, size_t offset, size_t size)
        {
            if (cudaMemcpy(
                (void*)src,
                (const void*)(m_device_ptr + offset),
                sizeof(T) * size,
                cudaMemcpyDeviceToHost);
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed download %i bytes "\
                    "from device address %p to host address %p with an offset "\
                    "of %i bytes",
                    sizeof(T) * size,
                    (void*)m_device_ptr,
                    (void*)src,
                    sizeof(T) * offset);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Downloaded %i bytes of from "\
                "device address %p to host address %p with an offset of %i "\
                "bytes",
                sizeof(T) * size,
                (void*)m_device_ptr,
                (void*)src,
                sizeof(T) * offset);
            return true;
        }

        template<typename T>
        inline bool CudaPitchedPtr<T>::allocate(size_t width, size_t height)
        {
            if (cudaMallocPitch(
                (void**)&m_device_ptr,
                &m_pitch,
                sizeof(T) * width,
                height) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to allocate "\
                    "device memory of %i bytes width and height %i",
                    sizeof(T) * width,
                    height);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Allocated device memory of "\
                "%i bytes width and height %i at device address %p",
                m_pitch,
                height,
                (void*)m_device_ptr);
            m_width = width;
            m_height = height;
            return true;
        }

        template<typename T>
        inline void CudaPitchedPtr<T>::free()
        {
            LDPLAB_LOG_TRACE("RTSCUDA resource: Freed device memory of "\
                "%i bytes width and height %i at device address %p",
                m_pitch,
                height,
                (void*)m_device_ptr);
            cudaFree((void*)m_device_ptr);
            m_device_ptr = 0;
            m_width = m_height = m_pitch = 0;
        }

        template<typename T>
        inline bool CudaPitchedPtr<T>::memset(int val)
        {
            if (cudaMemset2D(
                (void*)m_device_ptr,
                m_pitch,
                val,
                sizeof(T) * m_width,
                m_height) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to set device "\
                    "memory of %i bytes width and height %i to value %i at "\
                    "device address %p",
                    sizeof(T) * width,
                    height,
                    val,
                    (void*)m_device_ptr);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Set device memory of %i bytes "\
                "width and height %i to value %i at device address %p",
                sizeof(T) * width,
                height,
                val,
                (void*)m_device_ptr);
            return true;
        }

        template<typename T>
        inline bool CudaPitchedPtr<T>::upload(const T* src)
        {
            if (cudaMemcpy2D(
                (void*)m_device_ptr,
                m_pitch,
                (const void*)src,
                sizeof(T) * m_width,
                sizeof(T) * m_width,
                m_height,
                cudaMemcpyHostToDevice) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to upload memory "\
                    "of %i bytes width and height %i from host address %p to "\
                    "device address %p",
                    m_pitch,
                    m_height,
                    (void*)src,
                    (void*)m_device_ptr);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Uploaded memory of %i bytes "\
                "width and height %i from host address %p to device address %p",
                m_pitch,
                m_height,
                (void*)src,
                (void*)m_device_ptr);
            return true;
        }

        template<typename T>
        inline bool CudaPitchedPtr<T>::download(T* dst)
        {
            if (cudaMemcpy2D(
                (void*)src,
                sizeof(T) * m_width,
                (const void*)m_device_ptr,
                m_pitch,
                sizeof(T) * m_width,
                m_height,
                cudaMemcpyDeviceToHost) != cudaSuccess)
            {
                LDPLAB_LOG_ERROR("RTSCUDA resource: Failed to download memory "\
                    "of %i bytes width and height %i from device address %p to "\
                    "host address %p",
                    m_pitch,
                    m_height,
                    (void*)m_device_ptr,
                    (void*)src);
                return false;
            }
            LDPLAB_LOG_TRACE("RTSCUDA resource: Downloaded memory of %i bytes "\
                "width and height %i from device address %p to host address %p",
                m_pitch,
                m_height,
                (void*)m_device_ptr,
                (void*)src);
            return true;
        }
}
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_CUDA_RESOURCE_HPP