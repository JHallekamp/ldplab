#ifndef WWU_LDPLAB_UTILS_ARRAY_HPP
#define WWU_LDPLAB_UTILS_ARRAY_HPP

#include <cstddef>
#include <memory>
#include <vector>

namespace ldplab
{
    namespace utils
    {
        /**
         * @brief Wrapper around dynamic arrays, similar to std::vector but 
         *        with fixed length.
         * @tparam Type of the managed data.
         */
        template<typename T>
        class Array
        {
        public:
            inline Array();
            inline Array(size_t size);
            inline Array(const Array<T>& obj);
            inline Array(const std::vector<T>& obj);
            inline Array(Array<T>&& obj);
            inline ~Array() noexcept;
            /** 
             * @brief Resizes the array. 
             * @details Discards all the data.
             * @param[in] size The new size of the array.
             */
            inline void resize(size_t size);
            /** @brief Current size of the vector. */
            inline size_t size() const noexcept { return m_size; }
            /** @brief Array access. */
            inline T& operator[](size_t index);
            /** @brief Const array access. */
            inline const T& operator[](size_t index) const;
        private:
            inline void alloc(size_t size);
            inline void dealloc() noexcept;
        private:
            T* m_data;
            size_t m_size;
        };

        template<typename T>
        inline Array<T>::Array()
            :
            m_data{ nullptr },
            m_size{ 0 }
        { }

        template<typename T>
        inline Array<T>::Array(size_t size)
        {
            alloc(size);
        }

        template<typename T>
        inline Array<T>::Array(const Array<T>& obj)
        {
            if (obj.size())
            {
                alloc(obj.size());
                std::memcpy(m_data, obj.m_data, sizeof(T) * m_size);
            }
        }

        template<typename T>
        inline Array<T>::Array(const std::vector<T>& obj)
        {
            if (obj.size())
            {
                alloc(obj.size());
                std::memcpy(m_data, obj.data(), sizeof(T) * m_size);
            }
        }

        template<typename T>
        inline Array<T>::Array(Array<T>&& obj)
            :
            m_data{ obj.m_data },
            m_size{ obj.m_size }
        {
            obj.m_data = nullptr;
            obj.m_size = 0;
        }

        template<typename T>
        inline Array<T>::~Array() noexcept
        {
            dealloc();
        }

        template<typename T>
        inline void Array<T>::resize(size_t size)
        {
            dealloc();
            alloc(size);
        }

        template<typename T>
        inline T& Array<T>::operator[](size_t index)
        {
            return m_data[index];
        }

        template<typename T>
        inline const T& Array<T>::operator[](size_t index) const
        {
            return m_data[index];
        }

        template<typename T>
        inline void Array<T>::alloc(size_t size)
        {
            m_data = new T[size];
            m_size = size;
        }
        
        template<typename T>
        inline void Array<T>::dealloc() noexcept
        {
            if (m_data)
            {
                delete[] m_data;
                m_size = 0;
            }
        }
    }
}

#endif