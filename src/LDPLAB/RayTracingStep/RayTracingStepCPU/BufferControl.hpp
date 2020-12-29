#ifndef WWU_LDPLAB_RTSCPU_BUFFER_CONTROL_HPP
#define WWU_LDPLAB_RTSCPU_BUFFER_CONTROL_HPP

#include "Data.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    namespace rtscpu
    {
        class Context;

        /**
         * @brief The buffer control is responsible for creating, assigning and
         *        destroying ray buffers.
         * @details In the RTSCPU pipeline, each ray can be split into two
         *          rays, a reflected and a transmitted ray. Therefore each
         *          buffer will eventually split into two, one for reflected
         *          and one for transmitted rays. Therefore the ray buffers
         *          resemble a binary tree structure and regardless of their
         *          underlying data structures will provide a tree like
         *          interface.
         */
        class BufferControl
        {
        public:
            BufferControl(std::shared_ptr<Context> context);
            /** 
             * @brief Provides the initial batch buffer (depth 0).
             * @returns The root buffer of the buffer tree.
             */
            RayBuffer& initialBuffer();
            /**
             * @brief Provides the reflection buffer to a given buffer.
             * @param[in] buffer A buffer given by the buffer control.
             * @returns The left child of the given buffer or a dummy buffer if
             *          the depth of buffer is greater or equal to the maximum
             *          depth.
             */
            RayBuffer& getReflectionBuffer(RayBuffer& buffer);
            /**
             * @brief Provides the transmission buffer to a given buffer.
             * @param[in] buffer A buffer given by the buffer control.
             * @returns The right child of the given buffer or a dummy buffer
             *          if the depth of buffer is greater or equal to the
             *          maximum depth.
             */
            RayBuffer& getTransmissionBuffer(RayBuffer& buffer);
            /**
             * @brief Provides a temporary intersection buffer. 
             * @warning This always provides the same buffer.
             * @returns The intersection buffer.
             */
            IntersectionBuffer& getIntersectionBuffer();
            /**
             * @brief Provides the buffer index to identify a dummy buffer.
             * @returns The dummy buffer index.
             */
            size_t dummyBufferIndex();
        private:
            void initializeBuffers();
        private:
            std::shared_ptr<Context> m_context;
            std::vector<RayBuffer> m_buffers;
            std::vector<Ray> m_ray_data;
            std::vector<int32_t> m_index_data;
            RayBuffer m_dummy;
            std::vector<Vec3> m_point_data;
            std::vector<Vec3> m_normal_data;
            IntersectionBuffer m_intersection_buffer;
        };
    }
}

#endif