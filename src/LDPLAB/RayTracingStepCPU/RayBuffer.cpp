#include "RayBuffer.hpp"

ldplab::RayBuffer::RayBuffer(size_t size)
    :
    m_size(size),
    m_buffer(new Ray[size])
{
}
