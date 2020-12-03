#ifndef WWU_LDPLAB_RAY_BUFFER__HPP
#define WWU_LDPLAB_RAY_BUFFER__HPP

#include "..\Geometry.hpp"
#include <memory>

namespace ldplab
{

    struct Ray
    {
        Vec3 origin;
        Vec3 direction;
        double intensity;
    };

    class RayBuffer
    {
    public:
        RayBuffer(size_t size);
        size_t size() const { return m_size; };
        Ray operator[](size_t index) { return m_buffer[index]; }
    private:
        std::shared_ptr<Ray[]> m_buffer;
        const size_t m_size;
    };
}

#endif