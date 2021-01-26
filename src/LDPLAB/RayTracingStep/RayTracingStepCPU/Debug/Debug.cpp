#ifdef LDPLAB_FRAMEWORK_DEBUG
#include "Debug.hpp"

ldplab::rtscpu::Debug& ldplab::rtscpu::Debug::instance()
{
    static Debug my_instance;
    return my_instance;
}

std::ofstream& ldplab::rtscpu::Debug::getOfstream(std::string file)
{
    std::map<std::string, std::ofstream>::iterator it =
        m_ofstreams.find(file);
    if (it == m_ofstreams.end())
    {
        std::ofstream t{ file };
        m_ofstreams[file] = std::move(t);
        return m_ofstreams[file];
    }
    else
        return it->second;
}

std::uint64_t& ldplab::rtscpu::Debug::getUint64(std::string name)
{
    std::map<std::string, std::uint64_t>::iterator it =
        m_uints.find(name);
    if (it == m_uints.end())
    {
        std::uint64_t t = 0;
        m_uints[name] = std::move(t);
        return m_uints[name];
    }
    else
        return it->second;
}
#endif