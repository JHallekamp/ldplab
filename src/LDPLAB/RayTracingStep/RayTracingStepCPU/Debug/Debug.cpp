#ifdef LDPLAB_BUILD_FRAMEWORK_DEBUG
#include "Debug.hpp"
#include <sstream>

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

std::string ldplab::rtscpu::Debug::getUint64AsString(std::string name)
{
    std::stringstream ss;
    ss << getUint64(name);
    return ss.str();
}

double& ldplab::rtscpu::Debug::getDouble(std::string name)
{
    std::map<std::string, double>::iterator it =
        m_doubles.find(name);
    if (it == m_doubles.end())
    {
        double t = 0.0;
        m_doubles[name] = std::move(t);
        return m_doubles[name];
    }
    else
        return it->second;
}
#endif