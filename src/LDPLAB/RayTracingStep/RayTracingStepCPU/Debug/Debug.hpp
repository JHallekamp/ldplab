#ifndef WWU_LDPLAB_RTSCPU_DEBUG_HPP
#define WWU_LDPLAB_RTSCPU_DEBUG_HPP

#ifdef LDPLAB_BUILD_FRAMEWORK_DEBUG
#include <fstream>
#include <map>
#include <string>
namespace ldplab
{
    namespace rtscpu
    {
        class Debug
        {
        public:
            static Debug& instance();
            std::ofstream& getOfstream(std::string file);
            std::uint64_t& getUint64(std::string name);
            std::string getUint64AsString(std::string name);
            double& getDouble(std::string name);
        private:
            Debug() { }
        private:
            std::map<std::string, std::ofstream> m_ofstreams;
            std::map<std::string, std::uint64_t> m_uints;
            std::map<std::string, double> m_doubles;
        };
    }
}
#endif // LDPLAB_FRAMEWORK_DEBUG

#endif