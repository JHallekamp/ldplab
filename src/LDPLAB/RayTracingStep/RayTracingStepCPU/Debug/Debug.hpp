#ifndef WWU_LDPLAB_RTSCPU_DEBUG_HPP
#define WWU_LDPLAB_RTSCPU_DEBUG_HPP

#ifdef LDPLAB_BUILD_FRAMEWORK_DEBUG
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "..//Context.hpp"
#include "..//..//..//Geometry.hpp"
namespace ldplab
{
    namespace rtscpu
    {
        class Debug
        {
        public:
            enum class ForceType
            {
                reflected,
                transmitted
            };
        public:

            static void RtsExecutionStart(std::shared_ptr<Context> ctx);
            static void RtsExecutionFinish();
            static void RtsBatchStart();
            
            static void SetActiveRay(size_t ray_id);
            static void MarkActiveRayIntersectionCap();
            static void AddForce(Vec3 force, ForceType force_type, size_t ray_id);

        private:
            Debug();
            static Debug& instance();
        private:
            std::shared_ptr<Context> m_context;
            
            std::ofstream m_file_reflected_force_cap;
            std::ofstream m_file_reflected_force_shell;
            std::ofstream m_file_transmitted_force_cap;
            std::ofstream m_file_transmitted_force_shell;
            size_t m_execution_ctr;

            size_t m_active_ray;
            std::vector<bool> m_ray_cap_mark;

            Vec3 m_force_reflected_cap;
            Vec3 m_force_reflected_shell;
            Vec3 m_force_transmitted_cap;
            Vec3 m_force_transmitted_shell;
        };
    }
}
#endif // LDPLAB_FRAMEWORK_DEBUG

#endif