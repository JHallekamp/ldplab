#ifdef LDPLAB_BUILD_FRAMEWORK_DEBUG
#include "Debug.hpp"
#include <sstream>



void ldplab::rtscpu::Debug::RtsExecutionStart(std::shared_ptr<Context> ctx)
{
    Debug& me = instance();
    if (me.m_context == nullptr)
    {
        // First execution.
        me.m_context = ctx;
        me.m_file_reflected_force_cap = std::ofstream{ "debug_force_reflected_cap" };
        me.m_file_reflected_force_shell = std::ofstream{ "debug_force_reflected_shell" };
        me.m_file_transmitted_force_cap = std::ofstream{ "debug_force_transmitted_cap" };
        me.m_file_transmitted_force_shell = std::ofstream{ "debug_force_transmitted_shell" };
        me.m_ray_cap_mark.resize(me.m_context->parameters.number_rays_per_buffer, false);
        me.m_execution_ctr = 0;
    }

    me.m_force_reflected_cap = Vec3(0);
    me.m_force_reflected_shell = Vec3(0);
    me.m_force_transmitted_cap = Vec3(0);
    me.m_force_transmitted_shell = Vec3(0);
}

void ldplab::rtscpu::Debug::RtsExecutionFinish()
{
    Debug& me = instance();

    me.m_file_reflected_force_cap << me.m_execution_ctr << '\t' <<
        me.m_force_reflected_cap.x << '\t' <<
        me.m_force_reflected_cap.y << '\t' <<
        me.m_force_reflected_cap.z << '\t' << std::endl;

    me.m_file_reflected_force_shell << me.m_execution_ctr << '\t' <<
        me.m_force_reflected_shell.x << '\t' <<
        me.m_force_reflected_shell.y << '\t' <<
        me.m_force_reflected_shell.z << '\t' << std::endl;

    me.m_file_transmitted_force_cap << me.m_execution_ctr << '\t' <<
        me.m_force_transmitted_cap.x << '\t' <<
        me.m_force_transmitted_cap.y << '\t' <<
        me.m_force_transmitted_cap.z << '\t' << std::endl;

    me.m_file_transmitted_force_shell << me.m_execution_ctr << '\t' <<
        me.m_force_transmitted_shell.x << '\t' <<
        me.m_force_transmitted_shell.y << '\t' <<
        me.m_force_transmitted_shell.z << '\t' << std::endl;

    ++me.m_execution_ctr;
}

void ldplab::rtscpu::Debug::RtsBatchStart()
{
    Debug& me = instance();
    me.m_active_ray = 0;
    for (size_t i = 0; i < me.m_ray_cap_mark.size(); ++i)
        me.m_ray_cap_mark[i] = false;
}

void ldplab::rtscpu::Debug::SetActiveRay(size_t ray_id)
{
    Debug& me = instance();
    me.m_active_ray = ray_id;
}

void ldplab::rtscpu::Debug::MarkActiveRayIntersectionCap()
{
    Debug& me = instance();
    me.m_ray_cap_mark[me.m_active_ray] = true;
}

void ldplab::rtscpu::Debug::AddForce(Vec3 force, ForceType force_type, size_t ray_id)
{
    Debug& me = instance();
    bool cap = me.m_ray_cap_mark[ray_id];

    if (cap && force_type == ForceType::reflected)
        me.m_force_reflected_cap += force;
    else if (force_type == ForceType::reflected)
        me.m_force_reflected_shell += force;
    else if (cap)
        me.m_force_transmitted_cap += force;
    else
        me.m_force_transmitted_shell += force;
}

ldplab::rtscpu::Debug::Debug()
    :
    m_context{ nullptr },
    m_execution_ctr{ 0 }
{
}

ldplab::rtscpu::Debug& ldplab::rtscpu::Debug::instance()
{
    static Debug my_instance;
    return my_instance;
}

#endif