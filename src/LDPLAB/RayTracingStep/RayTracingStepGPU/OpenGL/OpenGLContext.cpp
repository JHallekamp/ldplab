#include "OpenGLContext.hpp"

#include "Context.hpp"
#include "../../../Log.hpp"

#include <vector>

ldplab::rtsgpu_ogl::ComputeShader::ComputeShader()
    :
    glid{ 0 }
{ }

ldplab::rtsgpu_ogl::ComputeShader::~ComputeShader()
{
    glDeleteProgram(glid);
}

ldplab::rtsgpu_ogl::OpenGLContext::OpenGLContext(
    std::shared_ptr<Context> context)
    :
    m_context{ context }
{ 
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: OpenGLContext instance "\
        "created", m_context->uid);
}

bool ldplab::rtsgpu_ogl::OpenGLContext::init()
{ 
    if (glewInit() != GLEW_OK)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Failed to initialize "\
            "OpenGLContext, could not initialize GLEW", m_context->uid);
        return false;
    }

    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: OpenGLContext initalized "\
        "successful", m_context->uid);
    return true;
}

bool ldplab::rtsgpu_ogl::OpenGLContext::CreateComputeShader(
    const std::string& shader_name,
    const std::string& glsl_code, 
    ComputeShader& shader) const
{
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Begins %s compute shader "\
        "compilation", m_context->uid, shader_name.c_str());

    // Compile the compute shader
    GLuint compute_shader = glCreateShader(GL_COMPUTE_SHADER);
    const GLchar* src = glsl_code.c_str();
    GLint src_length = static_cast<GLint>(glsl_code.length());
    glShaderSource(compute_shader, 1, &src, &src_length);
    glCompileShader(compute_shader);

    // Check for errors
    GLint result;
    GLsizei info_log_length;
    glGetShaderiv(compute_shader, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE)
    {
        // Errors
        glGetShaderiv(compute_shader, GL_INFO_LOG_LENGTH, &info_log_length);
        std::vector<GLchar> error_log(info_log_length, 0);
        glGetShaderInfoLog(
            compute_shader, info_log_length, NULL, error_log.data());
        std::string log = "RTSGPU (OpenGL) context %i: Failed to compile "\
            "%s compute shader, " + std::string(error_log.data());
        LDPLAB_LOG_ERROR(log.c_str(), m_context->uid, shader_name.c_str());
        return false;
    }

    // Link program
    GLuint program = glCreateProgram();
    glAttachShader(program, compute_shader);
    glLinkProgram(program);

    // Check for errors
    glGetProgramiv(program, GL_LINK_STATUS, &result);
    if (result == GL_FALSE)
    {
        // Errors
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &info_log_length);
        std::vector<GLchar> error_log(info_log_length, 0);
        glGetProgramInfoLog(program, info_log_length, NULL, error_log.data());
        std::string log = "RTSGPU (OpenGL) context %i: Failed to link "\
            "%s compute shader program, " + std::string(error_log.data());
        LDPLAB_LOG_ERROR(log.c_str(), m_context->uid, shader_name.c_str());
    }
    else
    {
        // No errors
        shader.glid = program;
        shader.name = shader_name;
        LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Finished %s compute "\
            "shader compilation successfully", 
            m_context->uid, shader_name.c_str());
    }

    // Clean up and return
    glDetachShader(program, compute_shader);
    glDeleteShader(compute_shader);
    return (result != GL_FALSE);
}

