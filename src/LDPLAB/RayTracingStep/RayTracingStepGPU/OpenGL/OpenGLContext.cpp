#include "OpenGLContext.hpp"

#include "Context.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Assert.hpp"

#include <vector>

ldplab::rtsgpu_ogl::ComputeShader::ComputeShader()
    :
    m_glid{ 0 }
{ }

ldplab::rtsgpu_ogl::ComputeShader::~ComputeShader()
{
    glDeleteProgram(m_glid);
}

const char* ldplab::rtsgpu_ogl::ComputeShader::name() const
{
    LDPLAB_ASSERT(isInitialized());
    return m_name.c_str();
}

GLint ldplab::rtsgpu_ogl::ComputeShader::getUniformLocation(
    const std::string& name) const
{
    LDPLAB_ASSERT(isInitialized());
    GLint uniform_location = glGetUniformLocation(m_glid, name.c_str());
    if (uniform_location < 0)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Shader %s does not "\
            "have a uniform buffer %s",
            m_context->uid, m_name.c_str(), name.c_str());
    }
    else
    {
        LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Shader %s provides "\
            "uniform buffer %s at location %i",
            m_context->uid, m_name.c_str(), name.c_str(), uniform_location);
    }
    return uniform_location;
}

void ldplab::rtsgpu_ogl::ComputeShader::use() const
{
    LDPLAB_ASSERT(isInitialized());
    glUseProgram(m_glid);
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Now uses compute shader %s",
        m_context->uid, m_name.c_str());
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

std::shared_ptr<ldplab::rtsgpu_ogl::ComputeShader> 
ldplab::rtsgpu_ogl::OpenGLContext::createComputeShader(
    const std::string& shader_name,
    const std::string& glsl_code) const
{
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Begins %s compute shader "\
        "compilation", m_context->uid, shader_name.c_str());

    std::shared_ptr<ComputeShader> shader{ nullptr };

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
        return shader;
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
        shader = std::shared_ptr<ComputeShader>(new ComputeShader());
        shader->m_context = m_context;
        shader->m_glid = program;
        shader->m_name = shader_name;
        LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Finished %s compute "\
            "shader compilation successfully", 
            m_context->uid, shader_name.c_str());
    }

    // Clean up and return
    glDetachShader(program, compute_shader);
    glDeleteShader(compute_shader);
    return shader;
}

std::shared_ptr<ldplab::rtsgpu_ogl::ShaderStorageBuffer> 
ldplab::rtsgpu_ogl::OpenGLContext::createShaderStorageBuffer(
    size_t buffer_size, 
    GLenum buffer_usage)
{
    std::shared_ptr<ShaderStorageBuffer> ssbo{ new ShaderStorageBuffer() };
    if (ssbo != nullptr)
    {
        ssbo->m_size = buffer_size;
        ssbo->m_usage = buffer_usage;
        glGenBuffers(1, &ssbo->m_glid);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo->m_glid);
        glBufferData(GL_SHADER_STORAGE_BUFFER, buffer_size, NULL, buffer_usage);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }
    return ssbo;
}

void ldplab::rtsgpu_ogl::ShaderStorageBuffer::bindToIndex(
    GLuint binding_index) const
{
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_index, m_glid);
}

void ldplab::rtsgpu_ogl::ShaderStorageBuffer::upload(
    size_t offset, 
    size_t size, 
    void* data)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_glid);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);
}

void ldplab::rtsgpu_ogl::ShaderStorageBuffer::upload(void* data)
{
    upload(0, m_size, data);
}

ldplab::rtsgpu_ogl::ShaderStorageBuffer::ShaderStorageBuffer()
    :
    m_glid{ 0 },
    m_size{ 0 },
    m_usage{ 0 }
{ }

ldplab::rtsgpu_ogl::ShaderStorageBuffer::~ShaderStorageBuffer()
{
    glDeleteBuffers(1, &m_glid);
}
