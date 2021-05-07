#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL

#include "OpenGLContext.hpp"

#include "Context.hpp"
#include "../../../Utils/Log.hpp"
#include "../../../Utils/Assert.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

void ldplab::rtsogl::ComputeShader::execute(
    size_t work_group_size_x, 
    size_t work_group_size_y, 
    size_t work_group_size_z)
{
    glDispatchCompute(
        static_cast<GLuint>(work_group_size_x),
        static_cast<GLuint>(work_group_size_y),
        static_cast<GLuint>(work_group_size_z));
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

ldplab::rtsogl::ComputeShader::ComputeShader()
    :
    m_glid{ 0 }
{ }

ldplab::rtsogl::ComputeShader::~ComputeShader()
{
    glDeleteProgram(m_glid);
}

const char* ldplab::rtsogl::ComputeShader::name() const
{
    return m_name.c_str();
}

GLint ldplab::rtsogl::ComputeShader::getUniformLocation(
    const std::string& name) const
{
    GLint uniform_location = glGetUniformLocation(m_glid, name.c_str());
    if (uniform_location < 0)
    {
        LDPLAB_LOG_ERROR("RTSOGL context %i: Shader %s does not "\
            "have a uniform buffer %s",
            m_context.uid, m_name.c_str(), name.c_str());
    }
    else
    {
        LDPLAB_LOG_TRACE("RTSOGL context %i: Shader %s provides "\
            "uniform buffer %s at location %i",
            m_context.uid, m_name.c_str(), name.c_str(), uniform_location);
    }
    return uniform_location;
}

void ldplab::rtsogl::ComputeShader::use() const
{
    glUseProgram(m_glid);
    LDPLAB_LOG_DEBUG("RTSOGL context %i: Now uses compute shader %s",
        m_context.uid, m_name.c_str());
}

ldplab::rtsogl::OpenGLContext::OpenGLContext(
    Context& context)
    :
    m_context{ context },
    m_gl_offscreen_context{ nullptr }
{ 
    LDPLAB_LOG_INFO("RTSOGL context %i: OpenGLContext instance "\
        "created", m_context.uid);
}

ldplab::rtsogl::OpenGLContext::~OpenGLContext()
{
    shutdown();
}

bool ldplab::rtsogl::OpenGLContext::init()
{ 
    // Init GLFW
    if (!glfwInit())
    {
        LDPLAB_LOG_ERROR("RTSOGL context %i: Failed to initialize "\
            "GLFW", m_context.uid);
        return false;
    }

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    m_gl_offscreen_context = glfwCreateWindow(800, 600, "", NULL, NULL);
    if (!m_gl_offscreen_context)
    {
        LDPLAB_LOG_ERROR("RTSOGL context %i: Failed to create GLFW "\
            "offscreen window", m_context.uid);
        return false;
    }
    LDPLAB_LOG_DEBUG("RTSOGL Context %i: GLFW initialzed",
        m_context.uid);

    // Init Glew
    bindGlContext();
    glewExperimental = GL_TRUE;
    GLenum err_code = glewInit();
    if (err_code != GLEW_OK)
    {
        LDPLAB_LOG_ERROR("RTSOGL context %i: Failed to initialize "\
            "GLEW: %s", m_context.uid, glewGetErrorString(err_code));
        return false;
    }
    LDPLAB_LOG_DEBUG("RTSOGL Context %i: GLEW initialzed, uses "\
        "GLEW %s", m_context.uid, glewGetString(GLEW_VERSION));
    unbindGlContext();

    // Done
    LDPLAB_LOG_INFO("RTSOGL context %i: OpenGLContext initalized "\
        "successful", m_context.uid);
    return true;
}

void ldplab::rtsogl::OpenGLContext::shutdown()
{
    glfwDestroyWindow(m_gl_offscreen_context);
    glfwTerminate();
}

std::shared_ptr<ldplab::rtsogl::ComputeShader> 
ldplab::rtsogl::OpenGLContext::createComputeShader(
    const std::string& shader_name,
    const std::string& glsl_code) const
{
    LDPLAB_LOG_DEBUG("RTSOGL context %i: Begins %s compute shader "\
        "compilation", m_context.uid, shader_name.c_str());

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
        std::string log = "RTSOGL context %i: Failed to compile "\
            "%s compute shader, " + std::string(error_log.data());
        LDPLAB_LOG_ERROR(log.c_str(), m_context.uid, shader_name.c_str());
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
        std::string log = "RTSOGL context %i: Failed to link "\
            "%s compute shader program, " + std::string(error_log.data());
        LDPLAB_LOG_ERROR(log.c_str(), m_context.uid, shader_name.c_str());
    }
    else
    {
        // No errors
        shader = std::shared_ptr<ComputeShader>(new ComputeShader());
        shader->m_context = m_context;
        shader->m_glid = program;
        shader->m_name = shader_name;
        LDPLAB_LOG_DEBUG("RTSOGL context %i: Finished %s compute "\
            "shader compilation successfully", 
            m_context.uid, shader_name.c_str());
    }

    // Clean up and return
    glDetachShader(program, compute_shader);
    glDeleteShader(compute_shader);
    return shader;
}

std::shared_ptr<ldplab::rtsogl::ComputeShader> 
    ldplab::rtsogl::OpenGLContext::createComputeShaderFromFile(
        const std::string& shader_name, const std::string& path)
{
    // Load file
    std::ifstream file(path);
    if (!file)
    {
        LDPLAB_LOG_ERROR("RTSOGL context %i: Unable to open shader "\
            "source file \"%s\"", m_context.uid, path.c_str());
        return nullptr;
    }
    std::stringstream code;
    code << file.rdbuf();
    // Create and return shader
    return createComputeShader(shader_name, code.str());
}

std::shared_ptr<ldplab::rtsogl::ShaderStorageBuffer> 
ldplab::rtsogl::OpenGLContext::createShaderStorageBuffer(
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

void ldplab::rtsogl::OpenGLContext::bindGlContext()
{
    glfwMakeContextCurrent(m_gl_offscreen_context);
}

void ldplab::rtsogl::OpenGLContext::unbindGlContext()
{
    glfwMakeContextCurrent(nullptr);
}

void ldplab::rtsogl::ShaderStorageBuffer::bindToIndex(
    GLuint binding_index) const
{
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_index, m_glid);
}

void ldplab::rtsogl::ShaderStorageBuffer::upload(
    size_t offset, 
    size_t size, 
    const void* data)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_glid);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);
}

void ldplab::rtsogl::ShaderStorageBuffer::upload(const void* data)
{
    upload(0, m_size, data);
}

void ldplab::rtsogl::ShaderStorageBuffer::download(
    size_t offset,
    size_t size, 
    void* target)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_glid);
    GLvoid* ssbo_data = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
    if (ssbo_data != nullptr)
    {
        ssbo_data = (void*)(((char*)ssbo_data) + offset);
        std::memcpy(target, ssbo_data, size);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSOGL: Failed to download data from "\
            "SSBO", );
    }
}

void ldplab::rtsogl::ShaderStorageBuffer::download(void* target)
{
    download(0, m_size, target);
}

ldplab::rtsogl::ShaderStorageBuffer::ShaderStorageBuffer()
    :
    m_glid{ 0 },
    m_size{ 0 },
    m_usage{ 0 }
{ }

ldplab::rtsogl::ShaderStorageBuffer::~ShaderStorageBuffer()
{
    glDeleteBuffers(1, &m_glid);
}

#endif