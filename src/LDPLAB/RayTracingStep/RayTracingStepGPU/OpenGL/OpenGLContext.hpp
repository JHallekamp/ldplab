#ifndef WWU_LDPLAB_RTSGPU_OGL_OPEN_GL_CONTEXT_HPP
#define WWU_LDPLAB_RTSGPU_OGL_OPEN_GL_CONTEXT_HPP

#include <memory>
#include <string>

#include <GL/glew.h>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /** @brief Encapsulates an OpenGL shader program. */
        struct ComputeShader
        {
            ComputeShader();
            /** @brief Destructs the shader program. */
            ~ComputeShader();
            /** @brief Implicit conversion into GLuint. */
            operator GLuint() const { return glid; }
            /** @brief OpenGL ID of the shader program. */
            GLuint glid;
            /** @brief The name of the shader program. */
            std::string name;
        };

        /**
         * @brief Provides an internal interface to OpenGL and manages the 
         *        necessary data and context.
         */
        class OpenGLContext
        {
        public:
            OpenGLContext(std::shared_ptr<Context> context);
            /** 
             * @brief Initializes glew, opengl and context data. 
             * @returns true if the initialization was successful.
             */
            bool init();
            /**
             * @brief Creates a compute shader program.
             * @param[in] shader_name The name of the glsl shader.
             * @param[in] glsl_code The glsl shader code of the compute shader.
             * @param[out] shader The target container for the compiled 
             *                    shader program.
             * @returns true if the shader program was created successfully.
             */
            bool CreateComputeShader(
                const std::string& shader_name,
                const std::string& glsl_code, 
                ComputeShader& shader) const;
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif