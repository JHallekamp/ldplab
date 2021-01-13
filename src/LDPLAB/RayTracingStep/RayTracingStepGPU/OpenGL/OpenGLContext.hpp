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
        class ComputeShader
        {
            friend class OpenGLContext;
        public:
            ComputeShader();
            /** @brief Destructs the shader program. */
            ~ComputeShader();
            /**
             * @brief Checks whether the shader is initialized correctly.
             * @returns true, if the shader is initialized correctly.
             */
            bool isInitialized() const;
            /**
             * @brief Provides the name of the shader.
             * @returns The name as a null terminated C string.
             */
            const char* name() const;
            /**
             * @brief Provides the location of the specified shader uniform.
             * @param[in] name The name of the uniform buffer.
             * @returns If successful it returns the location of the uniform
             *          inside the shader program, otherwise -1.
             */
            GLint getUniformLocation(const std::string& name) const;
            /** @brief Uses shader in the current OpenGL rendering state. */
            void use() const;
        private:
            std::shared_ptr<Context> m_context;
            /** @brief OpenGL ID of the shader program. */
            GLuint m_glid;
            /** @brief The name of the shader program. */
            std::string m_name;
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
            bool createComputeShader(
                const std::string& shader_name,
                const std::string& glsl_code, 
                ComputeShader& shader) const;
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif