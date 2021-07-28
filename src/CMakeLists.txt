#file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${LDPLAB_SOURCE_DIR}/include/LDPLAB/*.hpp")

set(HEADER_ROOT
	"${LDPLAB_SOURCE_DIR}/include/ldplab.hpp"
)

set(HEADER_LDPLAB
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/BuildInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Constants.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Geometry.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/LogCallback.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/Profiling.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/SimulationState.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/UID.hpp"
)
set(SOURCE_LDPLAB
	BuildInfo.cpp
    LogCallback.cpp
    Profiling.cpp
)

set(HEADER_LDPLAB_EXPERIMENTAL_SETUP
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/BoundingVolume.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightDirection.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightDistribution.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/LightPolarization.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Lightsource.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Particle.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ParticleGeometry.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/ParticleMaterial.hpp"
)

set(HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Utils/PropertyGenerator.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/ExperimentalSetup/Utils/ObjLoader.hpp"
)
set(SOURCE_EXPERIMENTAL_SETUP_UTILS
	ExperimentalSetup/Utils/PropertyGenerator.cpp
	ExperimentalSetup/Utils/ObjLoader.cpp
)

set(HEADER_LDPLAB_RAY_TRACING_STEP
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/AcceleratorStructureParameter.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/EikonalSolverParameter.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/IRayTracingStep.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepFactory.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepOpenGLInfo.hpp"
    "${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/RayTracingStepOutput.hpp"
)
set(SOURCE_RAY_TRACING_STEP
	RayTracingStep/RayTracingStepFactory.cpp
)

set(HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/Data.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultBoundingVolumeIntersectionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultGenericGeometryFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultParticleIntersectionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IBoundingVolumeIntersection.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IInitialStage.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IInnerParticlePropagation.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IParticleIntersection.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/PipelineConfiguration.hpp"
	"${LDPLAB_SOURCE_DIR}/include/LDPLAB/RayTracingStep/CPU/StageFactories.hpp"
)

set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
    RayTracingStep/CPU/Factory.hpp
    RayTracingStep/CPU/ImplBoundingVolumeIntersection.hpp
    RayTracingStep/CPU/ImplGenericGeometry.hpp
    RayTracingStep/CPU/ImplInitialStage.hpp
    RayTracingStep/CPU/ImplInnerParticlePropagation.hpp
    RayTracingStep/CPU/ImplParticleIntersection.hpp
    RayTracingStep/CPU/ImplSurfaceInteraction.hpp
    RayTracingStep/CPU/IntersectionTests.hpp
    RayTracingStep/CPU/MemoryControl.hpp
    RayTracingStep/CPU/Pipeline.hpp
    RayTracingStep/CPU/RayTracingStepCPU.hpp
)
set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU
    RayTracingStep/CPU/DefaultBoundingVolumeIntersectionFactories.cpp
    RayTracingStep/CPU/DefaultGenericGeometryFactories.cpp
    RayTracingStep/CPU/DefaultInitialStageFactories.cpp
    RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.cpp
    RayTracingStep/CPU/DefaultParticleIntersectionFactories.cpp
    RayTracingStep/CPU/DefaultSurfaceInteractionFactories.cpp
    RayTracingStep/CPU/Factory.cpp
    RayTracingStep/CPU/ImplBoundingVolumeIntersection.cpp
    RayTracingStep/CPU/ImplGenericGeometry.cpp
    RayTracingStep/CPU/ImplInitialStage.cpp
    RayTracingStep/CPU/ImplInnerParticlePropagation.cpp
    RayTracingStep/CPU/ImplParticleIntersection.cpp
    RayTracingStep/CPU/ImplSurfaceInteraction.cpp
    RayTracingStep/CPU/IntersectionTests.cpp
    RayTracingStep/CPU/IPipelineStage.cpp
    RayTracingStep/CPU/MemoryControl.cpp
    RayTracingStep/CPU/Pipeline.cpp
    RayTracingStep/CPU/RayTracingStepCPU.cpp
)

if (LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
		RayTracingStep/CUDA/Context.hpp
		RayTracingStep/CUDA/CudaResource.hpp
		RayTracingStep/CUDA/Data.hpp
		RayTracingStep/CUDA/Factory.hpp
		RayTracingStep/CUDA/GenericBoundingVolume.hpp
		RayTracingStep/CUDA/GenericParticleGeometry.hpp
		RayTracingStep/CUDA/GenericParticleMaterial.hpp
		RayTracingStep/CUDA/IntersectionTests.hpp
		RayTracingStep/CUDA/Pipeline.hpp
		RayTracingStep/CUDA/PipelineBoundingVolumeIntersection.hpp
		RayTracingStep/CUDA/PipelineBufferSetup.hpp
		RayTracingStep/CUDA/PipelineGatherOutput.hpp
		RayTracingStep/CUDA/PipelineInitial.hpp
		RayTracingStep/CUDA/PipelineInnerParticlePropagation.hpp
		RayTracingStep/CUDA/PipelineParticleInteraction.hpp
		RayTracingStep/CUDA/PipelineParticleIntersection.hpp
		RayTracingStep/CUDA/PipelineRayBufferReduce.hpp
		RayTracingStep/CUDA/RayTracingStep.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA
		RayTracingStep/CUDA/CudaResource.cu
		RayTracingStep/CUDA/Data.cu
		RayTracingStep/CUDA/Factory.cu
		RayTracingStep/CUDA/GenericBoundingVolume.cu
		RayTracingStep/CUDA/GenericParticleGeometry.cu
		RayTracingStep/CUDA/GenericParticleMaterial.cu
		RayTracingStep/CUDA/IntersectionTests.cu
		RayTracingStep/CUDA/Pipeline.cu
		RayTracingStep/CUDA/PipelineBoundingVolumeIntersection.cu
		RayTracingStep/CUDA/PipelineBufferSetup.cu
		RayTracingStep/CUDA/PipelineGatherOutput.cu
		RayTracingStep/CUDA/PipelineInitial.cu
		RayTracingStep/CUDA/PipelineInnerParticlePropagation.cu
		RayTracingStep/CUDA/PipelineParticleInteraction.cu
		RayTracingStep/CUDA/PipelineParticleIntersection.cu
		RayTracingStep/CUDA/PipelineRayBufferReduce.cu
		RayTracingStep/CUDA/RayTracingStep.cu
    )
else()
	set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
	set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/OpenGL/BufferControl.hpp
        RayTracingStep/OpenGL/Constants.hpp
        RayTracingStep/OpenGL/Context.hpp
        RayTracingStep/OpenGL/Data.hpp
        RayTracingStep/OpenGL/Factory.hpp
        RayTracingStep/OpenGL/InitialStage.hpp
        RayTracingStep/OpenGL/InnerParticlePropagationStage.hpp
        RayTracingStep/OpenGL/OpenGLContext.hpp
        RayTracingStep/OpenGL/Pipeline.hpp
        RayTracingStep/OpenGL/RayBoundingVolumeIntersectionTestStage.hpp
        RayTracingStep/OpenGL/RayParticleInteractionStage.hpp
        RayTracingStep/OpenGL/RayParticleIntersectionTestStage.hpp
        RayTracingStep/OpenGL/RayTracingStep.hpp
        RayTracingStep/OpenGL/SharedShaders.hpp
    )
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL
    	RayTracingStep/OpenGL/BufferControl.cpp
        RayTracingStep/OpenGL/Data.cpp
        RayTracingStep/OpenGL/Factory.cpp
        RayTracingStep/OpenGL/InitialStage.cpp
        RayTracingStep/OpenGL/InnerParticlePropagationStage.cpp
        RayTracingStep/OpenGL/OpenGLContext.cpp
        RayTracingStep/OpenGL/Pipeline.cpp
        RayTracingStep/OpenGL/RayBoundingVolumeIntersectionTestStage.cpp
        RayTracingStep/OpenGL/RayParticleInteractionStage.cpp
        RayTracingStep/OpenGL/RayParticleIntersectionTestStage.cpp
        RayTracingStep/OpenGL/RayTracingStep.cpp
        RayTracingStep/OpenGL/SharedShaders.cpp
    )
else()
    set(HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
    set(SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL)
endif()

set(HEADER_INTERN_UTILS
	Utils/Array.hpp
    Utils/Assert.hpp
    Utils/ComputeHelper.hpp
    Utils/Log.hpp
    Utils/Profiler.hpp
    Utils/ThreadPool.hpp
)

set(SOURCE_UTILS
	Utils/ComputeHelper.cpp
    Utils/Log.cpp
    Utils/Profiler.cpp
    Utils/ThreadPool.cpp
)

add_library(ldplab STATIC
    ${HEADER_ROOT}
    ${HEADER_LDPLAB}
	${SOURCE_LDPLAB}
    ${HEADER_LDPLAB_EXPERIMENTAL_SETUP}
    ${HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS}
    ${SOURCE_EXPERIMENTAL_SETUP_UTILS}
    ${HEADER_LDPLAB_RAY_TRACING_STEP}
    ${SOURCE_RAY_TRACING_STEP}
	${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA}
    ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL}
    ${HEADER_INTERN_UTILS}
    ${SOURCE_UTILS}
)

source_group("include" FILES ${HEADER_LDPLAB})
source_group("include\\ExperimentalSetup" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP})
source_group("include\\ExperimentalSetup\\Utils" FILES ${HEADER_LDPLAB_EXPERIMENTAL_SETUP_UTILS})
source_group("include\\RayTracingStep" FILES ${HEADER_LDPLAB_RAY_TRACING_STEP})
source_group("include\\RayTracingStep\\CPU" FILES ${HEADER_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("src\\ExperimentalSetup\\Utils" FILES ${SOURCE_EXPERIMENTAL_SETUP_UTILS})
source_group("src\\RayTracingStep" FILES ${SOURCE_RAY_TRACING_STEP})
source_group("src\\Utils" FILES ${HEADER_INTERN_UTILS})
source_group("src\\Utils" FILES ${SOURCE_UTILS})
source_group("src\\RayTracingStep\\CPU" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})
source_group("src\\RayTracingStep\\CPU" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CPU})

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    source_group("src\\RayTracingStep\\CUDA" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
    source_group("src\\RayTracingStep\\CUDA" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_CUDA})
endif()
if (LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    source_group("src\\RayTracingStep\\OpenGL" FILES ${HEADER_INTERN_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
    source_group("src\\RayTracingStep\\OpenGL" FILES ${SOURCE_RAY_TRACING_STEP_RAY_TRACING_STEP_OPENGL})
endif()

target_compile_features(ldplab PUBLIC cxx_std_14)

target_include_directories(ldplab
    PUBLIC
        ../include
        ../external/glm
		../external/tinyobjloader
)

target_link_libraries(ldplab
    PRIVATE
        glm
        Threads::Threads
)

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
	set_target_properties(ldplab PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
	set_target_properties(ldplab PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS  ON)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_PROFILING)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_ENABLE_PROFILING)
endif()

if(LDPLAB_BUILD_OPTION_DISABLE_LOGGING)
    target_compile_definitions(ldplab PRIVATE LDPLAB_BUILD_OPTION_DISABLE_LOGGING)
endif()

if (TINYOBJLOADER_USE_DOUBLE)
	target_compile_definitions(ldplab PRIVATE TINYOBJLOADER_USE_DOUBLE)
endif()

target_compile_definitions(ldplab PRIVATE "$<$<CONFIG:Debug>:LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING>")
