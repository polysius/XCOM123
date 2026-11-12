cmake_minimum_required(VERSION 3.24)
project(XCOM123 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Some dependencies declare very old minimum CMake versions; this keeps CMake 4.x from rejecting them.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ---------------------------------------------------------------------------
# Third-party libraries, downloaded at configure time at pinned versions.
# No Vulkan SDK is required: the Vulkan runtime ships with the GPU driver,
# volk loads it at startup, and glslang compiles our shaders during the build.
# ---------------------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# glslang's optimizer needs SPIRV-Tools, which we do not need for this project.
set(ENABLE_OPT OFF CACHE BOOL "" FORCE)
set(GLSLANG_TESTS OFF CACHE BOOL "" FORCE)
set(GLSLANG_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(ENABLE_GLSLANG_BINARIES ON CACHE BOOL "" FORCE)

FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)

FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE)

FetchContent_Declare(VulkanHeaders
    GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
    GIT_TAG vulkan-sdk-1.3.296.0
    GIT_SHALLOW TRUE)

# volk is compiled below as part of this project, so its own CMakeLists is skipped
# by pointing SOURCE_SUBDIR at a folder that does not exist.
FetchContent_Declare(volk
    GIT_REPOSITORY https://github.com/zeux/volk.git
    GIT_TAG vulkan-sdk-1.3.296.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR do-not-build)

FetchContent_Declare(glslang
    GIT_REPOSITORY https://github.com/KhronosGroup/glslang.git
    GIT_TAG 15.0.0
    GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(glfw glm VulkanHeaders volk glslang)

add_library(volk STATIC ${volk_SOURCE_DIR}/volk.c)
target_include_directories(volk PUBLIC ${volk_SOURCE_DIR})
target_link_libraries(volk PUBLIC Vulkan::Headers)

# ---------------------------------------------------------------------------
# Shaders: GLSL in shaders/ is compiled to SPIR-V in the build folder.
# ---------------------------------------------------------------------------
set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)
# Every .vert, .frag and .comp file becomes one SPIR-V binary. Shared code lives in .glsl
# files that the shaders #include, so a change to any of them rebuilds every shader.
file(GLOB SHADER_SOURCES CONFIGURE_DEPENDS
    ${SHADER_SOURCE_DIR}/*.vert ${SHADER_SOURCE_DIR}/*.frag ${SHADER_SOURCE_DIR}/*.comp)
file(GLOB SHADER_INCLUDES CONFIGURE_DEPENDS ${SHADER_SOURCE_DIR}/*.glsl)

set(SHADER_BINARIES "")
foreach(SHADER_SOURCE ${SHADER_SOURCES})
    get_filename_component(SHADER_NAME ${SHADER_SOURCE} NAME)
    set(SHADER_BINARY ${SHADER_OUTPUT_DIR}/${SHADER_NAME}.spv)
    add_custom_command(
        OUTPUT ${SHADER_BINARY}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_OUTPUT_DIR}
        COMMAND $<TARGET_FILE:glslang-standalone> -V --target-env vulkan1.3
                -I${SHADER_SOURCE_DIR} -o ${SHADER_BINARY} ${SHADER_SOURCE}
        DEPENDS ${SHADER_SOURCE} ${SHADER_INCLUDES} glslang-standalone
        COMMENT "Compiling shader ${SHADER_NAME}"
        VERBATIM)
    list(APPEND SHADER_BINARIES ${SHADER_BINARY})
endforeach()
add_custom_target(shaders DEPENDS ${SHADER_BINARIES})

# ---------------------------------------------------------------------------
# The engine executable
# ---------------------------------------------------------------------------
# Every .cpp file under src/ is part of the engine. CONFIGURE_DEPENDS makes the build notice
# new or removed files without re-running CMake by hand.
file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE ENGINE_HEADERS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
add_executable(XCOM123 ${ENGINE_SOURCES} ${ENGINE_HEADERS})

target_include_directories(XCOM123 PRIVATE src)
target_link_libraries(XCOM123 PRIVATE glfw glm::glm volk)
target_compile_definitions(XCOM123 PRIVATE
    GLFW_INCLUDE_NONE
    GLM_FORCE_RADIANS
    GLM_FORCE_DEPTH_ZERO_TO_ONE
    XCOM_SHADER_DIR="${SHADER_OUTPUT_DIR}")
add_dependencies(XCOM123 shaders)

if(MSVC)
    target_compile_options(XCOM123 PRIVATE /W4 /permissive- /MP)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT XCOM123)
endif()

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
# crowd_bench times the crowd simulation without a window or a GPU. It builds the world
# code (city and crowd) and the job system, nothing that needs Vulkan or GLFW.
file(GLOB_RECURSE CROWD_BENCH_WORLD_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/world/*.cpp)
file(GLOB PHYSICS_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/physics/*.cpp)
file(GLOB FLUID_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/fluid/*.cpp)
add_executable(crowd_bench
    tools/crowd_bench/main.cpp
    ${CROWD_BENCH_WORLD_SOURCES}
    ${PHYSICS_SOURCES}
    ${FLUID_SOURCES}
    src/core/JobSystem.cpp)
target_include_directories(crowd_bench PRIVATE src)
target_link_libraries(crowd_bench PRIVATE glm::glm)
target_compile_definitions(crowd_bench PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(crowd_bench PRIVATE /W4 /permissive- /MP)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
# unit_tests covers the logic that needs no window or GPU. `ctest` runs it, and the
# no-mistakes gate runs ctest through scripts\gate-check.cmd.
enable_testing()
# The unit tests also drive the crowd on small cities made by hand, so they build all of it.
file(GLOB UNIT_TEST_CROWD_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/world/crowd/*.cpp)
add_executable(unit_tests
    tests/unit_tests.cpp
    src/core/CommandLine.cpp
    src/core/FrameStatistics.cpp
    src/core/JobSystem.cpp
    src/world/Crowd.cpp
    src/world/TerrainField.cpp
    ${UNIT_TEST_CROWD_SOURCES}
    ${PHYSICS_SOURCES}
    ${FLUID_SOURCES})
target_include_directories(unit_tests PRIVATE src)
target_link_libraries(unit_tests PRIVATE glm::glm)
target_compile_definitions(unit_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(unit_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME unit_tests COMMAND unit_tests)

# physics_tests checks the rigid body physics: falling, resting, stacking, bouncing, friction, sleeping.
add_executable(physics_tests tests/PhysicsTests.cpp ${PHYSICS_SOURCES} ${FLUID_SOURCES})
target_include_directories(physics_tests PRIVATE src)
target_link_libraries(physics_tests PRIVATE glm::glm)
target_compile_definitions(physics_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(physics_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME physics_tests COMMAND physics_tests)

# fluid_tests checks the shallow water simulation: still water, wave speed, volume, slopes, dry ground.
add_executable(fluid_tests tests/FluidTests.cpp ${FLUID_SOURCES})
target_include_directories(fluid_tests PRIVATE src)
target_link_libraries(fluid_tests PRIVATE glm::glm)
target_compile_definitions(fluid_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(fluid_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME fluid_tests COMMAND fluid_tests)

# ---------------------------------------------------------------------------
# City generator tests
# ---------------------------------------------------------------------------
# A console program that checks the generated city against the rules the crowd relies on, and
# can draw a top-down map of it (CityTests --map city.png). It builds only the generator
# sources (src/world/City*.cpp) and glm, so it runs without Vulkan or a window.
file(GLOB CITY_GENERATOR_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/world/City*.cpp)
add_executable(CityTests tests/CityTests.cpp tests/PngImage.cpp tests/PngImage.h src/world/TerrainField.cpp ${CITY_GENERATOR_SOURCES})
target_include_directories(CityTests PRIVATE src)
target_link_libraries(CityTests PRIVATE glm::glm)
target_compile_definitions(CityTests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(CityTests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME CityTests COMMAND CityTests)
