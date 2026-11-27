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

# Dear ImGui draws the graph editor. The tag is the release of the master branch (not the docking one); like volk it is
# compiled below as part of this project, so its own files are not built by FetchContent.
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9b
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR do-not-build)

FetchContent_MakeAvailable(glfw glm VulkanHeaders volk glslang imgui)

add_library(volk STATIC ${volk_SOURCE_DIR}/volk.c)
target_include_directories(volk PUBLIC ${volk_SOURCE_DIR})
target_link_libraries(volk PUBLIC Vulkan::Headers)

# ImGui with the GLFW and Vulkan backends. The renderer loads Vulkan through volk, so the Vulkan backend must call volk's
# functions too (IMGUI_IMPL_VULKAN_USE_VOLK) and not the loader's. The demo window is not compiled.
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_VULKAN_USE_VOLK GLFW_INCLUDE_NONE)
target_link_libraries(imgui PUBLIC glfw volk)

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
target_link_libraries(XCOM123 PRIVATE glfw glm::glm volk imgui)
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

# material_gen writes the maps of a procedural material as PNG files. It builds only the material sources, the PNG
# writer and the job system.
file(GLOB MATERIAL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/material/*.cpp)
add_executable(material_gen tools/material_gen/main.cpp ${MATERIAL_SOURCES} src/core/PngWriter.cpp src/core/JobSystem.cpp)
target_include_directories(material_gen PRIVATE src)
target_link_libraries(material_gen PRIVATE glm::glm)
target_compile_definitions(material_gen PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(material_gen PRIVATE /W4 /permissive- /MP)
endif()

# character_gen makes a character and writes a contact sheet and an OBJ file. It builds only the character sources,
# the PNG writer and the job system.
file(GLOB CHARACTER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/character/*.cpp)
add_executable(character_gen tools/character_gen/main.cpp ${CHARACTER_SOURCES} src/core/PngWriter.cpp src/core/JobSystem.cpp)
target_include_directories(character_gen PRIVATE src)
target_link_libraries(character_gen PRIVATE glm::glm)
target_compile_definitions(character_gen PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(character_gen PRIVATE /W4 /permissive- /MP)
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
add_executable(physics_tests tests/PhysicsTests.cpp src/core/JobSystem.cpp ${PHYSICS_SOURCES} ${FLUID_SOURCES})
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

# material_tests checks the procedural materials: they are repeatable, tile without a seam, have unit normals that
# follow the height, stay within physical ranges and carry detail at every scale.
add_executable(material_tests tests/MaterialTests.cpp src/core/JobSystem.cpp ${MATERIAL_SOURCES})
target_include_directories(material_tests PRIVATE src)
target_link_libraries(material_tests PRIVATE glm::glm)
target_compile_definitions(material_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(material_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME material_tests COMMAND material_tests)

# character_tests checks the character generator: a closed surface, the stature and mass asked for, human proportions,
# symmetry, skin weights, skinning and bad parameters.
add_executable(character_tests tests/CharacterTests.cpp src/core/JobSystem.cpp ${CHARACTER_SOURCES})
target_include_directories(character_tests PRIVATE src)
target_link_libraries(character_tests PRIVATE glm::glm)
target_compile_definitions(character_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(character_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME character_tests COMMAND character_tests)

# graph_tests checks the visual scripting core: typed pins, the interpreter, validation and saving.
file(GLOB GRAPH_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/graph/*.cpp)
add_executable(graph_tests tests/GraphTests.cpp ${GRAPH_SOURCES})
target_include_directories(graph_tests PRIVATE src)
target_link_libraries(graph_tests PRIVATE glm::glm)
target_compile_definitions(graph_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(graph_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME graph_tests COMMAND graph_tests)

# graph_editor_tests checks the model behind the graph editor: selection, moving, wires, the palette, copy and paste, undo, play
# and saving. It builds only the model sources (src/graph_editor, not src/editor_ui) and the graph, so it needs no GPU or window.
file(GLOB GRAPH_EDITOR_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_editor/*.cpp)
add_executable(graph_editor_tests tests/GraphEditorTests.cpp ${GRAPH_EDITOR_SOURCES} ${GRAPH_SOURCES})
target_include_directories(graph_editor_tests PRIVATE src)
target_link_libraries(graph_editor_tests PRIVATE glm::glm)
target_compile_definitions(graph_editor_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(graph_editor_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME graph_editor_tests COMMAND graph_editor_tests)

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

# ---------------------------------------------------------------------------
# Animation: procedural motion for the characters of src/character
# ---------------------------------------------------------------------------
# anim_gen draws stick figure contact sheets of a walk, and animation_tests checks the module. Both build the animation
# sources with the skeleton and the parameters of the character, and nothing else of src/character (no mesh generator).
file(GLOB ANIMATION_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/animation/*.cpp)
set(ANIMATION_CHARACTER_SOURCES src/character/Skeleton.cpp src/character/CharacterParameters.cpp)
add_executable(anim_gen tools/anim_gen/main.cpp src/core/PngWriter.cpp ${ANIMATION_SOURCES} ${ANIMATION_CHARACTER_SOURCES})
add_executable(animation_tests tests/AnimationTests.cpp ${ANIMATION_SOURCES} ${ANIMATION_CHARACTER_SOURCES})
foreach(animation_target anim_gen animation_tests)
    target_include_directories(${animation_target} PRIVATE src)
    target_link_libraries(${animation_target} PRIVATE glm::glm)
    target_compile_definitions(${animation_target} PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
    if(MSVC)
        target_compile_options(${animation_target} PRIVATE /W4 /permissive- /MP)
    endif()
endforeach()
add_test(NAME animation_tests COMMAND animation_tests)

# skinning_tests checks the CPU side of skinned characters: the blend the vertex shader does, palette layout and offsets, weight
# normalisation, bounds and culling. It builds the skinning math and the character sources, so it needs no GPU or window.
add_executable(skinning_tests tests/SkinningTests.cpp src/render/SkinningMath.cpp src/core/JobSystem.cpp ${CHARACTER_SOURCES})
target_include_directories(skinning_tests PRIVATE src)
target_link_libraries(skinning_tests PRIVATE glm::glm)
target_compile_definitions(skinning_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(skinning_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME skinning_tests COMMAND skinning_tests)

# ---------------------------------------------------------------------------
# Engine nodes: what a graph can do to the engine
# ---------------------------------------------------------------------------
# graph_engine_tests runs the engine nodes and the example graphs of assets/graphs against a host that records its calls, and the
# editor model's play with that host. It builds the graph, the editor model and src/graph_engine, and nothing of the engine itself.
file(GLOB GRAPH_ENGINE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_engine/*.cpp)
add_executable(graph_engine_tests tests/GraphEngineTests.cpp ${GRAPH_ENGINE_SOURCES} ${GRAPH_SOURCES} ${GRAPH_EDITOR_SOURCES})
target_include_directories(graph_engine_tests PRIVATE src)
target_link_libraries(graph_engine_tests PRIVATE glm::glm)
target_compile_definitions(graph_engine_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE
    XCOM_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
if(MSVC)
    target_compile_options(graph_engine_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME graph_engine_tests COMMAND graph_engine_tests)

# panel_model_tests checks the models of the generator panels: the numbers of the controls, and the character and material generators
# running in the background. It builds src/panel_model with the generators and the job system, and no ImGui.
file(GLOB PANEL_MODEL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/panel_model/*.cpp)
add_executable(panel_model_tests tests/PanelModelTests.cpp ${PANEL_MODEL_SOURCES} ${CHARACTER_SOURCES} ${MATERIAL_SOURCES} src/core/JobSystem.cpp)
target_include_directories(panel_model_tests PRIVATE src)
target_link_libraries(panel_model_tests PRIVATE glm::glm)
target_compile_definitions(panel_model_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(panel_model_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME panel_model_tests COMMAND panel_model_tests)

# streaming_tests checks the streamed world: the chunk generator (pure per chunk, shared borders, budgets) and the streaming
# manager (ordering, budgets, teleports, threads). It builds the city builders, the crowd and the streaming sources, no GPU.
file(GLOB STREAMING_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/streaming/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/world/stream/*.cpp)
add_executable(streaming_tests tests/StreamingTests.cpp tests/StreamingManagerTests.cpp ${STREAMING_SOURCES}
    src/core/JobSystem.cpp src/world/Crowd.cpp src/world/TerrainField.cpp ${CITY_GENERATOR_SOURCES} ${UNIT_TEST_CROWD_SOURCES})
target_include_directories(streaming_tests PRIVATE src)
target_link_libraries(streaming_tests PRIVATE glm::glm)
target_compile_definitions(streaming_tests PRIVATE GLM_FORCE_RADIANS GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(MSVC)
    target_compile_options(streaming_tests PRIVATE /W4 /permissive- /MP)
endif()
add_test(NAME streaming_tests COMMAND streaming_tests)
