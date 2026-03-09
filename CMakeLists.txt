cmake_minimum_required(VERSION 3.10)
project(mujoco_drones C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── MuJoCo ──────────────────────────────────────────────────────────
# Expect MUJOCO_DIR to point at the MuJoCo install root, or fall back
# to ~/.mujoco/mujoco
if(NOT DEFINED MUJOCO_DIR)
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR
        "MuJoCo not found at ${MUJOCO_DIR}. "
        "Set -DMUJOCO_DIR=/path/to/mujoco")
endif()

message(STATUS "MuJoCo: ${MUJOCO_DIR}")

# ── GLFW (for interactive viewer) ───────────────────────────────────
find_package(glfw3 QUIET)
find_package(OpenGL QUIET)

# ── Build ───────────────────────────────────────────────────────────
add_executable(hummingbird
    src/main.c
    src/controller.c
    src/viewer.c
)

target_include_directories(hummingbird PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${MUJOCO_DIR}/include
)

target_link_directories(hummingbird PRIVATE
    ${MUJOCO_DIR}/lib
)

target_link_libraries(hummingbird PRIVATE
    mujoco
    m
)

# Link GLFW + OpenGL if available
if(glfw3_FOUND AND OpenGL_FOUND)
    target_link_libraries(hummingbird PRIVATE
        glfw
        OpenGL::GL
    )
    message(STATUS "GLFW + OpenGL found → interactive viewer enabled")
else()
    message(STATUS "GLFW or OpenGL not found → headless only")
    target_compile_definitions(hummingbird PRIVATE NO_GLFW)
endif()

# RPATH so the binary finds libmujoco.so at runtime
set_target_properties(hummingbird PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib"
    INSTALL_RPATH "${MUJOCO_DIR}/lib"
)

# Copy model directory next to binary for convenience
add_custom_command(TARGET hummingbird POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/model
        ${CMAKE_BINARY_DIR}/model
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/meshes
        ${CMAKE_BINARY_DIR}/meshes
    COMMENT "Symlinking model/ and meshes/ into build directory"
)
