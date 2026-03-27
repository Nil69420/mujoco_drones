cmake_minimum_required(VERSION 3.10)
project(mujoco_drones C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED MUJOCO_DIR)
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR
        "MuJoCo not found at ${MUJOCO_DIR}. "
        "Set -DMUJOCO_DIR=/path/to/mujoco")
endif()

message(STATUS "MuJoCo: ${MUJOCO_DIR}")

find_package(glfw3 QUIET)
find_package(OpenGL QUIET)
find_package(Threads REQUIRED)

option(ENABLE_IPC "Enable IPC transport (requires renoir)" ON)
option(ENABLE_FOXGLOVE "Enable Foxglove WebSocket bridge (requires IPC)" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

set(SOURCES
    src/main.c
    src/controller.c
    src/viewer.c
)

if(ENABLE_IPC)
    if(NOT DEFINED RENOIR_DIR)
        set(RENOIR_DIR "${CMAKE_SOURCE_DIR}/../renoir")
    endif()

    set(RENOIR_INCLUDE_DIR "${RENOIR_DIR}/target/include")
    set(RENOIR_LIB_DIR "${RENOIR_DIR}/target/release")

    if(NOT EXISTS "${RENOIR_LIB_DIR}/librenoir.so")
        set(RENOIR_LIB_DIR "${RENOIR_DIR}/target/debug")
    endif()

    if(NOT EXISTS "${RENOIR_INCLUDE_DIR}/renoir.h")
        message(FATAL_ERROR
            "Renoir header not found at ${RENOIR_INCLUDE_DIR}/renoir.h. "
            "Build renoir first: cd ${RENOIR_DIR} && cargo build --features c-api")
    endif()

    message(STATUS "Renoir IPC: ${RENOIR_DIR}")
    message(STATUS "Renoir lib: ${RENOIR_LIB_DIR}")

    list(APPEND SOURCES
        src/sensors/sensors.c
        src/transport/transport_renoir.c
    )

    if(ENABLE_FOXGLOVE)
        list(APPEND SOURCES src/foxglove/foxglove.c)
    endif()
endif()

add_executable(hummingbird ${SOURCES})

target_compile_options(hummingbird PRIVATE
    -Wall -Wextra -Wpedantic -Werror
    -Wshadow -Wdouble-promotion -Wformat=2
    $<$<C_COMPILER_ID:GNU>:-Wformat-overflow=2>
    $<$<C_COMPILER_ID:GNU>:-Wformat-truncation=2>
    -Wundef -Wconversion -Wsign-conversion
    -Wnull-dereference
    $<$<C_COMPILER_ID:GNU>:-Wimplicit-fallthrough=5>
    $<$<NOT:$<C_COMPILER_ID:GNU>>:-Wimplicit-fallthrough>
    -Wstrict-prototypes -Wold-style-definition
    -Wmissing-prototypes -Wmissing-declarations
    -Wstack-protector -Wvla
    -fstack-protector-strong
)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(hummingbird PRIVATE _FORTIFY_SOURCE=2)
endif()

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

if(glfw3_FOUND AND OpenGL_FOUND)
    target_link_libraries(hummingbird PRIVATE
        glfw
        OpenGL::GL
    )
    message(STATUS "GLFW + OpenGL found -> interactive viewer enabled")
else()
    message(STATUS "GLFW or OpenGL not found -> headless only")
    target_compile_definitions(hummingbird PRIVATE NO_GLFW)
endif()

if(ENABLE_IPC)
    target_include_directories(hummingbird PRIVATE ${RENOIR_INCLUDE_DIR})
    target_link_directories(hummingbird PRIVATE ${RENOIR_LIB_DIR})
    target_link_libraries(hummingbird PRIVATE renoir)
    target_compile_definitions(hummingbird PRIVATE ENABLE_IPC)

    if(ENABLE_FOXGLOVE)
        target_link_libraries(hummingbird PRIVATE Threads::Threads)
        target_compile_definitions(hummingbird PRIVATE ENABLE_FOXGLOVE)
        message(STATUS "Foxglove WebSocket bridge enabled")
    endif()

    set_target_properties(hummingbird PROPERTIES
        BUILD_RPATH "${RENOIR_LIB_DIR};${MUJOCO_DIR}/lib"
        INSTALL_RPATH "${RENOIR_LIB_DIR};${MUJOCO_DIR}/lib"
    )
else()
    set_target_properties(hummingbird PROPERTIES
        BUILD_RPATH "${MUJOCO_DIR}/lib"
        INSTALL_RPATH "${MUJOCO_DIR}/lib"
    )
endif()

add_custom_command(TARGET hummingbird POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/model
        ${CMAKE_BINARY_DIR}/model
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/meshes
        ${CMAKE_BINARY_DIR}/meshes
    COMMENT "Symlinking model/ and meshes/ into build directory"
)

if(ENABLE_ASAN)
    target_compile_options(hummingbird PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
        -fno-optimize-sibling-calls
        -fsanitize=float-divide-by-zero
        -fsanitize=float-cast-overflow
    )
    target_link_options(hummingbird PRIVATE -fsanitize=address,undefined)
    target_compile_definitions(hummingbird PRIVATE
        ASAN_OPTIONS="detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:detect_leaks=1"
        UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
    )
endif()

if(ENABLE_TSAN)
    target_compile_options(hummingbird PRIVATE
        -fsanitize=thread
        -fno-omit-frame-pointer
    )
    target_link_options(hummingbird PRIVATE -fsanitize=thread)
endif()

find_program(CLANG_TIDY clang-tidy)
if(CLANG_TIDY)
    set(TIDY_SRCS "")
    foreach(src ${SOURCES})
        list(APPEND TIDY_SRCS "${CMAKE_SOURCE_DIR}/${src}")
    endforeach()

    add_custom_target(lint
        COMMAND ${CLANG_TIDY}
            --extra-arg=-Wno-unknown-warning-option
            -p ${CMAKE_BINARY_DIR}
            ${TIDY_SRCS}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running clang-tidy..."
        VERBATIM
    )
endif()
