cmake_minimum_required(VERSION 3.10)
project(mujoco_drones C)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

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
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL QUIET)
find_package(Threads REQUIRED)

option(ENABLE_IPC "Enable IPC transport (requires renoir)" ON)
option(ENABLE_FOXGLOVE "Enable Foxglove WebSocket bridge (requires IPC)" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

set(SOURCES
    src/main.c
    src/cli.c
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
        src/sensors/sensors_init.c
        src/sensors/sensors_read.c
        src/sensors/sensors_camera.c
        src/sensors/camera_egl.c
        src/transport/transport_renoir.c
        src/transport/frame_out.c
    )

    if(ENABLE_FOXGLOVE)
        list(APPEND SOURCES
            src/foxglove/ws.c
            src/foxglove/proto.c
            src/foxglove/serialize.c
            src/foxglove/bridge.c
        )
    endif()

    list(APPEND VEHICLE_SOURCES
        src/vehicle/controller_registry.c
        src/vehicle/controllers/pd_multicopter.c
        src/vehicle/controllers/fixedwing_basic.c
        src/vehicle/controllers/omav_basic.c
        src/vehicle/estimator_registry.c
        src/vehicle/estimators/ground_truth.c
        src/vehicle/estimators/complementary_filter.c
        src/vehicle/estimators/kf_ins.c
        src/vehicle/estimators/kf_axis.c
        src/vehicle/localization_registry.c
        src/vehicle/localization/ground_truth.c
        src/vehicle/localization/gnss.c
        src/vehicle/airframe_registry.c
    src/vehicle/airframes/aslfixedwing_spec.c
    src/vehicle/airframes/quadx_spec.c
    src/vehicle/airframes/omav_spec.c
        src/vehicle/vehicle.c
        src/vehicle/assembly_config.c
        src/scenario_main.c
    )
    list(APPEND SOURCES ${VEHICLE_SOURCES})
endif()

add_executable(hummingbird ${SOURCES})

set(COMMON_COMPILE_OPTIONS
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

add_library(mdmodels STATIC
    src/conditions/atmosphere.c
    src/conditions/wind.c
    src/conditions/thermal.c
    src/conditions/vibration.c
    src/conditions/presets.c
    src/conditions/aero.c
    src/sensors/models/imu_model.c
    src/sensors/models/imu_profiles.c
    src/sensors/models/gnss_model.c
    src/sensors/models/gnss_profiles.c
    src/sensors/models/baro_model.c
    src/sensors/models/baro_profiles.c
    src/sensors/models/mag_model.c
    src/sensors/models/mag_profiles.c
    src/sensors/models/range_model.c
    src/sensors/models/range_profiles.c
    src/sensors/models/camera_model.c
    src/sensors/models/camera_profiles.c
    src/environments/terrain.c
    src/environments/biome.c
    src/environments/biome_scatter.c
    src/environments/biome_profiles.c
    src/environments/environment_registry.c
    src/foxglove/ws_proto.c
    src/foxglove/ops.c
    src/foxglove/sha1.c
    src/foxglove/base64.c
)
set_source_files_properties(src/sensors/models/camera_model.c PROPERTIES
    COMPILE_OPTIONS "-O3;-fno-math-errno;-ffinite-math-only;-fno-signed-zeros")
target_include_directories(mdmodels PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(mdmodels PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_directories(mdmodels PUBLIC ${MUJOCO_DIR}/lib)
target_include_directories(mdmodels PUBLIC ${MUJOCO_DIR}/include)
target_link_libraries(mdmodels PUBLIC m mujoco)

enable_testing()
add_executable(test_conditions tests/test_conditions.c)
target_include_directories(test_conditions PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_conditions PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_conditions PRIVATE mdmodels)
add_test(NAME conditions COMMAND test_conditions)

add_executable(test_imu_model tests/test_imu_model.c)
target_include_directories(test_imu_model PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_imu_model PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_imu_model PRIVATE mdmodels)
add_test(NAME imu_model COMMAND test_imu_model)

add_executable(test_gnss_model tests/test_gnss_model.c)
target_include_directories(test_gnss_model PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_gnss_model PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_gnss_model PRIVATE mdmodels)
add_test(NAME gnss_model COMMAND test_gnss_model)

add_executable(test_misc_models tests/test_misc_models.c)
target_include_directories(test_misc_models PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_misc_models PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_misc_models PRIVATE mdmodels)
add_test(NAME misc_models COMMAND test_misc_models)

add_executable(test_ws_proto tests/test_ws_proto.c)
target_include_directories(test_ws_proto PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_ws_proto PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_ws_proto PRIVATE mdmodels)
add_test(NAME ws_proto COMMAND test_ws_proto)

add_executable(test_fg_ops tests/test_fg_ops.c)
target_include_directories(test_fg_ops PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_fg_ops PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_fg_ops PRIVATE mdmodels)
add_test(NAME fg_ops COMMAND test_fg_ops)

add_executable(test_camera_model tests/test_camera_model.c)
target_include_directories(test_camera_model PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_camera_model PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_camera_model PRIVATE mdmodels)
add_test(NAME camera_model COMMAND test_camera_model)

add_executable(test_cli tests/test_cli.c src/cli.c)
target_include_directories(test_cli PRIVATE ${CMAKE_SOURCE_DIR}/tests ${CMAKE_SOURCE_DIR}/include)
target_compile_options(test_cli PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_cli PRIVATE mdmodels)
add_test(NAME cli COMMAND test_cli)

add_executable(test_import_config tests/test_import_config.c src/urdf_import_config.c)
target_include_directories(test_import_config PRIVATE ${CMAKE_SOURCE_DIR}/tests ${CMAKE_SOURCE_DIR}/src)
target_compile_options(test_import_config PRIVATE ${COMMON_COMPILE_OPTIONS})
add_test(NAME import_config COMMAND test_import_config)

add_executable(test_biomes tests/test_biomes.c)
target_include_directories(test_biomes PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_compile_options(test_biomes PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_biomes PRIVATE mdmodels)
add_test(NAME biomes COMMAND test_biomes WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(test_kf_ins tests/test_kf_ins.c
    src/vehicle/estimators/kf_ins.c
    src/vehicle/estimators/kf_axis.c
    src/vehicle/estimators/ground_truth.c
    src/vehicle/estimators/complementary_filter.c
    src/vehicle/estimator_registry.c)
target_include_directories(test_kf_ins PRIVATE ${CMAKE_SOURCE_DIR}/tests ${CMAKE_SOURCE_DIR}/include)
target_compile_options(test_kf_ins PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(test_kf_ins PRIVATE mdmodels)
add_test(NAME kf_ins COMMAND test_kf_ins)

target_compile_options(hummingbird PRIVATE ${COMMON_COMPILE_OPTIONS})

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(hummingbird PRIVATE _FORTIFY_SOURCE=2)
endif()

target_include_directories(hummingbird PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${MUJOCO_DIR}/include
)

target_link_directories(hummingbird PRIVATE
    ${MUJOCO_DIR}/lib
)

target_link_libraries(hummingbird PRIVATE
    mdmodels
    mujoco
    m
)

find_library(EGL_LIBRARY EGL)
find_path(EGL_INCLUDE_DIR EGL/egl.h)
if(EGL_LIBRARY AND EGL_INCLUDE_DIR)
    target_link_libraries(hummingbird PRIVATE ${EGL_LIBRARY})
    target_include_directories(hummingbird PRIVATE ${EGL_INCLUDE_DIR})
    target_compile_definitions(hummingbird PRIVATE HAVE_EGL)
    message(STATUS "EGL found -> headless camera enabled")
else()
    message(STATUS "EGL not found -> headless camera disabled")
endif()

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

    add_executable(send_command src/tools/send_command.c)
    target_sources(send_command PRIVATE src/transport/transport_renoir.c)
    target_compile_options(send_command PRIVATE ${COMMON_COMPILE_OPTIONS})
    target_include_directories(send_command PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${RENOIR_INCLUDE_DIR}
    )
    target_link_directories(send_command PRIVATE ${RENOIR_LIB_DIR})
    target_link_libraries(send_command PRIVATE renoir)

    add_executable(sensor_tap src/tools/sensor_tap.c)
    target_sources(sensor_tap PRIVATE src/transport/transport_renoir.c src/transport/frame_out.c)
    target_compile_options(sensor_tap PRIVATE ${COMMON_COMPILE_OPTIONS})
    target_include_directories(sensor_tap PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${RENOIR_INCLUDE_DIR}
    )
    target_link_directories(sensor_tap PRIVATE ${RENOIR_LIB_DIR})
    target_link_libraries(sensor_tap PRIVATE renoir m)
    add_test(NAME importer_golden COMMAND ${CMAKE_SOURCE_DIR}/tests/importer_golden.sh ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR})
    add_test(NAME wind_live COMMAND ${CMAKE_SOURCE_DIR}/tests/wind_live.sh ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR})
    add_test(NAME sensor_profiles_live COMMAND ${CMAKE_SOURCE_DIR}/tests/sensor_profiles_live.sh ${CMAKE_BINARY_DIR})
    target_compile_definitions(send_command PRIVATE ENABLE_IPC)

    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(send_command PRIVATE _FORTIFY_SOURCE=2)
    endif()

    if(ENABLE_FOXGLOVE)
        target_link_libraries(hummingbird PRIVATE Threads::Threads)
        target_compile_definitions(hummingbird PRIVATE ENABLE_FOXGLOVE)
        message(STATUS "Foxglove WebSocket bridge enabled")
    endif()

    set_target_properties(hummingbird PROPERTIES
        BUILD_RPATH "${RENOIR_LIB_DIR};${MUJOCO_DIR}/lib"
        INSTALL_RPATH "${RENOIR_LIB_DIR};${MUJOCO_DIR}/lib"
    )
    set_target_properties(send_command PROPERTIES
        BUILD_RPATH "${RENOIR_LIB_DIR}"
        INSTALL_RPATH "${RENOIR_LIB_DIR}"
    )

    add_executable(urdf_import
        src/urdf_import_main.c
        src/urdf_import_io.c
        src/urdf_import_mesh.c
        src/urdf_import_actuators.c
        src/urdf_import_emit.c
        src/urdf_import_validate.c
        src/urdf_import_xml.c
        src/urdf_import_config.c
        src/urdf_import_sdf.c
    )
    target_compile_options(urdf_import PRIVATE ${COMMON_COMPILE_OPTIONS})
    target_include_directories(urdf_import PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${MUJOCO_DIR}/include
    )
    target_link_directories(urdf_import PRIVATE ${MUJOCO_DIR}/lib)
    target_link_libraries(urdf_import PRIVATE mujoco m)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(urdf_import PRIVATE _FORTIFY_SOURCE=2)
    endif()
    set_target_properties(urdf_import PROPERTIES
        BUILD_RPATH "${MUJOCO_DIR}/lib"
        INSTALL_RPATH "${MUJOCO_DIR}/lib"
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
    set(COMMON_ASAN_COMPILE_OPTIONS
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
        -fno-optimize-sibling-calls
        -fsanitize=float-divide-by-zero
        -fsanitize=float-cast-overflow
    )
    set(COMMON_ASAN_LINK_OPTIONS -fsanitize=address,undefined)
    set(COMMON_ASAN_DEFINITIONS
        ASAN_OPTIONS="detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:detect_leaks=1"
        UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
    )
    foreach(tgt hummingbird send_command sensor_tap urdf_import mdmodels test_conditions test_imu_model test_gnss_model test_misc_models test_ws_proto test_fg_ops test_camera_model test_biomes test_kf_ins test_cli test_import_config)
        if(TARGET ${tgt})
            target_compile_options(${tgt} PRIVATE ${COMMON_ASAN_COMPILE_OPTIONS})
            target_link_options(${tgt} PRIVATE ${COMMON_ASAN_LINK_OPTIONS})
            target_compile_definitions(${tgt} PRIVATE ${COMMON_ASAN_DEFINITIONS})
        endif()
    endforeach()
endif()

if(ENABLE_TSAN)
    foreach(tgt hummingbird send_command sensor_tap urdf_import mdmodels test_conditions test_imu_model test_gnss_model test_misc_models test_ws_proto test_fg_ops test_camera_model test_biomes test_kf_ins test_cli test_import_config)
        if(TARGET ${tgt})
            target_compile_options(${tgt} PRIVATE
                -fsanitize=thread
                -fno-omit-frame-pointer
            )
            target_link_options(${tgt} PRIVATE -fsanitize=thread)
        endif()
    endforeach()
endif()

find_program(CLANG_TIDY clang-tidy)
if(CLANG_TIDY)
    set(TIDY_SRCS "")
    foreach(src ${SOURCES})
        list(APPEND TIDY_SRCS "${CMAKE_SOURCE_DIR}/${src}")
    endforeach()
    if(ENABLE_IPC)
        list(APPEND TIDY_SRCS "${CMAKE_SOURCE_DIR}/src/tools/send_command.c")
    endif()
    if(ENABLE_IPC)
        list(APPEND TIDY_SRCS
            "${CMAKE_SOURCE_DIR}/src/urdf_import_main.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_io.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_mesh.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_actuators.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_emit.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_validate.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_xml.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_config.c"
            "${CMAKE_SOURCE_DIR}/src/urdf_import_sdf.c"
        )
    endif()

    add_custom_target(lint
        COMMAND ${CLANG_TIDY}
            -quiet
            "-header-filter=^${CMAKE_SOURCE_DIR}/(src|include)/"
            --warnings-as-errors=*
            --extra-arg=-Wno-unknown-warning-option
            -p ${CMAKE_BINARY_DIR}
            ${TIDY_SRCS}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running clang-tidy..."
        VERBATIM
    )
endif()
