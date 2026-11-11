# 中转层(relay)CMake —— 复刻 motion/io ISTAR RPC 服务端,转发到 28 模型
# 只读引用 1927project 的接口库,不修改其任何文件。
cmake_minimum_required(VERSION 3.16)
project(relay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ============ 外部路径(1927project 根 = 本目录上三级:1927project\simulator\simbridge\relay) ============
# 允许外层用 -DPJ_ROOT=<1927project 根> 覆盖(rebuild_bin.bat 不传,靠相对路径)。
set(PJ_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH "1927project root")
set(MOTION    "${PJ_ROOT}/module/motion")
set(IO        "${PJ_ROOT}/module/io")
set(PLAT_INC  "${IO}/basiclibrary_output/include")
set(PLAT_LIB  "${IO}/basiclibrary_output/lib")
# 下游集成包位置(构建重构后):module/<m>/output/interface/rpc/{include,lib/release}
#   手写头在 interface/include,生成头(motion_server.h 等)+ dll/lib 由 install 规则
#   落到 output/interface/rpc/;旧的 interface/lib 已不再回写。
set(MOTION_INC "${MOTION}/output/interface/rpc/include")
set(MOTION_LIB "${MOTION}/output/interface/rpc/lib/release")
set(IO_INC    "${IO}/output/interface/rpc/include")
set(IO_LIB    "${IO}/output/interface/rpc/lib/release")

set(PLATFORM_INCLUDE_DIRS
    "${PLAT_INC}/common/include"
    "${PLAT_INC}/istarlog/include"
    "${PLAT_INC}/istarmw/include"
    "${PLAT_INC}/istarserializer/include"
    "${PLAT_INC}/istaryaml/include"
    "${PLAT_INC}/istarzmqworker/include"
    "${PLAT_INC}/istargf/include"
    "${PLAT_INC}/istarthreadpool/include"
    "${PLAT_INC}/istarcsv/include"
    "${PLAT_INC}/istarencryption/include"
    "${PLAT_INC}/istarfilesystem/include"
    "${PLAT_INC}/istarsm/include"
)

set(PLATFORM_LINK_LIBS
    ISTARLog ISTARYaml ISTARMW ISTARGF ISTARThreadPool ISTARCsv
    ISTAREncryption ISTARFileSystem ISTARSerializer ISTARSM ISTARZmqWorker
)

set(RELAY_INC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ============ 通用配置 ============
function(relay_configure TGT)
    target_include_directories(${TGT} PRIVATE ${RELAY_INC} ${PLATFORM_INCLUDE_DIRS})
    target_link_directories(${TGT} PRIVATE ${PLAT_LIB})
    if(MSVC)
        target_compile_options(${TGT} PRIVATE /utf-8 /bigobj /W3)
        target_compile_definitions(${TGT} PRIVATE _HAS_STD_BYTE=0 WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS)
    endif()
endfunction()

# ============ relay_motion(1 exe, 按 config1~4 起 4 个进程) ============
add_executable(relay_motion
    src/relay_motion_main.cpp
    src/motion_relay_server.cpp
    src/relay_cfg.cpp
    src/relay_sim.cpp
    src/relay_util.cpp
)
relay_configure(relay_motion)
target_include_directories(relay_motion PRIVATE ${MOTION_INC})
target_link_directories(relay_motion PRIVATE ${MOTION_LIB})
target_link_libraries(relay_motion PRIVATE motion ${PLATFORM_LINK_LIBS})

# ============ relay_io ============
add_executable(relay_io
    src/relay_io_main.cpp
    src/io_relay_server.cpp
    src/relay_cfg.cpp
    src/relay_sim.cpp
    src/relay_util.cpp
)
relay_configure(relay_io)
target_include_directories(relay_io PRIVATE ${IO_INC})
target_link_directories(relay_io PRIVATE ${IO_LIB})
target_link_libraries(relay_io PRIVATE io ${PLATFORM_LINK_LIBS})

# ============ probe(仅验证用;复用平台客户端库,同 mctrl 视角) ============
set(MCTRL_RPC  "${PJ_ROOT}/mctrl/MainProject/RPCModule")
set(MCTRL_LIB  "${MCTRL_RPC}/lib")

add_executable(relay_probe_motion src/relay_probe.cpp)
relay_configure(relay_probe_motion)
target_compile_definitions(relay_probe_motion PRIVATE PROBE_MOTION)
target_include_directories(relay_probe_motion PRIVATE
    "${MCTRL_RPC}/include/motion"
    "${MOTION_INC}")        # 生成的 motion_client.h
target_link_directories(relay_probe_motion PRIVATE ${MCTRL_LIB} ${MOTION_LIB})
target_link_libraries(relay_probe_motion PRIVATE motion ${PLATFORM_LINK_LIBS})

add_executable(relay_probe_io src/relay_probe.cpp)
relay_configure(relay_probe_io)
target_include_directories(relay_probe_io PRIVATE
    "${MCTRL_RPC}/include/io"
    "${IO_INC}")            # 生成的 io_client.h
target_link_directories(relay_probe_io PRIVATE ${MCTRL_LIB} ${IO_LIB})
target_link_libraries(relay_probe_io PRIVATE io ${PLATFORM_LINK_LIBS})

# ============ relay_bridge(MQTT 桥:视觉结果 + 共焦;无 ISTAR 依赖) ============
add_executable(relay_bridge
    src/relay_bridge_main.cpp
    src/relay_mqtt.cpp
    src/relay_sim.cpp
    src/relay_util.cpp
)
relay_configure(relay_bridge)

# ============ relay_m5_probe(扮演 GUI,走 GUI→mctrl→motion→28 全链路) ============
set(GUI_RPC "${PJ_ROOT}/gui/MainProject/RPCModule/include")
set(GUI_MCTRL_LIB "${PJ_ROOT}/gui/build/Release/MainProject")

add_executable(relay_m5_probe src/relay_m5_probe.cpp)
relay_configure(relay_m5_probe)
target_include_directories(relay_m5_probe PRIVATE ${GUI_RPC})
target_link_directories(relay_m5_probe PRIVATE ${GUI_MCTRL_LIB})
target_link_libraries(relay_m5_probe PRIVATE mctrl ${PLATFORM_LINK_LIBS})
