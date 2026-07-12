# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 GIMBAL_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "GIMBAL_BOARD")
#set(BOARD "CHASSIS_BOARD")


# =========================================================================
# 2. 声明Hero目标文件库（OBJECT库，仅编译目标文件不链接，保留原工程逻辑）
# =========================================================================
add_library(Hero OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Hero PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件（严格匹配文件名/大小写/路径） =====================
if (BOARD STREQUAL "GIMBAL_BOARD")
    target_sources(Hero
            PRIVATE
            # === Application ===
            Application/pyro_wl_chassis_app.cpp
            Application/pyro_wl_chassis_cfg.cpp

        #     # === Gimbal 模块 ===
        #     Gimbal/pyro_screw_gimbal.cpp
        #     Gimbal/fsm/pyro_screw_active_state.cpp
        #     Gimbal/fsm/pyro_screw_sling_state.cpp
        #     Gimbal/fsm/pyro_screw_autoaim_state.cpp
        #     Gimbal/fsm/pyro_screw_normal_state.cpp
        #     Gimbal/fsm/pyro_screw_calibration_state.cpp
        #     Gimbal/fsm/pyro_screw_idle_state.cpp
        #     Gimbal/fsm/pyro_screw_passive_state.cpp

        #     # === Booster 模块 ===
        #     Booster/pyro_quad_booster.cpp
        #     Booster/fsm/pyro_quad_active_state.cpp
        #     Booster/fsm/pyro_quad_anti_jam_state.cpp
        #     Booster/fsm/pyro_quad_busy_state.cpp
        #     Booster/fsm/pyro_quad_homing_state.cpp
        #     Booster/fsm/pyro_quad_interim_state.cpp
        #     Booster/fsm/pyro_quad_passive_state.cpp
        #     Booster/fsm/pyro_quad_ready_state.cpp
        #     Booster/fsm/pyro_quad_stall_state.cpp
        #     Booster/fsm/pyro_quad_temp_state.cpp

#             # === Communication ===
#             Communication/Gimbal_board/pyro_autoaim_drv.cpp
#             Communication/Gimbal_board/pyro_autoaim_com.cpp
# #            Communication/Gimbal_board/pyro_custom_drv.cpp
# #            Communication/Gimbal_board/pyro_custom_com.cpp
#             Communication/Gimbal_board/pyro_board_com.cpp
#             Communication/general/pyro_board_drv.cpp


        #     pyro_mission_planer.cpp
        #     pyro_init_thread.cpp
    )

    # 针对 Gimbal 的特殊头文件包含
    target_include_directories(Hero
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Booster
            ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
            ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Gimbal_board
            ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
            ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
            VT03_UART=PYRO_UART1
            AUTOAIM_UART=PYRO_UART7
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Hero
            PRIVATE
            # === Application ===
            Application/Chassis_board/pyro_chassis_app.cpp

            # === Chassis 模块 ===
            Chassis/pyro_hybrid_chassis.cpp
            Chassis/fsm/pyro_hybrid_active_state.cpp
            Chassis/fsm/pyro_hybrid_climbing_state.cpp
            Chassis/fsm/pyro_hybrid_cruising_state.cpp
            Chassis/fsm/pyro_hybrid_leg_retraction_state.cpp
            Chassis/fsm/pyro_hybrid_passive_state.cpp
            Chassis/fsm/pyro_hybrid_track_climbing_state.cpp

            # === Communication ===
            Communication/Chassis_board/pyro_board_com.cpp
            Communication/General/pyro_board_drv.cpp
            Communication/Chassis_board/pyro_ui_com.cpp
            Communication/Chassis_board/pyro_board_com.cpp


            pyro_mission_planer.cpp
            pyro_init_thread.cpp
    )

    # 针对 Chassis 的特殊头文件包含
    target_include_directories(Hero
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
            ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Chassis_board
            ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_compile_definitions(PYRo PUBLIC
            REFEREE_UART=PYRO_UART1
            SR05_UART=PYRO_UART5
            SR04_UART=PYRO_UART10
            SUPERCAP_UART=PYRO_UART7
    )
endif ()

# ===================== 头文件包含路径（覆盖所有模块根目录） =====================
target_include_directories(Hero
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Booster
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# ===================== 链接依赖库（保留原工程依赖，可按需增删） =====================
target_link_libraries(Hero
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
