# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(${CMAKE_CROSSCOMPILING})
    include(mbed_set_linker_script)
    
    add_subdirectory(TARGET_Ambiq_Micro EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Analog_Devices EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_ARM_FM EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_ARM_SSG EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Cypress EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Freescale EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_GigaDevice EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Maxim EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_NORDIC EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_NUVOTON EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_NXP EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_RASPBERRYPI EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_RENESAS EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Samsung EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_Silicon_Labs EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_STM EXCLUDE_FROM_ALL)
    add_subdirectory(TARGET_TOSHIBA EXCLUDE_FROM_ALL)
endif()
