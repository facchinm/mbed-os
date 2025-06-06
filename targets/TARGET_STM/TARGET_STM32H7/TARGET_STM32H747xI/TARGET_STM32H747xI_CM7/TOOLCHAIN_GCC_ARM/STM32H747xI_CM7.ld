/* Linker script to configure memory regions. */
/*
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016-2020 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
*/

#include "../cmsis_nvic.h"


#if !defined(MBED_APP_START)
  #define MBED_APP_START  MBED_ROM_START
#endif

#if !defined(MBED_APP_SIZE)
  #define MBED_APP_SIZE  MBED_ROM_SIZE
#endif

#if !defined(MBED_CONF_TARGET_BOOT_STACK_SIZE)
    /* This value is normally defined by the tools
       to 0x1000 for bare metal and 0x400 for RTOS */
    #define MBED_CONF_TARGET_BOOT_STACK_SIZE  0x400
#endif

/* Round up VECTORS_SIZE to 8 bytes */
#define VECTORS_SIZE  (((NVIC_NUM_VECTORS * 4) + 7) & 0xFFFFFFF8)

MEMORY
{
  FLASH (rx)     : ORIGIN = MBED_APP_START, LENGTH = MBED_APP_SIZE
  DTCMRAM (rwx)  : ORIGIN = NVIC_RAM_VECTOR_ADDRESS + VECTORS_SIZE, LENGTH = 128K - VECTORS_SIZE
  RAM (xrw)      : ORIGIN = MBED_RAM_START, LENGTH = MBED_RAM_SIZE
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}


#if defined(MCUBOOT_BOOTLOADER_BUILD) && ( defined(TARGET_PORTENTA_H7_M7) || defined(TARGET_NICLA_VISION) || defined(TARGET_OPTA) || defined(TARGET_GIGA))
_ConfigStart = 0x0802F000;
_IdentificationStart = 0x080002F0;
_EncryptKeyStart = 0x08000300;
_SigningKeyStart = 0x08000400;
#endif


/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   _estack
 */
ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
#if defined(MCUBOOT_BOOTLOADER_BUILD) && ( defined(TARGET_PORTENTA_H7_M7) || defined(TARGET_NICLA_VISION) || defined(TARGET_OPTA) || defined(TARGET_GIGA))
        . = ABSOLUTE(0x080002F0);
        KEEP(*(.bootloader_identification))
        KEEP(*(.signing_key))
        KEEP(*(.encrypt_key))
#endif
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

#if !(defined(MCUBOOT_BOOTLOADER_BUILD)  && defined(TARGET_PORTENTA_H7_M7))
        *(.rodata*)
        KEEP(*(.eh_frame*))
#endif

        *ltrans0*.o(.rodata*)
        *ltrans1*.o(.rodata*)
        *ltrans2*.o(.rodata*)
        *ltrans3*.o(.rodata*)
        *ltrans4*.o(.rodata*)

#if (defined(MCUBOOT_BOOTLOADER_BUILD)  && defined(TARGET_PORTENTA_H7_M7))
        . = ABSOLUTE(0x0801F000);
        KEEP(*(.bootloader_version))
#endif

        *lib*.o(.rodata*)

#if (defined(MCUBOOT_BOOTLOADER_BUILD) || defined(BOOTLOADER_BUILD))  && ( defined(TARGET_NICLA_VISION) || defined(TARGET_OPTA) || defined(TARGET_GIGA))
        . = ABSOLUTE(0x0801F000);
        KEEP(*(.bootloader_version))
#endif

#if (defined(MCUBOOT_BOOTLOADER_BUILD)  && defined(TARGET_PORTENTA_H7_M7))
        *(.rodata*)
        KEEP(*(.eh_frame*))
#endif

    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    __etext = .;
    _sidata = .;
    
    .data : AT (__etext)
    {
        __data_start__ = .;
        _sdata = .;
        *(vtable)
        *(.data*)

        . = ALIGN(8);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(8);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(8);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(8);
        /* All data end */
        __data_end__ = .;
        _edata = .;

    } > RAM

    /* Uninitialized data section
     * This region is not initialized by the C/C++ library and can be used to
     * store state across soft reboots. */
    .uninitialized (NOLOAD):
    {
        . = ALIGN(32);
        __uninitialized_start = .;
        *(.uninitialized)
        KEEP(*(.keep.uninitialized))
        . = ALIGN(32);
        __uninitialized_end = .;
    } > RAM

    .bss :
    {
        . = ALIGN(8);
        __bss_start__ = .;
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end__ = .;
        _ebss = .;
    } > RAM

    .heap (COPY):
    {
        __end__ = .;
        PROVIDE(end = .);
        *(.heap*)
        . = ORIGIN(RAM) + LENGTH(RAM) - MBED_CONF_TARGET_BOOT_STACK_SIZE;
        __HeapLimit = .;
    } > RAM

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > RAM

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    _estack = __StackTop;
    __StackLimit = __StackTop - MBED_CONF_TARGET_BOOT_STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    .lwip_sec (NOLOAD) : {
        . = ABSOLUTE(0x30040000);
        *(.RxDecripSection)

        . = ABSOLUTE(0x30040100);
        *(.TxDecripSection)

        . = ABSOLUTE(0x30040400);
        *(.RxArraySection)

        . = ABSOLUTE(0x30044000);
        *(.ethusbram)

    } >RAM_D2 AT> FLASH
}
