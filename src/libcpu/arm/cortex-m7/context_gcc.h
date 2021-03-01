/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-10-11     Bernard      first version
 * 2012-01-01     aozima       support context switch load/store FPU register.
 * 2013-06-18     aozima       add restore MSP feature.
 * 2013-06-23     aozima       support lazy stack optimized.
 * 2018-07-24     aozima       enhancement hard fault exception handler.
 */

    .cpu cortex-m7
    .syntax unified
    .thumb
    .text

    .equ    SCB_VTOR, 0xE000ED08            /* Vector Table Offset Register */
    .equ    ICSR, 0xE000ED04                /* interrupt control state register */
    .equ    PENDSVSET_BIT, 0x10000000       /* value to trigger PendSV exception */
    
    .equ    SHPR3, 0xE000ED20               /* system priority register (3) */
    .equ    PENDSV_PRI_LOWEST, 0xFFFF0000   /* PendSV and SysTick priority value (lowest) */

/*
 * void rt_hw_context_switch(rt_uint32 from, rt_uint32 to);
 * R0 --> from
 * R1 --> to
 */
    .global rt_hw_context_switch_interrupt
    .type rt_hw_context_switch_interrupt, %function
    .global rt_hw_context_switch
    .type rt_hw_context_switch, %function
rt_hw_context_switch_interrupt:
rt_hw_context_switch:
    /* set rt_thread_switch_interrupt_flag to 1 */
    LDR     R2, =rt_thread_switch_interrupt_flag
    LDR     R3, [R2]
    CMP     R3, #1
    BEQ     _reswitch
    MOV     R3, #1
    STR     R3, [R2]

    LDR     R2, =rt_interrupt_from_thread   /* set rt_interrupt_from_thread */
    STR     R0, [R2]

_reswitch:
    LDR     R2, =rt_interrupt_to_thread     /* set rt_interrupt_to_thread */
    STR     R1, [R2]

    LDR     R0, =ICSR                       /* trigger the PendSV exception (causes context switch) */
    LDR     R1, =PENDSVSET_BIT
    STR     R1, [R0]
    BX      LR

/* R0 --> switch from thread stack
 * R1 --> switch to thread stack
 * psr, pc, LR, R12, R3, R2, R1, R0 are pushed into [from] stack
 */
    .global PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    /* disable interrupt to protect context switch */
    MRS     R2, PRIMASK
    CPSID   I

    /* get rt_thread_switch_interrupt_flag */
    LDR     R0, =rt_thread_switch_interrupt_flag
    LDR     R1, [R0]
    CBZ     R1, pendsv_exit                 /* pendsv already handled */

    /* clear rt_thread_switch_interrupt_flag to 0 */
    MOV     R1, #0
    STR     R1, [R0]

    LDR     R0, =rt_interrupt_from_thread
    LDR     R1, [R0]
    CBZ     R1, switch_to_thread            /* skip register save at the first time */

    MRS     R1, PSP                         /* get from thread stack pointer */
    
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    TST     lr, #0x10                       /* if(!EXC_RETURN[4]) */

    IT      EQ
    VSTMDBEQ r1!, {d8 - d15}                /* push FPU register s16~s31 */
#endif
    
    STMFD   R1!, {R4 - R11}                 /* push R4 - R11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    MOV     r4, #0x00                       /* flag = 0 */

    TST     lr, #0x10                       /* if(!EXC_RETURN[4]) */

    IT      EQ
    MOVEQ   r4, #0x01                       /* flag = 1 */

    STMFD   r1!, {r4}                       /* push flag */
#endif

    LDR     R0, [R0]
    STR     R1, [R0]                        /* update from thread stack pointer */

switch_to_thread:
    LDR     R1, =rt_interrupt_to_thread
    LDR     R1, [R1]
    LDR     R1, [R1]                        /* load thread stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    LDMFD   r1!, {r3}                       /* pop flag */
#endif

    LDMFD   R1!, {R4 - R11}                 /* pop R4 - R11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    CMP     r3,  #0                         /* if(flag_r3 != 0) */

    IT      NE
    VLDMIANE  r1!, {d8 - d15}               /* pop FPU register s16~s31 */
#endif

    MSR     PSP, R1                         /* update stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    ORR     lr, lr, #0x10                   /* lr |=  (1 << 4), clean FPCA. */
    CMP     r3,  #0                         /* if(flag_r3 != 0) */

    IT      NE
    BICNE   lr, lr, #0x10                   /* lr &= ~(1 << 4), set FPCA. */
#endif

pendsv_exit:
    /* restore interrupt */
    MSR     PRIMASK, R2

    ORR     LR, LR, #0x04
    BX      LR

/*
 * void rt_hw_context_switch_to(rt_uint32 to);
 * R0 --> to
 */
    .global rt_hw_context_switch_to
    .type rt_hw_context_switch_to, %function
rt_hw_context_switch_to:
    LDR     R1, =rt_interrupt_to_thread
    STR     R0, [R1]

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    /* CLEAR CONTROL.FPCA */
    MRS     r2, CONTROL                     /* read */
    BIC     r2, #0x04                       /* modify */
    MSR     CONTROL, r2                     /* write-back */
#endif

    /* set from thread to 0 */
    LDR     R1, =rt_interrupt_from_thread
    MOV     R0, #0
    STR     R0, [R1]

    /* set interrupt flag to 1 */
    LDR     R1, =rt_thread_switch_interrupt_flag
    MOV     R0, #1
    STR     R0, [R1]

    /* set the PendSV and SysTick exception priority */
    LDR     R0, =SHPR3
    LDR     R1, =PENDSV_PRI_LOWEST
    LDR.W   R2, [R0,#0]                     /* read */
    ORR     R1, R1, R2                      /* modify */
    STR     R1, [R0]                        /* write-back */

    LDR     R0, =ICSR                       /* trigger the PendSV exception (causes context switch) */
    LDR     R1, =PENDSVSET_BIT
    STR     R1, [R0]

    /* restore MSP */
    LDR     r0, =SCB_VTOR
    LDR     r0, [r0]
    LDR     r0, [r0]
    NOP
    MSR     msp, r0

    /* enable interrupts at processor level */
    CPSIE   F
    CPSIE   I

    /* never reach here! */

/* compatible with old version */
    .global rt_hw_interrupt_thread_switch
    .type rt_hw_interrupt_thread_switch, %function
rt_hw_interrupt_thread_switch:
    BX      LR
    NOP

    .global HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    /* get current context */
    MRS     r0, msp                         /* get fault context from handler. */
    TST     lr, #0x04                       /* if(!EXC_RETURN[2]) */
    BEQ     _get_sp_done
    MRS     r0, psp                         /* get fault context from thread. */
_get_sp_done:
    STMFD   r0!, {r4 - r11}                 /* push r4 - r11 register */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    STMFD   r0!, {lr}                       /* push dummy for flag */
#endif
    STMFD   r0!, {lr}                       /* push exec_return register */

    TST     lr, #0x04                       /* if(!EXC_RETURN[2]) */
    BEQ     _update_msp
    MSR     psp, r0                         /* update stack pointer to PSP. */
    B       _update_done
_update_msp:
    MSR     msp, r0                         /* update stack pointer to MSP. */
_update_done:

    PUSH    {LR}
    BL      rt_hw_hard_fault_exception
    POP     {LR}

    ORR     LR, LR, #0x04
    BX      LR
