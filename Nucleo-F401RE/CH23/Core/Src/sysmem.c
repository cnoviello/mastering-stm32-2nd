/**
 ******************************************************************************
 * @file      sysmem.c
 * @author    Generated by STM32CubeIDE
 * @brief     STM32CubeIDE System Memory calls file
 *
 *            For more information about which C functions
 *            need which of these lowlevel functions
 *            please consult the newlib libc manual
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes */
#include <errno.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

#ifdef MALLOCS_INSIDE_ISRs
    // If we plan to use malloc() in a ISR context - not that good practice
    UBaseType_t usis; //saved interrupt status
    #define DRN_ENTER_CRITICAL_SECTION(_usis) { _usis = taskENTER_CRITICAL_FROM_ISR(); }
    #define DRN_EXIT_CRITICAL_SECTION(_usis)  { taskEXIT_CRITICAL_FROM_ISR(_usis);     }
#else
    #define DRN_ENTER_CRITICAL_SECTION(_usis) vTaskSuspendAll();
    #define DRN_EXIT_CRITICAL_SECTION(_usis)  xTaskResumeAll();
#endif

/**
 * Pointer to the current high watermark of the heap usage
 */
void * _sbrk_r(struct _reent *pReent, int incr) {
  extern uint8_t _end; /* Symbol defined in the linker script */
  extern uint8_t _estack; /* Symbol defined in the linker script */
  extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap = (uint8_t *)stack_limit;
  static uint8_t *__sbrk_heap_end = NULL;

  uint8_t *prev_heap_end;

  /* Initialize heap end at first call */
  if (NULL == __sbrk_heap_end) {
    __sbrk_heap_end = &_end;
  }

  /* Protect heap from growing into the reserved MSP stack */
  if (__sbrk_heap_end + incr > max_heap) {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}

char * sbrk(int incr) {
  return _sbrk_r(_impure_ptr, incr);
}

#ifdef MALLOCS_INSIDE_ISRs // block interrupts during free-storage use
  static UBaseType_t malLock_uxSavedInterruptStatus;
#endif
void __malloc_lock(struct _reent *r)   {
  (void)(r);
  #if defined(MALLOCS_INSIDE_ISRs)
    DRN_ENTER_CRITICAL_SECTION(malLock_uxSavedInterruptStatus);
  #else
    BaseType_t insideAnISR = xPortIsInsideInterrupt();
    configASSERT( !insideAnISR ); // Make sure no malloc() inside ISRs
    vTaskSuspendAll();
  #endif
};

void __malloc_unlock(struct _reent *r) {
  (void)(r);
  #if defined(MALLOCS_INSIDE_ISRs)
    DRN_EXIT_CRITICAL_SECTION(malLock_uxSavedInterruptStatus);
  #else
    (void)xTaskResumeAll();
  #endif
};

