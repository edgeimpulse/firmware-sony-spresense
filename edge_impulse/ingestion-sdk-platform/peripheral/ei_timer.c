/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Includes ---------------------------------------------------------------- */
#include "ei_timer.h"
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include <arch/chip/timer.h>

/* Private variables ------------------------------------------------------- */
#define TIMER_DEVICE        "/dev/timer0"

static struct timer_int_s {
    int fd;
    //unsigned int (*isr)(void);
//} s_timer_int = { -1, (void*)0 };
} s_timer_int = { -1};

static struct sigaction act;

#define EI_TIMER_SIGNO      (17u)

static int get_timer_status(int fd, struct timer_status_s* status);
static bool util_timer_is_running(int fd);
static int util_stop_timer(int fd);
static bool timer_handler(uint32_t *next_interval_us, void *arg);
/* Public functions ------------------------------------------------------- */

unsigned int (*local_isr)(void);

static void timer_sighandler(int signo, FAR siginfo_t *siginfo,
                             FAR void *context)
{
  /* Does nothing in this example except for increment a count of signals
   * received.
   *
   * NOTE: The use of signal handler is not recommended if you are concerned
   * about the signal latency.  Instead, a dedicated, high-priority thread
   * that waits on sigwaitinfo() is recommended.  High priority is required
   * if you want a deterministic wake-up time when the signal occurs.
   */
  
    if (local_isr != NULL) {
        local_isr();
    }
    
}

/**
 * @brief 
 * 
 * @param isr 
 * @param interval 
 * @return int 
 */
int ei_timer_init(unsigned int (*isr)(void), uint32_t interval)
{
    int ret;
    //struct timer_sethandler_s sethandler;
    struct timer_notify_s notify;

    local_isr = isr;

    s_timer_int.fd = open("/dev/timer0", O_RDONLY);

    if (s_timer_int.fd < 0)
    {
        //printf("ERROR: Failed to open /dev/timer0: %d\n", errno);
        return 1;
    }

    ret = ioctl(s_timer_int.fd, TCIOC_SETTIMEOUT, interval);
    if (ret < 0)
    {
        //printf("ERROR: Failed to set the timer interval: %d\n", errno);
        close(s_timer_int.fd);
        return 2;
    }

#if 0
    s_timer_int.isr = isr;

    sethandler.handler = timer_handler;
    sethandler.arg     = NULL;

    ret = ioctl(s_timer_int.fd, TCIOC_SETHANDLER, (unsigned long)&sethandler);
    if (ret < 0) {
        printf("ERROR: Failed to set timer handler (errno = %d)\n", errno);
        close(s_timer_int.fd);
        return 3;
    }
#else

    act.sa_sigaction = timer_sighandler;
    act.sa_flags     = SA_SIGINFO;

    sigfillset(&act.sa_mask);
    sigdelset(&act.sa_mask, EI_TIMER_SIGNO);

    ret = sigaction(EI_TIMER_SIGNO, &act, NULL);
    if (ret != OK) {      
        close(s_timer_int.fd);
        return 1;
    }

    notify.pid      = getpid();
    //notify.periodic = true;   // not implemented

    notify.event.sigev_notify = SIGEV_SIGNAL;
    notify.event.sigev_signo  = EI_TIMER_SIGNO;
    notify.event.sigev_value.sival_ptr = NULL;

    ret = ioctl(s_timer_int.fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
    if (ret < 0)
    {
        printf("ERROR: Failed to set the timer handler: %d\n", errno);
        close(s_timer_int.fd);
        return 2;
    }

#endif

    ret = ioctl(s_timer_int.fd, TCIOC_START, 0);
    if (ret < 0) {
        printf("ERROR: Failed to start timer (errno = %d)\n", errno);
        close(s_timer_int.fd);
        return 4;
    }

    return 0;
}

/**
 * @brief 
 * 
 */
void ei_timer_stop(void)
{
    util_stop_timer(s_timer_int.fd);

    close(s_timer_int.fd);
}
/* Private functions ------------------------------------------------------- */
static int get_timer_status(int fd, struct timer_status_s* status)
{
    if (ioctl(fd, TCIOC_GETSTATUS, (unsigned long)((uintptr_t)status)) < 0) {
        return ERROR;
    }

    return OK;
}

static bool util_timer_is_running(int fd)
{
    struct timer_status_s status;

    if (get_timer_status(fd, &status)) {
        return false;
    }
    
    return ((status.flags & TCFLAGS_ACTIVE) == TCFLAGS_ACTIVE);
}

/**
 * @brief 
 * 
 * @param fd 
 * @return int 
 */
static int util_stop_timer(int fd)
{
    if (util_timer_is_running(fd)) {
        int ret = ioctl(fd, TCIOC_STOP, 0);
        if (ret < 0) {            
            return ERROR;
        }
    }

    act.sa_handler = SIG_DFL;
    sigaction(EI_TIMER_SIGNO, &act, NULL);
    local_isr = NULL;

    return OK;
}

#if 0
/**
 * @brief 
 * 
 * @param next_interval_us 
 * @param arg 
 * @return true 
 * @return false 
 */
static bool timer_handler(uint32_t *next_interval_us, void *arg)
{
    (void*)arg;

    unsigned int next;

    if (!s_timer_int.isr) {
        return false;
    }

    next = s_timer_int.isr();
    printf("timer_handler next: %ld\n", next);

    if (next) {
        *next_interval_us = next;
        printf("timer_handler return true :D:D:D:D:D\n");
        return true;
    } else {
        printf("timer_handler return false :/");
        return false;
    }
}
#endif