/*
 * Copyright (c) 2022, smartmx - smartmx@qq.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _PT_PORTS_H_
#define _PT_PORTS_H_

#include "stdio.h"
#include <stdint.h>

typedef unsigned long                   clock_time_t; //����ʱ�ӱ�������

#define PROCESS_CONF_NO_PROCESS_NAMES   1 //�Ƿ񲻴洢process�����֣�һ�������û�б�Ҫ�洢������Ϊ1���ɡ�

#define RINGBUF_INTERRUPT_SAFE_TYPE     uint16_t  //������ϵͳ�жϰ�ȫ��λ����������ringbuf���������

#define PROTOTHREAD_CLOCK_SECONDS       1000  //�����ṩ������ʱ�Ӿ���������1msһ���ж����������ֵΪ 1s / 1ms = 1000

extern void protothread_clock_init(void);

extern clock_time_t protothread_clock_time(void);

/*
    ��ֲ��Ҫ�ڶ�ʱ���ж��е����������
    if(etimer_pending() && etimer_next_expiration_time()<= ptTicks)
    {
      etimer_request_poll();
    }
*/

#endif


