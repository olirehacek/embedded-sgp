/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SGPC3_H
#define SGPC3_H
#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

s16 sgpc3_probe(void);
s16 sgpc3_iaq_init(void);
s16 sgpc3_iaq_init0(void);
s16 sgpc3_iaq_init16(void);
s16 sgpc3_iaq_init64(void);
s16 sgpc3_iaq_init184(void);
s16 sgpc3_iaq_init_continuous(void);

const char *sgpc3_get_driver_version(void);
u8 sgpc3_get_configured_address(void);
s16 sgpc3_get_feature_set_version(u16 *feature_set_version, u8 *product_type);
s16 sgpc3_get_serial_id(u64 *serial_id);

s16 sgpc3_get_iaq_baseline(u16 *baseline);
s16 sgpc3_set_iaq_baseline(u16 baseline);
s16 sgpc3_get_tvoc_inceptive_baseline(u16 *tvoc_inceptive_baseline);

s16 sgpc3_measure_iaq_blocking_read(u16 *tvoc_ppb);
s16 sgpc3_measure_iaq(void);
s16 sgpc3_read_iaq(u16 *tvoc_ppb);

s16 sgpc3_measure_tvoc_blocking_read(u16 *tvoc_ppb);
s16 sgpc3_measure_tvoc(void);
s16 sgpc3_read_tvoc(u16 *tvoc_ppb);

s16 sgpc3_measure_signals_blocking_read(u16 *ethanol_signal);
s16 sgpc3_measure_signals(void);
s16 sgpc3_read_signals(u16 *ethanol_signal);

s16 sgpc3_measure_raw_blocking_read(u16 *tvoc_ppb, u16 *ethanol_signal);
s16 sgpc3_measure_raw(void);
s16 sgpc3_read_raw(u16 *tvoc_ppb, u16 *ethanol_signal);

s16 sgpc3_set_power_mode(u16 power_mode);
s16 sgpc3_set_absolute_humidity(u32 absolute_humidity);
s16 sgpc3_measure_test(u16 *test_result);

#ifdef __cplusplus
}
#endif

#endif /* SGPC3_H */

