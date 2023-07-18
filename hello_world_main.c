/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

// #include "dspi_dotprod.h"
// #include "dsp_esp32_s3_ext.h"
// #include "dsp_types.h"

#define ESP_ERR_DSP_PARAM_OUTOFRANGE 0

typedef struct image2d_s
{
    void* data; // could be int8_t, unt8_t, int16_t, unt16_t, float
    int step_x; // step of elements by X
    int step_y; // step of elements by Y, usually is 1
    int stride_x; // stride width: size of the elements in X axis * by step_x + padding
    int stride_y; // stride height: size of the elements in Y axis * by step_y + padding
    // Point[x,y] = data[width*y*step_y + x*step_x];
    // Full data size = width*height

} image2d_t;

esp_err_t dspi_dotprod_s16_aes3(image2d_t* in_image, image2d_t* filter, int16_t *out_value, int count_x, int count_y, int shift)
{
    // QR q0, q1, q2, q3, q4, q5;
    int32_t temp_data[4];

    if (in_image->step_x*count_x > in_image->stride_x) return ESP_ERR_DSP_PARAM_OUTOFRANGE;
    if (in_image->step_y*count_y > in_image->stride_y) return ESP_ERR_DSP_PARAM_OUTOFRANGE;
    if (filter->step_x*count_x > filter->stride_x) return ESP_ERR_DSP_PARAM_OUTOFRANGE;
    if (filter->step_y*count_y > filter->stride_y) return ESP_ERR_DSP_PARAM_OUTOFRANGE;

    // Check if filter data not aligned
    if (
        ((((uint32_t)filter->data)&0xf) != 0)
        || (filter->step_x*count_x != filter->stride_x)
        || (filter->step_x != 1)
        || (filter->step_y != 1)
    )
    {
        // return dspi_dotprod_s16_ansi(in_image, filter, out_value, count_x, count_y, shift);
        printf("Filter data not aligned");
        return ESP_OK;
    }
    if (
        (in_image->step_x != 1)
        || (in_image->step_y != 1)
        || ((count_x&0x7) != 0) // the count X must be divided by 8
        || (count_y < 4) // the count Y must be divided by 4
    )
    {
        printf("second check");
        return ESP_OK;
        // return dspi_dotprod_s16_ansi(in_image, filter, out_value, count_x, count_y, shift);
    }
    if ((count_x > 32) && ((count_x & 31) != 0)) // for counts 40, 48, 56...
    {
        printf("third check");
        return ESP_OK;
        // return dspi_dotprod_s16_ansi(in_image, filter, out_value, count_x, count_y, shift);
    }

    int16_t* i_data =  (int16_t*)in_image->data;
    int16_t* f_data =  (int16_t*)filter->data;
    uint32_t i_data_ptr =  (uint32_t)in_image->data;
    uint32_t f_data_ptr =  (uint32_t)filter->data;

    uint32_t i_data_ptr_n = i_data_ptr + 16;

    int i_step = in_image->stride_x*in_image->step_y*sizeof(int16_t); // step in bytes
    int f_step = filter->stride_x*filter->step_y;
    int rest_i_update = in_image->stride_x - count_x;
    rest_i_update = rest_i_update*sizeof(int16_t);

    uint32_t update_x_0 = 16;
    uint32_t update_x_1 = 16;
    uint32_t update_x_2 = 16;

    uint32_t total_x = 0;
    uint32_t total_y = count_y;
    asm(".ttt1: nop;");
    // WUR_SAR_BYTE(0);
    __builtin_xtensa_wur_sar_byte(0);
    // WUR_ACCX_0(0);
    __builtin_xtensa_wur_accx_0(0);
    // WUR_ACCX_1(0);
    __builtin_xtensa_wur_accx_1(0);

    // Load f[0]
    // EE_VLD_128_IP(q0, f_data_ptr, 16);

    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);

    // Load i_data[0]
    if (count_x == 24)
    {
        update_x_0 = i_step - 32;
        update_x_1 = -16;
        update_x_2 = 32;

        asm("   dotprod_s16_24: nop");
        // EE_LD_128_USAR_IP(q4, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(4, i_data_ptr, 16);
        // EE_LD_128_USAR_IP(q2, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(2, i_data_ptr, 16);
        // EE_SRC_Q_LD_IP(q3, i_data_ptr, 16, q4, q2); // q4 = shifted
        __builtin_xtensa_ee_src_q_ld_ip(3, i_data_ptr, 16, 4, 2); // q4 = shifted

        for (size_t y = 0; y < total_y; y++)
        {
            {
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q4, i_data_ptr, update_x_0, q0, q4, q2, q3);
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(4, i_data_ptr, update_x_0, 0, 4, 2, 3);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);

                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q2, i_data_ptr, update_x_1, q0, q2, q3, q4);
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(2, i_data_ptr, update_x_1, 0, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q4, i_data_ptr, update_x_2);
                    __builtin_xtensa_ee_ld_128_usar_xp(4, i_data_ptr, update_x_2);

                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q3, i_data_ptr, 16, q0, q3, q4, q2);
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 0, 3, 4, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            }
        }
    }

    if (count_x == 16)
    {
        total_y = total_y>>1;
        update_x_0 = i_step - 16;
        update_x_1 = -16;
        update_x_2 = 32;

        // EE_LD_128_USAR_IP(q5, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(5, i_data_ptr, 16);
        // EE_LD_128_USAR_IP(q2, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(2, i_data_ptr, 16);
        asm("   dotprod_s16_16: nop");
        // EE_SRC_Q_LD_XP(q3, i_data_ptr, update_x_0, q5, q2);
        __builtin_xtensa_ee_src_q_ld_xp(3, i_data_ptr, update_x_0, 5, 2);
        for (size_t y = 0; y < total_y; y++)
        {
            {
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q4, i_data_ptr, update_x_1, q0, q5, q2, q3);// acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(4, i_data_ptr, update_x_1, 0, 5, 2, 3);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q3, i_data_ptr, update_x_2);
                    __builtin_xtensa_ee_ld_128_usar_xp(3, i_data_ptr, update_x_2);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q5, i_data_ptr, update_x_0, q1, q2, q3, q4);// acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(5, i_data_ptr, update_x_0, 1, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q2, i_data_ptr, update_x_1, q0, q3, q4, q5);// acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(2, i_data_ptr, update_x_1, 0, 3, 4, 5);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_2);
                    __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_2);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q3, i_data_ptr, update_x_0, q1, q4, q5, q2);// acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(3, i_data_ptr, update_x_0, 1, 4, 5, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            }
        }
    }
    if (count_x == 8)
    {
        total_y = total_y>>2;
        rest_i_update =  i_step;
        update_x_0 = -16;
        update_x_1 = i_step + 16;
        i_data_ptr += 16;

        // EE_LD_128_USAR_XP(q2, i_data_ptr, update_x_0);
        __builtin_xtensa_ee_ld_128_usar_xp(2, i_data_ptr, update_x_0);
        // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_1);
        __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_1);
        asm("   dotprod_s16_8: nop");
        // EE_SRC_Q_LD_XP(q3, i_data_ptr, update_x_0, q5, q2);
        __builtin_xtensa_ee_src_q_ld_xp(3, i_data_ptr, update_x_0, 5, 2);
        // EE_LD_128_USAR_XP(q2, i_data_ptr, update_x_1);
        __builtin_xtensa_ee_ld_128_usar_xp(2, i_data_ptr, update_x_1);
        for (size_t y = 0; y < total_y; y++)
        {
            {
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q4, i_data_ptr, update_x_0, q0, q5, q2, q3);// acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(4, i_data_ptr, update_x_0, 0, 5, 2, 3);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q3, i_data_ptr, update_x_1);
                    __builtin_xtensa_ee_ld_128_usar_xp(3, i_data_ptr, update_x_1);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q5, i_data_ptr, update_x_0, q1, q2, q3, q4);// acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(5, i_data_ptr, update_x_0, 1, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q4, i_data_ptr, update_x_1);
                    __builtin_xtensa_ee_ld_128_usar_xp(4, i_data_ptr, update_x_1);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q2, i_data_ptr, update_x_0, q0, q3, q4, q5);// acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(2, i_data_ptr, update_x_0, 0, 3, 4, 5);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_1);
                    __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_1);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q3, i_data_ptr, update_x_0, q1, q4, q5, q2);// acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_xp_qup(3, i_data_ptr, update_x_0, 1, 4, 5, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_LD_128_USAR_XP(q2, i_data_ptr, update_x_1);
                    __builtin_xtensa_ee_ld_128_usar_xp(2, i_data_ptr, update_x_1);
            }
        }
    }
    if (count_x == 32)
    {
        update_x_0 = i_step - (count_x - 8)*2;
        update_x_1 = -16;
        update_x_2 = 32;

        // EE_LD_128_USAR_IP(q5, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(5, i_data_ptr, 16);
        // EE_LD_128_USAR_IP(q2, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(2, i_data_ptr, 16);
        asm("   dotprod_s16_32: nop");
        // EE_SRC_Q_LD_IP(q3, i_data_ptr, 16, q5, q2);
        __builtin_xtensa_ee_src_q_ld_ip(3, i_data_ptr, 16, 5, 2);
        for (size_t y = 0; y < total_y; y++)
        {
            {
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q4, i_data_ptr, 16, q0, q5, q2, q3);// acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(4, i_data_ptr, 16, 0, 5, 2, 3);
                    // EE_VLD_128_IP(q1, f_data_ptr, 160;
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q5, i_data_ptr, update_x_0, q1, q2, q3, q4);// acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(5, i_data_ptr, 16, 1, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_XP_QUP (q2, i_data_ptr, update_x_1, q0, q3, q4, q5);// acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(2, i_data_ptr, 16, 0, 3, 4, 5);
                    // Load q5 again
                    // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_2);
                    __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_2);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q3, i_data_ptr, 16, q1, q4, q5, q2);// acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 1, 4, 5, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            }
        }
    }
    if (count_x == 64)
    {
        update_x_0 = i_step - (count_x - 8)*2;
        update_x_1 = -16;
        update_x_2 = 32;
        total_x = count_x/32 - 1;

        // EE_LD_128_USAR_IP(q5, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(5, i_data_ptr, 16);
        // EE_LD_128_USAR_IP(q2, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(2, i_data_ptr, 16);
        asm("   dotprod_s16_64: nop");
        // EE_SRC_Q_LD_IP(q3, i_data_ptr, 16, q5, q2);
        __builtin_xtensa_ee_src_q_ld_ip(3, i_data_ptr, 16, 5, 2);
        for (size_t y = 0; y < total_y; y++)
        {
            {
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q4, i_data_ptr, 16, q0, q5, q2, q3);// acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(4, i_data_ptr, 16, 0, 5, 2, 3);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q5, i_data_ptr, 16, q1, q2, q3, q4);// acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(5, i_data_ptr, 16, 1, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q2, i_data_ptr, 16, q0, q3, q4, q5);// acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(2, i_data_ptr, 16, 0, 3, 4, 5);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q3, i_data_ptr, 16, q1, q4, q5, q2);// acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 1, 4, 5, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            }
            // EE_VMULAS_S16_ACCX_LD_IP_QUP(q4, i_data_ptr, 16, q0, q5, q2, q3); // acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(4, i_data_ptr, 16, 0, 5, 2, 3);
            // EE_VLD_128_IP(q1, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_XP_QUP(q5, i_data_ptr, update_x_0, q1, q2, q3, q4); // acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(5, i_data_ptr, 16, 1, 2, 3, 4);
            // EE_VLD_128_IP(q0, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_XP_QUP(q2, i_data_ptr, update_x_1, q0, q3, q4, q5); // acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(2, i_data_ptr, 16, 0, 3, 4, 5);
            // Load q5 again
            // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_2);
            __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_2);
            // EE_VLD_128_IP(q1, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_IP_QUP(q3, i_data_ptr, 16, q1, q4, q5, q2); // acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 1, 4, 5, 2);
            // EE_VLD_128_IP(q0, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
        }
    }

    if (count_x > 64)
    {
        update_x_0 = i_step - (count_x - 8)*2;
        update_x_1 = -16;
        update_x_2 = 32;
        total_x = count_x/32 - 1;

        // EE_LD_128_USAR_IP(q5, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(5, i_data_ptr, 16);
        // EE_LD_128_USAR_IP(q2, i_data_ptr, 16);
        __builtin_xtensa_ee_ld_128_usar_ip(2, i_data_ptr, 16);
        asm("   dotprod_s16_32n: nop");
        // EE_SRC_Q_LD_IP(q3, i_data_ptr, 16, q5, q2);
        __builtin_xtensa_ee_src_q_ld_ip(3, i_data_ptr, 16, 5, 2);
        for (size_t y = 0; y < total_y; y++)
        {
            for (size_t x = 0; x < total_x; x++)
            {
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q4, i_data_ptr, 16, q0, q5, q2, q3);// acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(4, i_data_ptr, 16, 0, 5, 2, 3);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q5, i_data_ptr, 16, q1, q2, q3, q4);// acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(5, i_data_ptr, 16, 1, 2, 3, 4);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q2, i_data_ptr, 16, q0, q3, q4, q5);// acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(2, i_data_ptr, 16, 0, 3, 4, 5);
                    // EE_VLD_128_IP(q1, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
                    // EE_VMULAS_S16_ACCX_LD_IP_QUP (q3, i_data_ptr, 16, q1, q4, q5, q2);// acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
                    __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 1, 4, 5, 2);
                    // EE_VLD_128_IP(q0, f_data_ptr, 16);
                    __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            }
            // EE_VMULAS_S16_ACCX_LD_IP_QUP(q4, i_data_ptr, 16, q0, q5, q2, q3); // acc = q0*q5, q4 = i_data, q2 = q2|q3 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(4, i_data_ptr, 16, 0, 5, 2, 3);
            // EE_VLD_128_IP(q1, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_XP_QUP(q5, i_data_ptr, update_x_0, q1, q2, q3, q4); // acc = q1*q2, q5 = i_data, q3 = q3|q4 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(5, i_data_ptr, 16, 1, 2, 3, 4);
            // EE_VLD_128_IP(q0, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_XP_QUP(q2, i_data_ptr, update_x_1, q0, q3, q4, q5); // acc = q0*q3, q2 = i_data, q4 = q4|q5 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(2, i_data_ptr, 16, 0, 3, 4, 5);
            // Load q5 again
            // EE_LD_128_USAR_XP(q5, i_data_ptr, update_x_2);
            __builtin_xtensa_ee_ld_128_usar_xp(5, i_data_ptr, update_x_2);
            // EE_VLD_128_IP(q1, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(1, f_data_ptr, 16);
            // EE_VMULAS_S16_ACCX_LD_IP_QUP(q3, i_data_ptr, 16, q1, q4, q5, q2); // acc = q1*q4, q3 = i_data, q5 = q5|q2 >> sar
            __builtin_xtensa_ee_vmulas_s16_accx_ld_ip_qup(3, i_data_ptr, 16, 1, 4, 5, 2);
            // EE_VLD_128_IP(q0, f_data_ptr, 16);
            __builtin_xtensa_ee_vld_128_ip(0, f_data_ptr, 16);
        }
    }

    int64_t acc;
    // temp_data[0] = RUR_ACCX_0();
    __builtin_xtensa_rur_accx_0(temp_data[0]);
    // temp_data[1] = RUR_ACCX_1();
    __builtin_xtensa_rur_accx_1(temp_data[1]);
    acc = *(int64_t*)temp_data;
    if (shift > 0)
    {
        temp_data[0] = acc >> (shift-1);
        temp_data[0] += 1;
        *out_value = temp_data[0]>>1;
    }
    else
    {
        *out_value = acc;
    }
    return ESP_OK;
}

void app_main(void)
{
    printf("Hello world!\n");

    // Print chip information
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}


