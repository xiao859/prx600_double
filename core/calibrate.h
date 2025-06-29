#ifndef _CALIBRATE_H
#define _CALIBRATE_H

#include "stdint.h"


typedef struct
{
    uint16_t store_flag; // ä¿å­˜æ•°æ®æ ‡å¿—ï¼Œéœ€è¦å­˜å‚¨æŸ¥æ‰¾è¡¨ï¼Œæ ¹æ®éœ€è¦ç½®ä¸º1~3ï¼Œå­˜å‚¨å®Œæ¯•ï¼Œç½®ä¸º0ï¼Œåˆå§‹åŒ–ä¸º0
    uint16_t calibraflag;//è‡ªåŠ¨è·Ÿæ–°
    uint16_t calibr_enble_flag;//è·Ÿæ–°ä½¿èƒ½
    uint16_t calibr_expo1_flag;
    uint16_t calibr_expo2_flag;
    uint32_t calibr_enble_count;
    uint16_t calibr_expo1_count;
    uint16_t calibr_expo2_count;
    uint32_t calibr_lampctrl_count;
    uint32_t calibr_wait_count;
}ctrl_calibr;

extern volatile ctrl_calibr ctrl_calibr_data;

void Autocalibrationcount(void);

///* 3¡¢Êı¾İÀàĞÍ¶¨Òå */
//typedef struct {
//    xray_mode   mode;                   /* Ğ£×¼Ä£Ê½£ºÂö³å0»òÕßÁ¬Ğø1 */
//    uint8_t     curr_index[XRAY_NUMS];             /* Ğ£×¼½×¶Î0~9´ú±í */
//    uint8_t     finished_flag[XRAY_NUMS];          /* ½áÊø±ê¼Ç */

//    float       tube_vol_realtime[XRAY_NUMS];
//    float       tube_vol_step[XRAY_NUMS];

//    uint32_t    timmer_count[XRAY_NUMS];           /* ¼ÆÊ±£¬ÓÃÓÚÀäÈ´»òÕß¿ØÖÆÆØ¹â */
//    uint32_t    cycle_count[XRAY_NUMS];            /* ¼ÆÊı£¬µ¥ºÁ°²Ñ­»· */

//    uint32_t    expoTime_expect[XRAY_NUMS];        /* ÉèÖÃµÄÆØ¹âÊ±¼ä£¬µ¥¸öµçÁ÷ÏÂ£¬Ã¿¸öÂö³åµÄ³ÖĞøÊ±¼ä */
//    uint32_t    coolTime_expect[XRAY_NUMS];        /* ÉèÖÃµÄÀäÈ´Ê±¼ä£¬µ¥¸öµçÁ÷ÏÂ£¬Ã¿¸öÂö³å¼äµÄÀäÈ´Ê±¼ä */
//    uint32_t    expoCycle_perCurrent[XRAY_NUMS];   /* Ğ£×¼£¬µ¥ºÁ°²ÆØ¹â´ÎÊı */

//    uint32_t    para_save_flag[XRAY_NUMS];
//} xray_calibrate_data;
//extern volatile xray_calibrate_data cali_data;

///* 4¡¢º¯ÊıÉùÃ÷ */
//void calibrate_para_init(void);
//void calibrate_task(void);

#endif
