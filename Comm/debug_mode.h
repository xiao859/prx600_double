#ifndef _DEBUG_MODE_H
#define _DEBUG_MODE_H


#include "stdint.h"
#include "HV_exposure.h"


#define IDLE_FILAMENT_REF_DEBUG       0



typedef struct {
    uint32_t    timmer_count[XRAY_NUMS];           
    uint32_t    cycle_count[XRAY_NUMS];            

    uint32_t    expoTime_expect[XRAY_NUMS];        
    uint32_t    coolTime_expect[XRAY_NUMS];       
    uint32_t    expoCycle_perCurrent[XRAY_NUMS];   /**/
} xray_debug_data;
extern volatile xray_debug_data debug_data;


void debug_task(void);
void xray_HV_enable_debug(uint16_t value);

#endif
