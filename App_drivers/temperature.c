/*
 *  temperature.c
 *
 *  Created on: Jun 13, 2025
 *  Author: Administrator
 *
 */
#include "temperature.h"

/* r=(20*AD*3.3*5/4095)/(15-AD*3.3*5/4095)  */
float get_Resist(float temp_ad)
{
    return (float)((3300 * temp_ad) / (614250 - 165 * temp_ad));//468375
}

/*
-30 ~ -20   153.61-89.776  y = -0.15666x - 5.93546
-20 ~ -10   89.776-53.198  y = -0.27339x + 4.54386
-10 ~ -0    53.198-32.116  y = -0.47434x + 15.23394
0   ~ 10    32.116-19.783  y = -0.81083x + 26.04062
10  ~ 20    19.783-12.487  y = -1.37061x + 37.11478
20  ~ 30    12.487-8.052  y = -2.25479x + 48.15556
30  ~ 40    8.052-5.309  y = -3.64564x + 59.35469
40  ~ 50    5.309-3.588  y = -5.81058x + 70.84837
50  ~ 60    3.588-2.457  y = -8.84173x + 81.72413
60  ~ 70    2.457-1.721  y = -13.58696x + 93.38316
70  ~ 80    1.721-1.288  y = -20.28398x + 104.90873
*/
float temp_translate(float Resist)
{
    float temp;
    if (Resist > 89.776f) {
        temp = -0.15666 * Resist - 5.93546;
    } else if (Resist > 53.198f) {
        temp = -0.27339 * Resist + 4.54386;
    } else if (Resist > 32.116f) {
        temp = -0.47434 * Resist + 15.23394;
    } else if (Resist > 19.783f) {
        temp = -0.81083 * Resist + 26.04062;
    } else if (Resist > 12.487f) {
         temp = -1.37061 * Resist + 37.11478;
    } else if (Resist > 8.052f) {
        temp = -2.25479 * Resist + 48.15556;
    } else if (Resist > 5.309f) {
        temp = -3.64564 * Resist + 59.35469;
    } else if (Resist > 3.588f) {
         temp = -5.81058 * Resist + 70.84837;
    } else if (Resist > 2.457f) {
        temp = -8.84173 * Resist + 81.72413;
    } else if (Resist > 1.721f) {
        temp = -13.58696 * Resist + 93.38316;
    } else if (Resist > 1.288f) {
        temp = -20.28398 * Resist + 104.90873;
    }

    return temp;
}

float get_temp(float temp_ad)
{
    float Resist = get_Resist(temp_ad);

    return temp_translate(Resist);
}

