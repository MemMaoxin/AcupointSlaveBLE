#ifndef _BOARD_RTC_H_
#define _BOARD_RTC_H_

/*********************************************************************
 * INCLUDES
 */
#include "sdk_common.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * API FUNCTIONS
 */
void RTC_Init(void);
void RTC_SetTime(uint32_t timestampNow);
uint32_t RTC_GetTime(void);
void RTC_Enable(uint32_t timestampNow);
void RTC_Disable(void);

#endif /* _BOARD_RTC_H_ */
