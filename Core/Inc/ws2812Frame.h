#ifndef ws2812Frame_h
#define ws2812Frame_h
 
#include "main.h"
#include "stdint.h"
#include "string.h"
 
#define WS_BIT_1            0xFC
#define WS_BIT_0            0xC0
#define WS_RESET            0x00
 
#define WS_DATALENGTH          24
 
struct SWS2812_Struct {
    void (*WS2812DataTransmit)(uint8_t *pData, uint16_t dataSize);
    uint8_t num;
    uint8_t index;
    uint8_t *sendBuff;
};
 
typedef struct SWS2812_Struct WS2812_Struct;
typedef WS2812_Struct *PWS2812_Struct;
 
#define WS2812_INIT(name,xNum,xSendBuff,xWS2812DataTransmit)      \
WS2812_Struct name = {                                      \
    .num = xNum,                                            \
    .index = 0,                                             \
    .sendBuff = xSendBuff,                                   \
    .WS2812DataTransmit = xWS2812DataTransmit,                  \
};
 
/*************************************************************
** Function name:       WS2812_TIMEBASE
** Descriptions:        时基,放在周期为1ms的函数里面执行
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*************************************************************/
#define WS2812_TIMEBASE(ms)                \
        gWS2812_TimeCNT+= ms
 
/*************************************************************
** Function name:       WS2812_GETTIME
** Descriptions:        获取起始时间
** Input parameters:    None
** Output parameters:   None
** Returned value:      (uint32_t)起始时间
*************************************************************/
#define WS2812_GETTIME(void)                        \
    gWS2812_TimeCNT
 
/*************************************************************
** Function name:       WS2812_TIMEOUT
** Descriptions:        检查超时
** Input parameters:    timeOut：(uint32_t)超时时间
**                      startTime:(uint32_t)开始的时间
** Output parameters:   None
** Returned value:      false,未超时，true，超时
*************************************************************/
#define WS2812_TIMEOUT(timeOut,startTime)                \
    ((gWS2812_TimeCNT - startTime) >= timeOut ? 1 : 0)
 
 
void SetWSColor(PWS2812_Struct pWs2812, uint8_t index, uint8_t R,uint8_t G,uint8_t B);
void ClearAllColor(PWS2812_Struct pWs2812);
void ClearIndexColor(PWS2812_Struct pWs2812,uint8_t index);
void WS2812SendDataLoop(PWS2812_Struct ws2812);
void WS2812SendMassge(uint8_t *sendBuff, uint16_t sendSize);
	 
extern uint32_t gWS2812_TimeCNT;
 
 
#endif /* ws2812Frame_h */