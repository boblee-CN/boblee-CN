#include "ws2812Frame.h"
#include "spi.h"

uint32_t gWS2812_TimeCNT;

// GRB格式
 
/*************************************************************
** Function name:       WS2812SendMassge
** Descriptions:        通过SPI发送WS2812的数据
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void WS2812SendMassge(uint8_t *sendBuff, uint16_t sendSize){
 
    HAL_SPI_Transmit_DMA(&hspi1, sendBuff, sendSize);
}
 
 
/*************************************************************
** Function name:       CreatData
** Descriptions:        组合数据 并拷贝到BUFF中对应的位置
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
//实际BRG
static void CreatData(PWS2812_Struct pWs2812, uint8_t index, uint8_t R,uint8_t G,uint8_t B){
    // 先组合成8*3个字节的数据
    uint8_t temp[24] = {0};
    for (uint8_t i=0;i<8;i++){
        temp[7-i] =  (B & 0x01) ? WS_BIT_1 : WS_BIT_0; 
        B = B >> 1;
    }
    for (uint8_t i=0;i<8;i++){
        temp[15-i] =  (R & 0x01) ? WS_BIT_1 : WS_BIT_0; 
        R = R >> 1;
    }
    for (uint8_t i=0;i<8;i++){
        temp[23-i] =  (G & 0x01) ? WS_BIT_1 : WS_BIT_0; 
        G = G >> 1;
    }
    // 拷贝到对应的Buff中
    memcpy(&pWs2812->sendBuff[index*24], temp, 24);
}
 
/*************************************************************
** Function name:       SetWSColor
** Descriptions:        设置WS2812颜色
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SetWSColor(PWS2812_Struct pWs2812, uint8_t index, uint8_t R,uint8_t G,uint8_t B){
    CreatData(pWs2812,index,R,G,B);
}
 
 
/*************************************************************
** Function name:       ClearAllColor
** Descriptions:        清除所有颜色（关闭灯光）
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void ClearAllColor(PWS2812_Struct pWs2812){
    for (uint8_t i=0;i<pWs2812->num;i++){
        CreatData(pWs2812,i,0,0,0);
    }
}
 
 
/*************************************************************
** Function name:       ClearIndexColor
** Descriptions:        清除指定WS2812的颜色
** Input parameters:    index ： 灯珠的序号  从0开始
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void ClearIndexColor(PWS2812_Struct pWs2812,uint8_t index){
    if (index >= pWs2812->num){
        return ;
    }
    CreatData(pWs2812,index,0,0,0);
}
 
 
/*************************************************************
** Function name:       WS2812SendDataLoop
** Descriptions:        WS2812数据发送函数，需要放到循环中
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void WS2812SendDataLoop(PWS2812_Struct ws2812){
    static uint32_t startTime = 0;
    if  ( WS2812_TIMEOUT(10,startTime) ){
        // 这里会指向SPI的发送函数  最上面的第一个函数
        ws2812->WS2812DataTransmit(ws2812->sendBuff, ws2812->num * WS_DATALENGTH);
        startTime = WS2812_GETTIME();
    }
}