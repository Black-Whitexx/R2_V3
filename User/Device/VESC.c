//
// Created by BxW on 2024/6/11.
//

//PS.openloop control

#include "VESC.h"

int32_t rpm_status;

uint8_t VESC_Set(int32_t rpm)
{
    rpm_status = rpm;
    static FDCAN_TxHeaderTypeDef txHeader;
    static uint32_t mbox;
    static uint32_t tx_id;
    static union_32 temp;
    static uint8_t txData[8];

    tx_id = VESC_ID| (CAN_PACKET_SET_RPM) << 8;

    txHeader.Identifier=tx_id;                       //32位ID
    txHeader.IdType=FDCAN_EXTENDED_ID;                  //远程帧ID
    txHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    txHeader.DataLength= FDCAN_DLC_BYTES_4;             //数据长度4字节
    txHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    txHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    txHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    txHeader.MessageMarker=0x00;                        //不管捏

    // 数据先行位为倒序
    temp.data_int = rpm;
    txData[0] = temp.data_8[3];
    txData[1] = temp.data_8[2];
    txData[2] = temp.data_8[1];
    txData[3] = temp.data_8[0];

    //等一个空の邮箱呢
    while(HAL_FDCAN_GetTxFifoFreeLevel(VESC_CAN_PTR) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_FDCAN_AddMessageToTxFifoQ(VESC_CAN_PTR, &txHeader, txData) != HAL_OK)
    {
//        printf("Error\n");
        Error_Handler();
    }
    return 0;
}

uint8_t VESC_Read(int32_t *rpm){
    *rpm = rpm_status;
    return 0;
}