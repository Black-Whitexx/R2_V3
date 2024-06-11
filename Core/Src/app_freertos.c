/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t str_flag;
/* USER CODE END Variables */
osThreadId Debug_TaskHandle;
osThreadId RoboRun_TaskHandle;
osThreadId NRF_TaskHandle;
osThreadId Control_TaskHandle;
osThreadId HandleBall_TaskHandle;
osThreadId Suction_TaskHandle;
osThreadId Run1to3_TaskHandle;
osThreadId Vision_TaskHandle;
osThreadId VisionRun_TaskHandle;
osThreadId Start_TaskHandle;
osMessageQId NRF_RX_QueueHandle;
osMessageQId SuctionSpeed_QueueHandle;
osMessageQId VisionData_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DebugTask(void const * argument);
void RoboRunTask(void const * argument);
void NRFTask(void const * argument);
void ControlTask(void const * argument);
void HandleBallTask(void const * argument);
void SuctionTask(void const * argument);
void Run1to3Task(void const * argument);
void VisionTask(void const * argument);
void VisionRunTask(void const * argument);
void StartTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of NRF_RX_Queue */
  osMessageQDef(NRF_RX_Queue, 1, RemoteRXSturct);
  NRF_RX_QueueHandle = osMessageCreate(osMessageQ(NRF_RX_Queue), NULL);

  /* definition and creation of SuctionSpeed_Queue */
  osMessageQDef(SuctionSpeed_Queue, 1, int16_t);
  SuctionSpeed_QueueHandle = osMessageCreate(osMessageQ(SuctionSpeed_Queue), NULL);

  /* definition and creation of VisionData_Queue */
  osMessageQDef(VisionData_Queue, 1, VisionStruct);
  VisionData_QueueHandle = osMessageCreate(osMessageQ(VisionData_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, DebugTask, osPriorityNormal, 0, 1024);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of RoboRun_Task */
  osThreadDef(RoboRun_Task, RoboRunTask, osPriorityNormal, 0, 1024);
  RoboRun_TaskHandle = osThreadCreate(osThread(RoboRun_Task), NULL);

  /* definition and creation of NRF_Task */
  osThreadDef(NRF_Task, NRFTask, osPriorityNormal, 0, 256);
  NRF_TaskHandle = osThreadCreate(osThread(NRF_Task), NULL);

  /* definition and creation of Control_Task */
  osThreadDef(Control_Task, ControlTask, osPriorityNormal, 0, 1024);
  Control_TaskHandle = osThreadCreate(osThread(Control_Task), NULL);

  /* definition and creation of HandleBall_Task */
  osThreadDef(HandleBall_Task, HandleBallTask, osPriorityNormal, 0, 1024);
  HandleBall_TaskHandle = osThreadCreate(osThread(HandleBall_Task), NULL);

  /* definition and creation of Suction_Task */
  osThreadDef(Suction_Task, SuctionTask, osPriorityNormal, 0, 512);
  Suction_TaskHandle = osThreadCreate(osThread(Suction_Task), NULL);

  /* definition and creation of Run1to3_Task */
  osThreadDef(Run1to3_Task, Run1to3Task, osPriorityNormal, 0, 1024);
  Run1to3_TaskHandle = osThreadCreate(osThread(Run1to3_Task), NULL);

  /* definition and creation of Vision_Task */
  osThreadDef(Vision_Task, VisionTask, osPriorityNormal, 0, 1024);
  Vision_TaskHandle = osThreadCreate(osThread(Vision_Task), NULL);

  /* definition and creation of VisionRun_Task */
  osThreadDef(VisionRun_Task, VisionRunTask, osPriorityNormal, 0, 2048);
  VisionRun_TaskHandle = osThreadCreate(osThread(VisionRun_Task), NULL);

  /* definition and creation of Start_Task */
  osThreadDef(Start_Task, StartTask, osPriorityNormal, 0, 1024);
  Start_TaskHandle = osThreadCreate(osThread(Start_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  vTaskSuspend(RoboRun_TaskHandle);
//  vTaskSuspend(Vision_TaskHandle);
  vTaskSuspend(VisionRun_TaskHandle);
  vTaskSuspend(Start_TaskHandle);
//  vTaskSuspend(NRF_TaskHandle);
//  vTaskSuspend(Control_TaskHandle);
//  vTaskSuspend(HandleBall_TaskHandle);
//  vTaskSuspend(Suction_TaskHandle);
  vTaskSuspend(Run1to3_TaskHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_DebugTask */
/**
  * @brief  此任务用于一些参数和外设的初始化以及使用串口和LED调试
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DebugTask */
void DebugTask(void const * argument)
{
  /* USER CODE BEGIN DebugTask */
  /** 等待NRF校准 **/
  while (NRF24L01_Check())
  {
      printf("no\n");
  }
  NRF24L01_RX_Mode();
  PID_Set(&Wheels[0],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[1],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[2],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[3],7.2f,0.1f,1.0f,10000);

  PID_Set(&Slope_Speed_t,7.2f,0.15f,2.2f,10000);
  PID_Set(&Slope_Position_t,1.85f,0,0.8f,0);
  PID_Set(&Toggle_Speed_t,7.2f,0.5f,2.0f,10000);
  PID_Set(&Toggle_Position_t,0.8f,0,0.8f,0);

  PID_Set(&Left_Speed_t,3,0.14f,2.2f,10000);
  PID_Set(&Right_Speed_t,3,0.14f,2.2f,10000);

  PID_Set(&Translation_PID,1.35f,0.0f,0.3f,0.0f);
  PID_Set(&Turn_PID,0.035f,0.0f,0.2f,0.0f);

  PID_Set(&VisionRun2,1.35f,0.0f,0.3f,0.0f);

  PID_Set(&VisionPID_X,0.0020f,0.0f,0.0f,0.0f);
  /* Infinite loop */
  for(;;)
  {
    LED0_Flashing;
//    printf("X:%f,Y:%f,Angle:%f\n",LiDar.locx,LiDar.locy,LiDar.yaw);
    osDelay(100);
  }
  /* USER CODE END DebugTask */
}

/* USER CODE BEGIN Header_RoboRunTask */
/**
* @brief 此任务用于R2跑点或�?�遥�?????????????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RoboRunTask */
void RoboRunTask(void const * argument)
{
  /* USER CODE BEGIN RoboRunTask */
    RemoteRXSturct NRFRX_Data;
  /* Infinite loop */
  for(;;)
  {
    if(Control_Mode == AutoRun_Mode)//跑点模式
    {
        if( Distance_Calc(Aim_Points[AimPoints_Index],LiDar.locx,LiDar.locy) < 0.02f && fabsf(LiDar.yaw - Aim_Points[AimPoints_Index].angle) < 0.5f )
        {
            cnt = 0;
            Car_Stop;
            AimPoints_Index ++;
            if(State == Run2Store_State) //从绿区到黄区的跑点，用于去放球，到点后切换状态为Store_State
            {
                State = Store_State;
                vTaskSuspend(RoboRun_TaskHandle);
            }
            else
            {
                vTaskSuspend(RoboRun_TaskHandle);
            }
        }
        else
        {
            Chassis_Move(&Aim_Points[AimPoints_Index]);
        }
    }
    else//遥控模式
    {
        if( xQueueReceive(NRF_RX_QueueHandle, &NRFRX_Data, 0) == pdTRUE )//读取队列中NRF传来的数据，从NRF任务中写
        {
            SGW2Wheels((float)NRFRX_Data.rx * 3 / 128,(float)NRFRX_Data.ry * 3 / 128,(float)NRFRX_Data.lx * 3 / 128,0);
        }
    }
    osDelay(5);
  }
  /* USER CODE END RoboRunTask */
}

/* USER CODE BEGIN Header_NRFTask */
/**
* @brief 此函数用于接收遥控器的数�??
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRFTask */
void NRFTask(void const * argument)
{
  /* USER CODE BEGIN NRFTask */
    /** 定义存储数据变量 **/
    static uint8_t rc_data[RX_PLOAD_WIDTH] = {0};//接收数组缓冲
    static RemoteRXSturct RemoteRX;//用于存遥控器传来的数
    uint8_t cmd;
    /* Infinite loop */
  for(;;)
  {
      if (NRF24L01_RxPacket(rc_data) == 0)  //接收遥控器数据，若收到返0，若没收到返1
      {
          /** 读取左右摇杆值，限制�??-128~128 **/
          RemoteRX.lx = (int16_t)-(rc_data[1] - 128);
          RemoteRX.ly = (int16_t)-(rc_data[2] - 128);
          RemoteRX.rx = (int16_t)-(rc_data[3] - 128);
          RemoteRX.ry = (int16_t)-(rc_data[4] - 128);
          /** 接收遥控器按键命 **/
          RemoteRX.command = rc_data[5];
          /** 设置摇杆值的死区 **/
          if (abs(RemoteRX.rx) < 2) RemoteRX.rx = 0;
          if (abs(RemoteRX.ry) < 2) RemoteRX.ry = 0;
          if (abs(RemoteRX.lx) < 2) RemoteRX.lx = 0;
          if (abs(RemoteRX.ly) < 2) RemoteRX.ly = 0;
          /** 对遥控器进行滤波，原因是遥控器有莫名其妙的电平跳 **/
          if (RemoteRX.rx == -5) RemoteRX.rx = 0;
          if (RemoteRX.ry == -4) RemoteRX.ry = 0;
          if (RemoteRX.lx == -4) RemoteRX.lx = 0;
          if (RemoteRX.ly == -4) RemoteRX.ly = 0;

          xQueueOverwrite(NRF_RX_QueueHandle, &RemoteRX);/** 把遥控器的数据放入队列，用于遥控 **/

          /** 对遥控器按键命令进行响应 **/
          switch (RemoteRX.command) {
              case Left_Up_Up://切换成手动模�??
                  Control_Mode = Manual_Mode;
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Left_Up://�??键启�??
                  SUCTION_ON;  /** 伸出吸球机构 **/
                  Toggle_Pos = Toggle_Mid; /** 夹爪翻到中位 **/
                  osDelay(500);     /** 避免夹爪提前打开 **/
                  CLAW_OFF;                /** 打开夹爪 **/
                  vTaskResume(Start_TaskHandle);
                  vTaskSuspend(NRF_TaskHandle);
                  break;
              case Right_Up:
                  cmd = 0x01;
                  HAL_UART_Transmit(&huart2,&cmd, sizeof(cmd),0xfffff);
                  break;
              case Right_Down:
                  OpenSuction();
                  break;
              case Right_Right:
                  CloseSuction();
                  break;
              default:
                  break;
          }
//          printf("%d,%d\n",RemoteRX.command,RemoteRX.ly);/** 用于调试遥控器按 **/
          RemoteRX.command = 0; //重置命令
      }
    osDelay(10);
  }
  /* USER CODE END NRFTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief 此任务用于运动状态的判断切换跑点模式或视觉追踪模�????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void const * argument)
{
  /* USER CODE BEGIN ControlTask */
  uint8_t vision_cmd = 0;
  /* Infinite loop */
  for(;;)
  {
      /** 对当前状态做出反�?? **/
        switch (State) {
            case Default_State:
                break;
            case Run2Get_State:
                Set_Point(&Vision_Points[0],-1.43f,1.49f,90,0);
                vision_cmd = 0x02;
                HAL_UART_Transmit(&huart2,&vision_cmd, sizeof(vision_cmd),0xfffff);
                vTaskResume(VisionRun_TaskHandle);
                vTaskResume(Vision_TaskHandle);
                while (State == Run2Get_State){ osDelay(1);}
                break;
            case Run2Store_State:
//                if (Store_Flag == 1)
//                {
//                    vTaskResume(RoboRun_TaskHandle);
//                }
//                cnt = 0;
//                Set_Point(&Aim_Points[AimPoints_Index],0.30f,1.45f,90,0);
//                vTaskSuspend(Vision_TaskHandle);
//                vTaskResume(RoboRun_TaskHandle);
//                while (State == Run2Store_State){ osDelay(1);}
                break;
            case TakeRightBall_State:/** 取正确的�???? **/
                osDelay(500);
                CLAW_ON;//关闭夹爪
                cnt = 0;
                Set_Point(&Aim_Points[AimPoints_Index],0.30f,1.45f,90,0);
                vTaskSuspend(Vision_TaskHandle);
                vTaskResume(RoboRun_TaskHandle);
                CloseSuction();
                State = Run2Store_State;//状�?�切换为Run2Store_State
                Toggle_Pos = Toggle_Up;//夹爪翻上
                osDelay(500);
                SUCTION_OFF;//吸球机构推回
                while (State == Run2Store_State){ osDelay(1);}
                break;
            case Store_State://放球
                CLAW_OFF;//打开夹爪
                osDelay(1000);//等待球滚出去
                State = Run2Get_State;//状�?�切换为Run2Get_State
                Toggle_Pos = Toggle_Mid;//夹爪回中�????
                SUCTION_ON;//吸球机构推出
                break;
            default:
                break;
        }
    osDelay(10);
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_HandleBallTask */
/**
* @brief 此任务用于控制电机实现左右拨球或者夹�?????????????/放球，控�?????????????3�?????????????2006和一�?????????????3508电机、两个气�?????????????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleBallTask */
void HandleBallTask(void const * argument)
{
  /* USER CODE BEGIN HandleBallTask */
  int16_t Left_Speed,Slope_Speed,Toggle_Speed;
  float Slope_Temp,Toggle_Temp;
  /* Infinite loop */
  for(;;)
  {
//    Left_Speed = (int16_t)PID_Realise(&Left_Speed_t, Left_TargetSpe, DjiMotor_Info[5].speed, M2006_CURRENT_MAX, 5);
//
//    Slope_Temp = PID_Realise(&Slope_Position_t, Slope_Pos, DjiMotor_Info[6].actual_total_angle, 2000, 10.0f);
//    Slope_Speed = (int16_t)PID_Realise(&Slope_Speed_t, Slope_Temp, DjiMotor_Info[6].speed, M2006_CURRENT_MAX, 5);
//
//    Toggle_Temp = PID_Realise(&Toggle_Position_t, Toggle_Pos, DjiMotor_Info[7].actual_total_angle, 1000, 5.0f);
//    Toggle_Speed = (int16_t)PID_Realise(&Toggle_Speed_t, Toggle_Temp, DjiMotor_Info[7].speed, M3508_CURRENT_MAX, 5);
//
//    Set_Current(&hfdcan2,0x1FF,0,Left_Speed,Slope_Speed,Toggle_Speed);
//    osDelay(10);
  }

  /* USER CODE END HandleBallTask */
}

/* USER CODE BEGIN Header_SuctionTask */
/**
* @bri
 * ef 此任务用于吸球，控制VESC电调驱动�?????????????�?????????????5065电机
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SuctionTask */
void SuctionTask(void const * argument)
{
  /* USER CODE BEGIN SuctionTask */
  int16_t suctionSpeed = 0;//VESC速度
  /* Infinite loop */
  for(;;)
  {
      if(xQueuePeek(SuctionSpeed_QueueHandle,&suctionSpeed,0) == pdTRUE)
      {
          Vesc_SetSpeed(&hfdcan1,VESC_ID,suctionSpeed);
      }
      osDelay(5);
  }
  /* USER CODE END SuctionTask */
}

/* USER CODE BEGIN Header_Run1to3Task */
/**
* @brief 此任务用于比赛开始时R2�???1区跑�???3�???
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Run1to3Task */
void Run1to3Task(void const * argument)
{
  /* USER CODE BEGIN Run1to3Task */
  uint8_t index = 0;
  /* Infinite loop */
  for(;;)
  {
      if( Distance_Calc(Run1to3_Points[index],LiDar.locx,LiDar.locy) < 0.10f && fabsf(LiDar.yaw - Run1to3_Points[index].angle) < 0.5f )
      {
          index++;
          if(index == 5)
          {
              vTaskSuspend(Run1to3_TaskHandle);
          }
      }
      else
      {
          Chassis_Move(&Run1to3_Points[index]);
      }
    osDelay(5);
  }
  /* USER CODE END Run1to3Task */
}

/* USER CODE BEGIN Header_VisionTask */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisionTask */
void VisionTask(void const * argument)
{
  /* USER CODE BEGIN VisionTask */
  float Speed_x,omega;
  VisionStruct visiondata;
  static uint8_t last_state;
  /* Infinite loop */
  for(;;)
  {
      if( xQueueReceive(VisionData_QueueHandle,&visiondata,0) == pdTRUE )
      {
          if (visiondata.flag == 1)//在黄区找到球�?
          {
              printf("1,1\n");
              cnt = 0;
              State = Run2Get_State2;
              Set_Point(&Vision_Points[0], -visiondata.vision_y / 1000.0f + LiDar.locx,visiondata.vision_x / 1000.0f + LiDar.locy, 90, 0);
              vTaskResume(VisionRun_TaskHandle);
              vTaskSuspend(Vision_TaskHandle);
          }
          else if (visiondata.flag == 2)
          {
              switch ((int16_t) (visiondata.vision_y)) {
                  case 1:/** 看到蓝色球了，平台回正，把夹爪放下去 **/
                      printf("2,1\n");
                      OpenSuction();
                      Toggle_Pos = Toggle_Down;/** 夹爪放下来，平台回正 **/
                      Slope_Pos = 0;
                      osDelay(100);/** 前进�?段时间后停车 **/
                      Car_Stop;
                      break;
                  case 2:/** 快进来的是无效球，停车等待球吐出�? **/
                      printf("2,2\n");
                      OpenSuction();
                      Toggle_Pos = Toggle_Mid;/** 爪子回中�? **/
                      osDelay(100);/** 前进�?段时间后停车 **/
                      Car_Stop;
                      break;
                  case 0:/** 跟踪�? **/
                      printf("2,0\n");
                      OpenSuction();
                      if (Vision_State != Vision_Delay) {
                          if (str_flag == 1)/** 对球完毕 **/
                          {
                              Speed_x = -PID_Realise(&VisionPID_X, 0, Vision_Data.vision_x, 1.0f, 3);
                              omega = PID_Realise(&Turn_PID, 90, LiDar.yaw, 0.5f, 0.5f);
                              SGW2Wheels(Speed_x, 0.5f, omega, 0);
                          } else/** 还没对好 **/
                          {
                              Speed_x = -PID_Realise(&VisionPID_X, 0, Vision_Data.vision_x, 1.0f, 3);
                              omega = PID_Realise(&Turn_PID, 90, LiDar.yaw, 0.5f, 0.5f);
                              SGW2Wheels(Speed_x, 0, omega, 0);
                              if (fabsf(Vision_Data.vision_x) < 8.0f) {
                                  str_flag = 1;
                              }
                          }
                      }
                      break;
                  default:
                      break;
              }
          }
          else if (visiondata.flag == 3)
          {
              switch ((int16_t) (visiondata.vision_y)) {
                  case 6:/** 进来的是有效球，进入TakeRightBall_State **/
                      printf("3,6\n");
                      str_flag = 0;
                      State = TakeRightBall_State;
                      vTaskSuspend(Vision_TaskHandle);
                      break;
                  case 7:/** 车内没球 **/
                      if (Vision_State == Vision_Delay) {
                          OpenSuction();
                          Vision_State = 0;
                      }
                      break;
                  case 2:/** 紫球还在里面 **/
                      printf("3,2\n");
                      str_flag = 0;
                      Vision_State = Vision_Delay;
                      CloseSuction();
                      Car_Stop;
                      Toggle_Pos = Toggle_Mid;
                      break;
                  case 3:/** 视野里没有球，向后�?? **/
                      printf("3,3\n");
                      Vision_State = Vision_FindBall;
                      State = Run2Get_State2;
                      Set_Point(&Vision_Points[0], -1.43f, 1.49f, 90, 0);
                      vTaskResume(VisionRun_TaskHandle);
                      break;
                  default:
                      break;
              }
          }
          visiondata.flag = 0;
      }
    osDelay(5);
  }
  /* USER CODE END VisionTask */
}

/* USER CODE BEGIN Header_VisionRunTask */
/**
* @brief 该任务是视觉跑点，由于对精度要求不高且�?�度要求高，�???以单�???�???个任�???
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisionRunTask */
void VisionRunTask(void const * argument)
{
  /* USER CODE BEGIN VisionRunTask */
  uint8_t vision_cmd=0;
  /* Infinite loop */
  for(;;)
  {
    if( Distance_Calc(Vision_Points[0],LiDar.locx,LiDar.locy) < 0.1f && fabsf(LiDar.yaw - Vision_Points[0].angle) < 1.0f )
    {
        cnt = 0;
        Car_Stop;
        if( (Vision_State == Vision_FindBall) || (State == Run2Get_State) || State == (Run2Get_State2)) /** 从黄区到绿区的跑点，用于去找球，到点后启�?5065，开启视觉任�? **/
        {
            Slope_Pos = Slope_Left;//平台倾斜
            Left_TargetSpe = Left_Spe;//2006旋转
            OpenSuction();
            vTaskResume(Vision_TaskHandle);

            if(State != Run2Get_State)
            {
                vision_cmd = 0x02;
                HAL_UART_Transmit(&huart2,&vision_cmd, sizeof(vision_cmd),0xFFFFF);
                printf("send:%d\n",vision_cmd);
            }

            State = 0;
            Vision_State = 0;

            printf("VS:%d,State:%d\n",Vision_State,State);
            vTaskSuspend(VisionRun_TaskHandle);
        }
    }
    else
    {
//        printf("%f,x:%f,y:%f,%f\n",Distance_Calc(Vision_Points[0],LiDar.locx,LiDar.locy),LiDar.locx,LiDar.locy,LiDar.yaw);
        Chassis_Move_OfVision(&Vision_Points[0]);
    }
    osDelay(5);
  }
  /* USER CODE END VisionRunTask */
}

/* USER CODE BEGIN Header_StartTask */
/**
* @brief Function implementing the Start_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask */
void StartTask(void const * argument)
{
  /* USER CODE BEGIN StartTask */
  uint8_t index = 0,vision_cmd = 0;
  PointStruct Start_Points[2] = {
          {.x = -2.94f,.y = 1.43f,.angle = 90.0f,.num = 0},
          {.x = -4.10f,.y = 1.43f,.angle = 90.0f,.num = 0}
  };
  /* Infinite loop */
  for(;;)
  {
        if( Distance_Calc(Start_Points[index],LiDar.locx,LiDar.locy) < 0.10f && fabsf(LiDar.yaw - Start_Points[index].angle) < 0.5f )
        {
            index ++;
            cnt = 0;
            if(index == 1)
            {
                Car_Stop;    /** 停车 **/
                Slope_Pos = Slope_Left; /** 平台向左倾斜 **/
                Left_TargetSpe = Left_Spe; /** 2006�?
 * �? **/
                OpenSuction();
                vTaskResume(Vision_TaskHandle);
                osDelay(34);
                /** 发一个信号给摄像�? **/
                vision_cmd = 0x01;
                HAL_UART_Transmit(&huart2,&vision_cmd, sizeof(vision_cmd),0xFFFFF);
                /** 挂起自己 **/
                vTaskSuspend(Start_TaskHandle);
            }
        }
        else
        {
            Chassis_Move_OfVision(&Start_Points[index]);
        }
    osDelay(5);
  }
  /* USER CODE END StartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

