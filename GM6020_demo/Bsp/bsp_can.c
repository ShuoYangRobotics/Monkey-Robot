/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#include "bsp_can.h"
#include "bsp_led.h"

moto_info_t motor_info[MOTOR_MAX_NUM];
uint16_t can_cnt;
extern float motor_angle_rad[2];
uint8_t motor_rotation_count_inited[2] = {0, 0};  // this only use once
int   rotation_count[2] = {0,0};       // 0 :	0 - 2PI
                                       // -1:  -2PI - 0
																       //  1:  2PI - 4PI
extern float prev_motor_angle_rad[2];  // because motor_angle_rad is only in 0- 2PI, so in order to 
																// use it to control the system properly,  have to detect change

extern float modified_motor_angle_rad[2];  
extern float motor_velocity_rads[2];
/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void can_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
  HAL_CAN_Start(&hcan1);                          // start can1
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		
		/// custom use of can data
		motor_angle_rad[index] = motor_info[index].rotor_angle/8192.0f*2*PI;
		if (motor_rotation_count_inited[index] == 0) // only do this once
		{
			prev_motor_angle_rad[index] = motor_angle_rad[index];
			if (motor_angle_rad[index] > PI)
			{
				rotation_count[index]--;
			}
			
			motor_rotation_count_inited[index] = 1;
		}
		
		if ((motor_angle_rad[index] - prev_motor_angle_rad[index])> 1.5f*PI) //change from 0 to 2PI
			rotation_count[index]--;
		if ((motor_angle_rad[index] - prev_motor_angle_rad[index])< -1.5f*PI) //change from 2PI to 0
			rotation_count[index]++;
		
		modified_motor_angle_rad[index] = motor_angle_rad[index]+rotation_count[index]*2*PI;
		prev_motor_angle_rad[index] = motor_angle_rad[index];
		motor_velocity_rads[index] = motor_info[index].rotor_speed*PI/30.0f;
		/// custom use of can data
  }
  if (can_cnt == 500)
  {
    can_cnt = 0;
    //LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
  }
}

/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}
