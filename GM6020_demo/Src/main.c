/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "pid.h"
#include "bsp_imu.h"
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

/* USER CODE BEGIN PV */

//extern UART_HandleTypeDef huart2;
extern imu_t imu;
char buf[300];

// feedback information of motors and system
extern moto_info_t motor_info[2];
float motor_angle_rad[2] = {0, 0};
float prev_motor_angle_rad[2] = {0, 0};  // because motor_angle_rad is only in 0- 2PI, so in order to 
																// use it to control the system properly,  have to detect change
float modified_motor_angle_rad[2] = {0, 0};  
float motor_velocity_rads[2] = {0, 0};

// target control actions, be very careful about the rotation direction of the motors
float target_angle_rad[2] = {0, 0};
float target_velocity_rads[2] = {0, 0};
float target_current[2] = {0, 0};
int16_t target_voltage[2] = {0, 0};

// motor control PIDs
pid_struct_t motor_angle_pid[2];
pid_struct_t motor_velocity_pid[2];
pid_struct_t motor_current_pid[2];

// TODO: put these in a separate file 
// simple position trajectory, this is used in ctrl_mode 3
// in these trajectories, state is [q, qdot] (angle and angular velocity)
// for this simpel trajectory, it is initialized in the traj.h
float Tf = 0.66f; // use 2 seconds to finish this trajectory

float aux_traj_Tf = 3.0f;  // the time to execute init pose auxiluary trajectory

float left_start_state[2];   // 
float left_end_state[2];     // 
float left_state_angle[stepNum+1];
float left_state_velocity[stepNum+1];

float right_start_state[2];   // 
float right_end_state[2];     // 
float right_state_angle[stepNum+1];
float right_state_velocity[stepNum+1];

//// record data pool for debug
float left_real_pos[301];
float right_real_pos[301];
float left_real_vel[301];
float right_real_vel[301];
bool acked  = true;
uint16_t packIndex = 0;
//// 

extern Trajectory traj;
extern Trajectory aux_traj;

// init simple trajectory
float tgt_state[2] = {0,0};
float tgt_velocity[2];
float dt = 0.001f; // the time between each loop run
float traj_timer = 0.0f;
uint16_t traj_start = 0;
int traj_count = 0;
	
/// mode selection flags
/// mode selection flags
RobotControl robot_control = { 	.debug_print = 0, .ctrl_mode = 0, .output_enable = 0, 
																.pwm_pulse_left = 1500, .pwm_pulse_right = 1500, .acked = 1 };
/// mode selection flags
/// mode selection flags

uint16_t count = 0;
//uint16_t pwm_pulse_left = 1500;  // default pwm pulse width:1080~1920
//uint16_t pwm_pulse_right = 1500;  // default pwm pulse width:1080~1920
/* Private variables ---------------------------------------------------------*/
//bluetooth message buffer
uint8_t aTxStartMessage[] = "\r\n******Init Done. Program start to receive command.******\r\n";
uint8_t aRxBuffer[RxBufferSize];
uint8_t *head = aRxBuffer;
uint8_t *tail = aRxBuffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_simple_trajectory(int motor_idx, float T_curr, float tgt_state[], 
														float state_angle[], float state_velocity[])
{
	float curr_angle, curr_velocity;
	float dt = T_curr/stepNum;
	if (motor_idx == 0) // left
	{
		curr_angle = modified_motor_angle_rad[0];
		curr_velocity = motor_velocity_rads[0];
	}
	else // right
	{
		curr_angle = modified_motor_angle_rad[1];
		curr_velocity = motor_velocity_rads[1];
	}
	state_angle[0] = curr_angle;
	state_velocity[0] = curr_velocity;
	for (int i = 1; i < stepNum; i++)
	{
		state_angle[i] = state_angle[0] + (float)i/stepNum*(tgt_state[0] - state_angle[0]);
		
		state_velocity[i] = 0.6f*(state_angle[i] - state_angle[i-1])/dt + 0.4f*state_velocity[i-1];
	}
	state_angle[stepNum] = tgt_state[0];
	state_velocity[stepNum] = tgt_state[1];
}

void init_test_swing_trajectory(int motor_idx, float Tf, float state_angle[], float state_velocity[])
{
	float curr_angle, curr_velocity, tgt_angle, tgt_velocity;
	float dt = Tf/stepNum;
	if (motor_idx == 0) // left
	{
		curr_angle = modified_motor_angle_rad[0];
		tgt_angle = modified_motor_angle_rad[0];
		curr_velocity = 0.0f;
		tgt_velocity = 0.0f;
	}
	else // right
	{
		curr_angle = -90.0f/180.0f*PI;
		tgt_angle = -(90.0f+90.0f+90.0f)/180.0f*PI;
		curr_velocity = 0.0f;
		tgt_velocity = 0.0f;
	}
	state_angle[0] = curr_angle;
	state_velocity[0] = curr_velocity;
	for (int i = 1; i < stepNum; i++)
	{
		state_angle[i] = state_angle[0] + (float)i/stepNum*(tgt_angle - state_angle[0]);
		
		state_velocity[i] = 0.6f*(state_angle[i] - state_angle[i-1])/dt + 0.4f*state_velocity[i-1];
	}
	state_angle[stepNum] = tgt_angle;
	state_velocity[stepNum] = tgt_velocity;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_SPI5_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	// tim2 enable interupt
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	// imu setup
	mpu_device_init();
	init_quaternion();	

	// motor setup
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  pwm_init();                              // start pwm output
  can_user_init(&hcan1);                   // config can filter, start can

	// PID setup
  pid_init(&motor_angle_pid[0], 380, 0.0001, 0.01, 200, 800);       //init pid parameter, kp=38, ki=0.001, kd=0.5, output limit = 200rads
  pid_init(&motor_angle_pid[1], 380, 0.0001, 0.01, 200, 800);       //init pid parameter, kp=38, ki=0.001, kd=0.5, output limit = 200rads
  pid_init(&motor_velocity_pid[0], 8, 0.00001, 0.06, 125, 700); //init pid parameter, kp=7, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_velocity_pid[1], 8, 0.00001, 0.06, 125, 700); //init pid parameter, kp=7, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_current_pid[0], 160, 0.001, 0.06, 20000, 30000); //init pid parameter, kp=1000, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_current_pid[1], 160, 0.001, 0.06, 20000, 30000); //init pid parameter, kp=1000, ki=3, kd=0.06, output limit = 30000
	
	// end of setup, set uart to send start message and start listening
	HAL_UART_Receive_DMA(&huart2,aRxBuffer,14);
  HAL_UART_Transmit_DMA(&huart2,aTxStartMessage,sizeof(aTxStartMessage));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		// start control loop
		
		// 2019-04-03 test pwm
    /* set pwm pulse width */
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, robot_control.pwm_pulse_left);   // PA8
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_pulse_right);
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_pulse_left);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, robot_control.pwm_pulse_right); //PE14

//		count++;
//		if (count == 500)
//		{
//			LED_RED_TOGGLE();
////			pwm_pulse_left += 380;
////			pwm_pulse_right -=380;
//			count = 0;
//		}
//		if(robot_control.debug_print == 4) {
//			int i;
//			for(i=0; i<300; i++) {
//				Serial_struct dataPack = pack(50, 0, (int32_t)(left_real_pos[i]*1000), (int32_t)(left_real_vel[i]*1000));
//				HAL_UART_Transmit(&huart2, (uint8_t*)&dataPack, sizeof(dataPack),100);
//				setLastSend(dataPack); acked = false;
//				while(!acked);
//				//LED_RED_TOGGLE();	
//				
//				dataPack = pack(50, 1, (int32_t)(right_real_pos[i]*1000), (int32_t)(right_real_vel[i]*1000));
//				HAL_UART_Transmit(&huart2, (uint8_t*)&dataPack, sizeof(dataPack),100);
//				setLastSend(dataPack); acked = false;
//				while(!acked);
//				
//				//LED_RED_TOGGLE();	
//			}
//			Serial_struct finishPack = pack(52, 0, 0, 0);
//			HAL_UART_Transmit(&huart2, (uint8_t*)&finishPack, sizeof(finishPack),100);
//			robot_control.debug_print = 0;
//		}
//		if (pwm_pulse_left > 1881)
//		{
//			pwm_pulse_left = 1500;
//			pwm_pulse_right = 1500;
//		}
		// end control loop
		
    /* system delay 1ms */
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {
		
		switch (robot_control.debug_print) {
			case 0 : // print nothing
				break;
			
			case 1:
				sprintf(buf, "ctrl_mode %d \t ax: %d \t ay: %d \t az: %d\n", robot_control.ctrl_mode, imu.ax, imu.ay, imu.az);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1)); 
				//				memset(buf, 0, sizeof(buf));
				break;
			
			case 2:
				sprintf(buf, "ctrl_mode %d \t angle1:%4.3f \t angle2:%4.3f \t vel1:%4.3f \t vel2:%4.3f crt1:%d \t crt2:%d \n", 
					robot_control.ctrl_mode, modified_motor_angle_rad[0], modified_motor_angle_rad[1], motor_velocity_rads[0], motor_velocity_rads[1], motor_info[0].torque_current,motor_info[1].torque_current);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1));
				//				memset(buf, 0, sizeof(buf));
				break;
			
			case 3:
				sprintf(buf, "ctrl_mode %d \t tgt_angle1:%4.3f \t tgt_angle2:%4.3f \t tgt_vel1:%4.3f \t tgt_vel2:%4.3f \t set_voltage1:%d \t set_voltage2:%d \n", 
					robot_control.ctrl_mode, target_angle_rad[0], target_angle_rad[1], target_velocity_rads[0], target_velocity_rads[1], motor_info[0].set_voltage, motor_info[1].set_voltage);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1));
				//				memset(buf, 0, sizeof(buf));	
				break;
			
			case 4: { // send out data in every 1ms
				if(robot_control.acked) {
					if(packIndex<300){
						Serial_struct dataPack = pack(50, 0, (int32_t)(left_real_pos[packIndex]*1000), (int32_t)(left_real_vel[packIndex]*1000));
						HAL_UART_Transmit(&huart2, (uint8_t*)&dataPack, sizeof(dataPack), 100);
						
					}else if(packIndex<600) {
						Serial_struct dataPack = pack(50, 1, (int32_t)(right_real_pos[packIndex-300]*1000), (int32_t)(right_real_vel[packIndex-300]*1000));
						HAL_UART_Transmit(&huart2, (uint8_t*)&dataPack, sizeof(dataPack), 100);
						
					}else if(packIndex >= 600) {
						Serial_struct finishPack = pack(52, 0, 0, 0);
						HAL_UART_Transmit(&huart2, (uint8_t*)&finishPack, sizeof(finishPack), 100);
						robot_control.debug_print = 0;
						packIndex = 0;
					}
					robot_control.acked = 0; // set zero to wait for ack
					LED_RED_TOGGLE();
				}
				break;
			}
			default:
				; // to remove warning
				Serial_struct data = {0xAA, 1, 6377, 1, 1};

				// do crc
				uint16_t crc_ccitt_ffff_val = 0xffff;
				uint8_t* ptr = (uint8_t *) &data;
				int i;
				for(i = 0; i<12; i++) { // do crc with the first 12 uint8
					crc_ccitt_ffff_val = update_crc_ccitt(crc_ccitt_ffff_val, *ptr);
					ptr++;
				}
				
				data.crc = crc_ccitt_ffff_val;
				
				//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&data, sizeof(data));
				break;
		}
	}
	
	if (htim->Instance == htim3.Instance) { // 1ms trig for control
		
		
		
		
		
		switch (robot_control.ctrl_mode) {
			case 0 :
				break;
			case 1 : {
				/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
				target_velocity_rads[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], motor_angle_rad[0]);
				motor_info[0].set_voltage = pid_calc(&motor_velocity_pid[0], target_velocity_rads[0], motor_velocity_rads[0]);
				/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
				target_velocity_rads[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], motor_angle_rad[1]);
				motor_info[1].set_voltage = pid_calc(&motor_velocity_pid[1], target_velocity_rads[1], motor_velocity_rads[1]);

				/* send motor control message through can bus*/
				if (robot_control.output_enable == 1)
				{		
					set_motor_voltage(0, 
						motor_info[0].set_voltage, 
						motor_info[1].set_voltage);
				}
				traj_start = 0;
				break;
			}
			case 2 : {
				/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
				motor_info[0].set_voltage = pid_calc(&motor_velocity_pid[0], target_velocity_rads[0], motor_velocity_rads[0]);
				/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
				motor_info[1].set_voltage = pid_calc(&motor_velocity_pid[1], target_velocity_rads[1], motor_velocity_rads[1]);

				/* send motor control message through can bus*/
				if (robot_control.output_enable == 1)
				{		
					set_motor_voltage(0, 
						motor_info[0].set_voltage, 
						motor_info[1].set_voltage);
				}
				break;
			}
			case 3 : {
				motor_info[0].set_voltage = target_voltage[0];
				motor_info[1].set_voltage = target_voltage[1];
				
				/* send motor control message through can bus*/
				if (robot_control.output_enable == 1)
				{		
					set_motor_voltage(0, 
						motor_info[0].set_voltage, 
						motor_info[1].set_voltage);
				}
				break;
			}
			case 4 : {
				// first enter, should have traj_start = 0
				if (traj_start >= 60)  // a 60ms delay
				{
					// take waypoint from trajectory
					if (traj_timer > traj_count*Tf/stepNum)
					{
						if (traj_count < stepNum)
						{
							traj_count++;
						}
						else
						{
							traj_timer = Tf;
							// after finish case 4, go to ctrl mode 6, standby 
							robot_control.ctrl_mode = 6;		
							//robot_control.debug_print = 4; // this moved to after receiving data pack type 49
							// start to send traj data to up
							traj_start = 0;
						}
					}
							
					target_angle_rad[0] = traj.left_state_angle[traj_count];
					target_velocity_rads[0] = traj.left_state_velocity[traj_count];
					target_angle_rad[1] = traj.right_state_angle[traj_count];
					target_velocity_rads[1] = traj.right_state_velocity[traj_count];
					
					/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
					tgt_velocity[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], modified_motor_angle_rad[0]);
					target_current[0] = pid_calc(&motor_velocity_pid[0], tgt_velocity[0]+target_velocity_rads[0], motor_velocity_rads[0]);
					motor_info[0].set_voltage = pid_calc(&motor_current_pid[0], target_current[0], motor_info[0].torque_current/5700.0f);
					// record left
					left_real_pos[traj_count] = modified_motor_angle_rad[0];
					left_real_vel[traj_count] = motor_velocity_rads[0];
					
					/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
					tgt_velocity[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], modified_motor_angle_rad[1]);
					target_current[1] = pid_calc(&motor_velocity_pid[1], tgt_velocity[1]+target_velocity_rads[1], motor_velocity_rads[1]);
					motor_info[1].set_voltage = pid_calc(&motor_current_pid[1], target_current[1], motor_info[1].torque_current/5700.0f);
					// record right
					right_real_pos[traj_count] = modified_motor_angle_rad[1];
					right_real_vel[traj_count] = motor_velocity_rads[1];
					
					/* send motor control message through can bus*/
					if (robot_control.output_enable == 1)
					{			 		
						set_motor_voltage(0, 
							motor_info[0].set_voltage, 
							motor_info[1].set_voltage);
					}
					
					traj_timer += dt;
					
					if (traj_timer > Tf*0.9f) {
						robot_control.pwm_pulse_right = 1500;
					}
				}
				else
				{
					traj_start++;
					traj_timer = 0.0f;
					traj_count = 0;
//					init_simple_trajectory(0, Tf, tgt_state, left_state_angle, left_state_velocity);
//					init_simple_trajectory(1, Tf, tgt_state, right_state_angle, right_state_velocity);
//					init_test_swing_trajectory(0, Tf, left_state_angle, left_state_velocity);
//					init_test_swing_trajectory(1, Tf, right_state_angle, right_state_velocity);
					
					// open right hand as a test step
					robot_control.pwm_pulse_right = 1500+384;
				}
				break;
			}
			case 8: {
				// 03242019
				// this is a intermediate stage, it is also exexutive trajectory, but this movement is just to prepare robot to execuate case 4
				// here a aux_traj is inited and used to control the robot 
			  // first enter, should have traj_start = 0
				if (traj_start == 1)
				{
					// take waypoint from aux trajectory
					if (traj_timer > traj_count*aux_traj_Tf/stepNum)  //TODO: this time extract out
					{
						if (traj_count < stepNum)
						{
							traj_count++;
						}
						else
						{
								traj_timer = aux_traj_Tf;
								// after finish case 5, go to ctrl mode 6 
								robot_control.ctrl_mode = 6;					
								traj_start = 0;
						}
					}
					
					target_angle_rad[0] = aux_traj.left_state_angle[traj_count];
					target_velocity_rads[0] = aux_traj.left_state_velocity[traj_count];
					target_angle_rad[1] = aux_traj.right_state_angle[traj_count];
					target_velocity_rads[1] = aux_traj.right_state_velocity[traj_count];
					
					/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
					tgt_velocity[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], modified_motor_angle_rad[0]);
					target_current[0] = pid_calc(&motor_velocity_pid[0], tgt_velocity[0], motor_velocity_rads[0]);
					motor_info[0].set_voltage = pid_calc(&motor_current_pid[0], target_current[0], motor_info[0].torque_current/5700.0f);
					
					
					/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
					tgt_velocity[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], modified_motor_angle_rad[1]);
					target_current[1] = pid_calc(&motor_velocity_pid[1], tgt_velocity[1], motor_velocity_rads[1]);
					motor_info[1].set_voltage = pid_calc(&motor_current_pid[1], target_current[1], motor_info[1].torque_current/5700.0f);

					/* send motor control message through can bus*/
					if (robot_control.output_enable == 1)
					{			 		
						set_motor_voltage(0, 
							motor_info[0].set_voltage, 
							motor_info[1].set_voltage);
					}
					
					traj_timer += dt;
				}
				else
				{
					traj_start = 1;
					traj_timer = 0.0f;
					traj_count = 0;
					tgt_state[0] = traj.left_state_angle[0];
					tgt_state[1] = traj.left_state_velocity[0];
					init_simple_trajectory(0, aux_traj_Tf, tgt_state, aux_traj.left_state_angle, aux_traj.left_state_velocity);
					tgt_state[0] = traj.right_state_angle[0];
					tgt_state[1] = traj.right_state_velocity[0];
					init_simple_trajectory(1, aux_traj_Tf, tgt_state, aux_traj.right_state_angle, aux_traj.right_state_velocity);
//					init_test_swing_trajectory(0, Tf, left_state_angle, left_state_velocity);
//					init_test_swing_trajectory(1, Tf, right_state_angle, right_state_velocity);
				}
				break;
			}
			case 6 : {
				// hold the motor with previous stage motor command 
				/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
				tgt_velocity[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], modified_motor_angle_rad[0]);
				target_current[0] = pid_calc(&motor_velocity_pid[0], tgt_velocity[0], motor_velocity_rads[0]);
				motor_info[0].set_voltage = pid_calc(&motor_current_pid[0], target_current[0], motor_info[0].torque_current/5700.0f);
				
				
				/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
				tgt_velocity[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], modified_motor_angle_rad[1]);
				target_current[1] = pid_calc(&motor_velocity_pid[1], tgt_velocity[1], motor_velocity_rads[1]);
				motor_info[1].set_voltage = pid_calc(&motor_current_pid[1], target_current[1], motor_info[1].torque_current/5700.0f);

				/* send motor control message through can bus*/
				if (robot_control.output_enable == 1)
				{			 		
					set_motor_voltage(0, 
						motor_info[0].set_voltage, 
						motor_info[1].set_voltage);
				}
				
				//robot_control.debug_print = 4;
				break;
			}
			case 9 : {
				// hold the motor with previous stage motor command for only 500ms
				/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
				tgt_velocity[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], modified_motor_angle_rad[0]);
				target_current[0] = pid_calc(&motor_velocity_pid[0], tgt_velocity[0], motor_velocity_rads[0]);
				motor_info[0].set_voltage = pid_calc(&motor_current_pid[0], target_current[0], motor_info[0].torque_current/5700.0f);
				
				
				/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
				tgt_velocity[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], modified_motor_angle_rad[1]);
				target_current[1] = pid_calc(&motor_velocity_pid[1], tgt_velocity[1], motor_velocity_rads[1]);
				motor_info[1].set_voltage = pid_calc(&motor_current_pid[1], target_current[1], motor_info[1].torque_current/5700.0f);

				/* send motor control message through can bus*/
				if (robot_control.output_enable == 1)
				{			 		
					set_motor_voltage(0, 
						motor_info[0].set_voltage, 
						motor_info[1].set_voltage);
				}
				traj_count++;
				
				if (traj_count > 500) {
					robot_control.output_enable = 0;
				}
				break;
			}
			default :
				break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
