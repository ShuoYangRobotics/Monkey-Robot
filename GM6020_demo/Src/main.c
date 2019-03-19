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
#include "arm_math.h"

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
//extern UART_HandleTypeDef huart2;
extern imu_t              imu;
char buf[300];
int count;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t led_cnt;

// feedback information of motors and system
extern moto_info_t motor_info[2];
float motor_angle_rad[2];
float motor_velocity_rads[2];

// target control actions, be very careful about the rotation direction of the motors
float target_angle_rad[2];
float target_velocity_rads[2];
float target_current[2];
int16_t target_voltage[2];

// motor control PIDs
pid_struct_t motor_angle_pid[2];
pid_struct_t motor_velocity_pid[2];
pid_struct_t motor_current_pid[2];

// TODO: put these in a separate file 
// simple position trajectory, this is used in ctrl_mode 3
// in these trajectories, state is [q, qdot] (angle and angular velocity)
// for this simpel trajectory, it is initialized in the traj.h
float Tf = 2.0f; // use 2 seconds to finish this trajectory
const int step = 200;  // how many points in the trajectory
float left_start_state[2];   // 
float left_end_state[2];     // 
float left_state_angle[step+1];
float left_state_velocity[step+1];

float right_start_state[2];   // 
float right_end_state[2];     // 
float right_state_angle[step+1];
float right_state_velocity[step+1];

void init_simple_trajectory(int motor_idx, float Tf, const int step, float tgt_state[], float state_angle[], float state_velocity[])
{
	float curr_angle, curr_velocity;
	float dt = Tf/step;
	if (motor_idx == 0) // left
	{
		curr_angle = motor_angle_rad[0];
		curr_velocity = motor_velocity_rads[0];
	}
	else // right
	{
		curr_angle = motor_angle_rad[1];
		curr_velocity = motor_velocity_rads[1];
	}
	state_angle[0] = curr_angle;
	state_velocity[0] = curr_velocity;
	for (int i = 1; i < step; i++)
	{
		state_angle[i] = state_angle[0] + (float)i/step*(tgt_state[0] - state_angle[0]);
		
		state_velocity[i] = 0.6f*(state_angle[i] - state_angle[i-1])/dt + 0.4f*state_velocity[i-1];
	}
	state_angle[step] = tgt_state[0];
	state_velocity[step] = tgt_state[1];
}	


/// mode selection flags
/// mode selection flags
/// mode selection flags
/// mode selection flags
int debug_print = 0; // if debug print = 1, print imu info to UART, if debug print = 2 print motor info to UART, if debug print = 3 print control loop info to UART
int ctrl_mode = 0 ;  // if ctrl_mode = 0, use target position for control    // if ctrl_mode = 1 use target velocity // if ctrl_mode = 2 direct control voltage
int output_enable = 0; // if output_enable == 0, do not output control voltage to motors
/// mode selection flags
/// mode selection flags
/// mode selection flags
/// mode selection flags

uint16_t pwm_pulse_left = 1500;  // default pwm pulse width:1080~1920
uint16_t pwm_pulse_right = 1500;  // default pwm pulse width:1080~1920
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[] = "*********SENDING DATA USING USART1 with DMA***********\r\n";

// serial communication protocol
struct serial_struct {
  uint8_t flag;            // AA    0b10101010
	uint8_t type;            // 2 bytes
	uint16_t value;          // 4 bytes
	float position;          // 8 bytes
	float velocity;          // 8 bytes
  uint16_t crc;						 // 4 bytes
	  
} __attribute__((packed));  
// sizeof(serial_struct)  // 28 bytes


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//bluetooth message buffer
uint8_t aTxStartMessages[] = "\r\n******UART commucition using IT******\r\nThis is a demo!\r\n";
uint8_t aRxBuffer[] = "    is signal received\n";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
	
	mpu_device_init();
	init_quaternion();	
	//Bluetooth setup
//	HAL_UART_Transmit(&huart2 ,(uint8_t*)aTxStartMessages,sizeof(aTxStartMessages),55); 
//	HAL_UART_Receive_IT(&huart2,(uint8_t*)aRxBuffer,3); //will receive exactly 3 characters. until the receive is done, then trigger the callback function
	HAL_UART_Receive_DMA(&huart2,aRxBuffer,3);
  HAL_UART_Transmit_DMA(&huart2,aTxBuffer,sizeof(aTxBuffer));
	
	//either listen to imu or can for imu/can readings
	
	//motor setup
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  pwm_init();                              // start pwm output
  can_user_init(&hcan1);                   // config can filter, start can

	// PID set up
  pid_init(&motor_angle_pid[0], 268, 0.00001, 0.0001, 200, 700);       //init pid parameter, kp=38, ki=0.001, kd=0.5, output limit = 200rads
  pid_init(&motor_angle_pid[1], 268, 0.00001, 0.0001, 200, 700);       //init pid parameter, kp=38, ki=0.001, kd=0.5, output limit = 200rads
  pid_init(&motor_velocity_pid[0], 6, 0.00001, 0.06, 125, 400); //init pid parameter, kp=7, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_velocity_pid[1], 6, 0.00001, 0.06, 125, 400); //init pid parameter, kp=7, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_current_pid[0], 160, 0.001, 0.06, 20000, 30000); //init pid parameter, kp=1000, ki=3, kd=0.06, output limit = 30000
  pid_init(&motor_current_pid[1], 160, 0.001, 0.06, 20000, 30000); //init pid parameter, kp=1000, ki=3, kd=0.06, output limit = 30000
	

	target_angle_rad[0] = PI;
	target_angle_rad[1] = PI;
	target_velocity_rads[0] = 0;
	target_velocity_rads[1] = 0;
	target_current[0] = 0;
	target_current[1] = 0;
	
	// init simple trajectory
	float tgt_state[2] = {PI,0};
	float tgt_velocity[2];
	float dt = 0.001f; // the time between each loop run
	float traj_timer = 0.0f;
	bool traj_start = 0;
	int traj_count = 0;
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		// start control loop
		motor_angle_rad[0] = motor_info[0].rotor_angle/8192.0f*2*PI;
		motor_angle_rad[1] = motor_info[1].rotor_angle/8192.0f*2*PI;
		motor_velocity_rads[0] = motor_info[0].rotor_speed*PI/30.0f;
		motor_velocity_rads[1] = motor_info[1].rotor_speed*PI/30.0f;
		
		
		if (ctrl_mode == 0)
		{
			/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
			target_velocity_rads[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], motor_angle_rad[0]);
			motor_info[0].set_voltage = pid_calc(&motor_velocity_pid[0], target_velocity_rads[0], motor_velocity_rads[0]);
			/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
			target_velocity_rads[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], motor_angle_rad[1]);
			motor_info[1].set_voltage = pid_calc(&motor_velocity_pid[1], target_velocity_rads[1], motor_velocity_rads[1]);

			/* send motor control message through can bus*/
			if (output_enable == 1)
			{		
				set_motor_voltage(0, 
					motor_info[0].set_voltage, 
					motor_info[1].set_voltage);
			}
			traj_start = 0;
		}
		else if (ctrl_mode == 1)
		{
			/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
			motor_info[0].set_voltage = pid_calc(&motor_velocity_pid[0], target_velocity_rads[0], motor_velocity_rads[0]);
			/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
			motor_info[1].set_voltage = pid_calc(&motor_velocity_pid[1], target_velocity_rads[1], motor_velocity_rads[1]);

			/* send motor control message through can bus*/
			if (output_enable == 1)
			{		
				set_motor_voltage(0, 
					motor_info[0].set_voltage, 
					motor_info[1].set_voltage);
			}
		}
		else if (ctrl_mode == 2)
		{
			motor_info[0].set_voltage = target_voltage[0];
			motor_info[1].set_voltage = target_voltage[1];
			
			/* send motor control message through can bus*/
			if (output_enable == 1)
			{		
				set_motor_voltage(0, 
					motor_info[0].set_voltage, 
					motor_info[1].set_voltage);
			}
		}
		else if (ctrl_mode == 3)
		{
			// first enter, should have traj_start = 0
			if (traj_start == 1)
			{
				// take waypoint from trajectory
				if (traj_timer > traj_count*Tf/step)
				{
					if (traj_count < step)
					{
						traj_count++;
					}
					else
					{
						traj_timer = Tf;
					}
				}
				
				target_angle_rad[0] = left_state_angle[traj_count];
				target_velocity_rads[0] = left_state_velocity[traj_count];
				target_angle_rad[1] = right_state_angle[traj_count];
				target_velocity_rads[1] = right_state_velocity[traj_count];
				
				/* motor speed pid calc ID1 ID1 ID1 ID1 ID1 ID1 ID1 ID1*/
				tgt_velocity[0] = pid_calc(&motor_angle_pid[0], target_angle_rad[0], motor_angle_rad[0]);
				target_current[0] = pid_calc(&motor_velocity_pid[0], tgt_velocity[0], motor_velocity_rads[0]);
				motor_info[0].set_voltage = pid_calc(&motor_current_pid[0], target_current[0], motor_info[0].torque_current/5700.0f);
				
				
				/* motor speed pid calc ID2 ID2 ID2 ID2 ID2 ID2 ID2 ID2*/
				tgt_velocity[1] = pid_calc(&motor_angle_pid[1], target_angle_rad[1], motor_angle_rad[1]);
				target_current[1] = pid_calc(&motor_velocity_pid[1], tgt_velocity[1], motor_velocity_rads[1]);
				motor_info[1].set_voltage = pid_calc(&motor_current_pid[1], target_current[1], motor_info[1].torque_current/5700.0f);

				/* send motor control message through can bus*/
				if (output_enable == 1)
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
				init_simple_trajectory(0, Tf, step, tgt_state, left_state_angle, left_state_velocity);
				init_simple_trajectory(1, Tf, step, tgt_state, right_state_angle, right_state_velocity);
			}
				
		}
		// end control loop

		/* led blink and debug */
    led_cnt ++;
    if (led_cnt == 150)
    {	  
			if (debug_print == 1) {
//				HAL_Delay(5);		
				sprintf(buf, "ctrl_mode %d \t ax: %d \t ay: %d \t az: %d\n", ctrl_mode, imu.ax, imu.ay, imu.az);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1)); 
//				HAL_Delay(5);	
//				memset(buf, 0, sizeof(buf));				
			}
			else if (debug_print == 2) {
//				HAL_Delay(5);		
				sprintf(buf, "ctrl_mode %d \t angle1:%4.3f \t angle2:%4.3f \t vel1:%4.3f \t vel2:%4.3f crt1:%d \t crt2:%d \n", 
					ctrl_mode, motor_angle_rad[0], motor_angle_rad[1], motor_velocity_rads[0], motor_velocity_rads[1], motor_info[0].torque_current,motor_info[1].torque_current);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1));
//				HAL_Delay(5);		
//				memset(buf, 0, sizeof(buf));	
			}
			else if (debug_print == 3) {
//				HAL_Delay(5);		
				sprintf(buf, "ctrl_mode %d \t tgt_angle1:%4.3f \t tgt_angle2:%4.3f \t tgt_vel1:%4.3f \t tgt_vel2:%4.3f \t set_voltage1:%d \t set_voltage2:%d \n", 
					ctrl_mode, target_angle_rad[0], target_angle_rad[1], target_velocity_rads[0], target_velocity_rads[1], motor_info[0].set_voltage, motor_info[1].set_voltage);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1));
//				HAL_Delay(5);		
//				memset(buf, 0, sizeof(buf));	
			}
			else if (debug_print == 4) {
				sprintf(buf, "ctrl_mode %d \t trajcount %d \t angle1:%4.3f \t angle2:%4.3f mvolt1:%d \t mvolt2:%d crt1:%4.3f \t crt2:%4.3f \n", ctrl_mode, traj_count, 
					left_state_angle[traj_count], right_state_angle[traj_count], motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[0].torque_current/5700.0f,motor_info[1].torque_current/5700.0f);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (COUNTOF(buf)-1));
			}

      led_cnt = 0;
      LED_RED_TOGGLE(); //blink cycle 500ms
    }
		
    /* system delay 1ms */
    HAL_Delay(0);
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
void Start_Moving(void){
	//toggle some flag so that in main loop it starts moving
}

void IMU_read(void) //send imu data back through UART connection
{

	HAL_Delay(5);		
	sprintf(buf, " Roll: %8.3lf    Pitch: %8.3lf    Yaw: %8.3lf\n", imu.rol, imu.pit, imu.yaw);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf, (sizeof(buf) / sizeof(*(buf)))-1, 55); //transmit data
	HAL_Delay(5);	
}

void CAN_read(void)
{
	 for (uint8_t i = 0; i < 2; i++)
    {
			HAL_Delay(5);
			sprintf(buf, " CAN ID: %d  Rotor Angle: %d  Rotor Speed: %d  Torque Current: %d\n", motor_info[i].can_id, motor_info[i].rotor_angle, motor_info[i].rotor_speed, motor_info[i].torque_current);
			HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), 55);
			HAL_Delay(5);
    }
		/*
		inside motor info
	  uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
		*/
}

// callback after the DMA transfer complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	HAL_UART_Transmit_DMA(&huart2,aRxBuffer,3);
	HAL_UART_Receive_DMA(&huart2,aRxBuffer,3);
	
	if (strncmp((char*)aRxBuffer, "imu",3) == 0)
	{ 
		// read imu info
		debug_print = 1;
	} 
	else if (strncmp((char*)aRxBuffer, "mtr",3) == 0) 
	{ 
		// read motor info
		debug_print = 2;
	} 
	else if (strncmp((char*)aRxBuffer, "mov",3) == 0) 
	{
		//START TO MOVE!!!!!!!!!!!!!!!!!!!!!!!!!
		ctrl_mode = 3;
		output_enable = 1;
		debug_print = 4;
	} 
	else if (strncmp((char*)aRxBuffer, "ctl",3) == 0) 
	{
		// read control loop debug info
		debug_print = 3;
	}
	else if (strncmp((char*)aRxBuffer, "stp",3) == 0) 
	{
		
		ctrl_mode = 0;
		output_enable = 0;
		debug_print = 0;
	}
	//HAL_UART_Transmit(&huart2,(uint8_t*)aRxBuffer,1,55);//(uint8_t*)aRxBuffer??????,10??????,0xFFFF?????
	//HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	//HAL_UART_Receive_IT(&huart2,(uint8_t*)aRxBuffer,3);
}

// callback after the data string sent complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//HAL_UART_Transmit_DMA(&huart2,aTxBuffer,sizeof(aTxBuffer));
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
