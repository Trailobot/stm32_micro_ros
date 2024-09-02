/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include "geometry_msgs/msg/twist.h"
#include <geometry_msgs/msg/vector3.h>

#include "usart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "bno055_stm32.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPR 800
#define TIM_FREQ 1000000
#define ALPHA (2*3.14159/SPR)
#define PI 3.14159
#define wheel_radius 0.1
//#define wheel_separation 0.5
#define ARRAY_LEN 200

#define gear_ratio 10
#define wheel_separation 0.50
#define wheel_radius 0.1

#define p_imu 1.0  // 0%

uint8_t state_reconnect;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float imu_yaw;
//extern float x_pos,y_pos, yaw_pos;
float x_pos,y_pos, yaw_pos;
float z_yaw;
//extern float imu_yaw;
int32_t diff1,diff2;
uint8_t pub_state;

extern uint16_t step_count_right;
extern uint16_t step_count_left;

uint16_t last_step_count_right = 0;
uint16_t last_step_count_left = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float angular_velocity_left_con = 0.0;
float angular_velocity_right_con = 0.0;

float angular_velocity_left = 0.0;
float angular_velocity_right = 0.0;

int direction_right;
int direction_left;

extern float wheel_angular_velocity_right;
extern float wheel_angular_velocity_left;

rcl_ret_t rc;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

geometry_msgs__msg__Vector3 vector3_subscribe;
geometry_msgs__msg__Vector3 vector3_publish;
geometry_msgs__msg__Twist twist_msg;

/* USER CODE END Variables */
/* Definitions for microros */
osThreadId_t microrosHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t microros_attributes = {
  .name = "microros",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

float pi_to_pi(float angle) {
    // Use fmod to wrap the angle within the range [-π, π)
    angle = fmod(angle + M_PI, 2 * M_PI);

    if (angle < 0) {
        angle += 2 * M_PI;
    }

    return angle - M_PI;
}


bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;

	  angular_velocity_left_con = (twist_msg->linear.x -  (twist_msg->angular.z * wheel_separation) / 2.0) / wheel_radius;
	  angular_velocity_right_con = (twist_msg->linear.x +  (twist_msg->angular.z * wheel_separation) / 2.0) / wheel_radius;

	// Direction of stepper motor left
	if (angular_velocity_left_con >= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
		direction_left = 1;
		angular_velocity_left = gear_ratio * angular_velocity_left_con;// have value 10 * value

	}else if (angular_velocity_left_con < 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
		direction_left = 0;
		angular_velocity_left = -1.0 * gear_ratio * angular_velocity_left_con;
	}

	// Direction of stepper motor right
	if (angular_velocity_right_con >= 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		direction_right = 1;
		angular_velocity_right = gear_ratio * angular_velocity_right_con;

	}else if (angular_velocity_right_con < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		direction_right = 0;
		angular_velocity_right = -1.0 * gear_ratio * angular_velocity_right_con;
	}

}

void twist_publish_callback(rcl_timer_t * timer, int64_t last_call_time){
	(void) last_call_time;
	if (timer != NULL){


//		  if(rc != RCL_RET_OK){
//			  state_reconnect = 1;
//			  NVIC_SystemReset();
//		  }
		// Update the robot position and orientation based on the wheel movements:
		  diff1 = step_count_right - last_step_count_right;
		  diff2 = step_count_left - last_step_count_left;

		  if (diff1 > 32768){
			  diff1 = diff1 - 65535;
		  }
		  else if(diff1 < -32768){
			  diff1 = diff1 + 65535;
		  }
		  if (diff2 > 32768){
			  diff2 = diff2 - 65535;
		  }
		  else if(diff2 < -32768){
			  diff2 = diff2 + 65535;
		  }

		  // calculation

		  wheel_angular_velocity_right = 2*PI * (diff1) / (2*SPR * 10);
		  wheel_angular_velocity_left = 2*PI * (diff2)  / (2*SPR * 10);

		  z_yaw = z_yaw + (wheel_angular_velocity_right - wheel_angular_velocity_left) * wheel_radius / wheel_separation;

		  yaw_pos =  p_imu * pi_to_pi(imu_yaw) + (1 - p_imu) * pi_to_pi(z_yaw);

		  x_pos = x_pos + cos(yaw_pos) * ((wheel_angular_velocity_right + wheel_angular_velocity_left) * wheel_radius / 2);
		  y_pos = y_pos + sin(yaw_pos) * ((wheel_angular_velocity_right + wheel_angular_velocity_left) * wheel_radius / 2);


		  last_step_count_right = step_count_right;
		  last_step_count_left = step_count_left;

		  pub_state ++;
		  if (pub_state > 1){
			  pub_state = 0;
				vector3_publish.x = x_pos;
				vector3_publish.y = y_pos;
				vector3_publish.z = yaw_pos;
				rc = rcl_publish(&publisher, &vector3_publish, NULL);
		  }


	}

}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void imu_task_fn(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
//  Twist_queueHandle = osMessageQueueNew (16, 24, &Twist_queue_attributes);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of microros */
  microrosHandle = osThreadNew(StartDefaultTask, NULL, &microros_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(imu_task_fn, NULL, &imu_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	  // micro-ROS configuration
	  // micro-ROS configuration
	  char test_array[ARRAY_LEN];
	  memset(test_array, 'z', ARRAY_LEN);

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate = microros_zero_allocate;



	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	    printf("Error: Failed to set default allocators.\n");
	    while (1); // Halt execution if allocator setup fails
	  }

	  // Initialize micro-ROS allocator
	  rcl_allocator_t allocator = rcl_get_default_allocator();


	  // Initialize support object// Initialize and modify options (Set DOMAIN ID to 10)
	  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	  rcl_init_options_init(&init_options, allocator);
	  rcl_init_options_set_domain_id(&init_options, 21);	// 21

	  rclc_support_t support;
	  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		  state_reconnect = 1;
	    printf("Error: Failed to initialize rclc_support.\n");
	    while (1); // Halt execution if support initialization fails
	  }


	  // Create node object
	  rcl_node_t node;
	  if (rclc_node_init_default(&node, "stm32f446re_node", "", &support) != RCL_RET_OK) {
		  state_reconnect = 2;
	    printf("Error: Failed to initialize rclc_node.\n");
	    while (1); // Halt execution if node initialization fails
	  }

	  // Create publisher
	  const char *pub_topic_name = "/position_uros";
	  const rosidl_message_type_support_t *pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
	  if (rclc_publisher_init_default(&publisher, &node, pub_type_support, pub_topic_name) != RCL_RET_OK) {
		  state_reconnect = 3;
	    printf("Error: Failed to initialize publisher.\n");
	    while (1); // Halt execution if publisher initialization fails
	  }

	  // Create timer
	  rcl_timer_t timer;
	  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), twist_publish_callback) != RCL_RET_OK) {
		  state_reconnect = 4;
	    printf("Error: Failed to initialize timer.\n");
	    while (1); // Halt execution if timer initialization fails
	  }

	  // Create subscriber
	  const char *sub_topic_name = "/cmd_vel_out";
	  const rosidl_message_type_support_t *sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
	  if (rclc_subscription_init_default(&subscriber, &node, sub_type_support, sub_topic_name) != RCL_RET_OK) {
		  state_reconnect = 5;
	    printf("Error: Failed to initialize subscription.\n");
	    while (1); // Halt execution if subscription initialization fails
	  }

	  // Create executor
	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
		  state_reconnect = 6;
	    printf("Error: Failed to initialize executor.\n");
	    while (1); // Halt execution if executor initialization fails
	  }

	  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
		  state_reconnect = 7;
	    printf("Error: Failed to add timer to executor.\n");
	    while (1); // Halt execution if adding timer to executor fails
	  }

	  if (rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA) != RCL_RET_OK) {
		  state_reconnect = 8;
	    printf("Error: Failed to add subscription to executor.\n");
	    while (1); // Halt execution if adding subscription to executor fails
	  }

	  // Spin executor to receive messages
	  rclc_executor_prepare(&executor);
	  rclc_executor_spin(&executor);

	  // cleaning Up
	  rc += rcl_subscription_fini(&subscriber, &node);
	  rc += rcl_node_fini(&node);

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_imu_task_fn */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task_fn */
void imu_task_fn(void *argument)
{
  /* USER CODE BEGIN imu_task_fn */
	  bno055_assignI2C(&hi2c1);
	  bno055_setup();
	  bno055_setOperationModeNDOF();
  /* Infinite loop */
  for(;;)
  {
	  if(rc != RCL_RET_OK){

		  NVIC_SystemReset();
	  }
	  if (state_reconnect != 0){
		  NVIC_SystemReset();
	  }
	  bno055_vector_t v = bno055_getVectorEuler();
	  imu_yaw = -1.0 * v.x * M_PI /180;
    osDelay(20);
  }
  /* USER CODE END imu_task_fn */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

