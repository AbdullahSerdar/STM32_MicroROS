/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <rcutils/logging_macros.h> // rosout için gerekli kütüphane

#include <std_msgs/msg/empty.h>
#include "geometry_msgs/msg/twist.h"
#include "sensor_msgs/msg/nav_sat_fix.h"
#include <sensor_msgs/msg/imu.h>
#include <rcutils/logging_macros.h>
#include <std_msgs/msg/int32.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t data[2];
uint32_t adc_buffer[2];
float linear_x = 0.0;
float angular_z = 0.0;
//float roll, pitch, yaw;
double UAV_x = 0.0;
double UAV_y = 0.0;
double UGV_x = 0.0;
double UGV_y = 0.0;
double angle_deg = 0.0;
double necc_yaw = 0.0;
double add_cnt_1 = 60.0;
double add_cnt_2 = 5.0;
int correct_yaw = 0;
double distance = 2.0;
double yaw = 0.0;
float global_center_x = 0.0;
float global_center_y = 0.0;
double global_center_axis_x = 0.0;
double global_center_axis_y = 0.0;
double res_axis = 0.0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		data[0] = adc_buffer[0];
		data[1] = adc_buffer[1];
	}
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //while (MPU6050_Init(&hi2c1) == 1);
    HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param Nonepython3 topic_relay.py
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

double calculate_image_point_x(float axis_x)
{
	res_axis = (-((axis_x/320.0) * 0.1) + 0.1) ;
	return res_axis;
}

double calculate_image_point_y(float axis_y)
{
	res_axis = (-((axis_y/180.0) * 0.1) + 0.1) ;
	return res_axis;
}
/*
double calculate_yaw(double UAV_x, double UAV_y, double UGV_x, double UGV_y)
{
	UAV_x += 500.0;
	UAV_y += 500.0;
	UGV_x += 500.0;
	UGV_y += 500.0;
    angle_deg = (atan2(((UGV_y * 100000) - (UAV_y * 100000)), ((UGV_x * 100000) - (UAV_x * 100000)))) * (180.0 / M_PI);
    if (0 < angle_deg && angle_deg <= 90) {
        necc_yaw = -1.6 - ((angle_deg / 90.0) * 1.6);
    } else if (90 < angle_deg && angle_deg <= 180) {
        necc_yaw = 3.2 - ((angle_deg - 90) / 90.0) * 1.6;
    } else if (-90 < angle_deg && angle_deg <= 0) {
        necc_yaw = -1.6 - ((angle_deg / 90.0) * 1.6);
    } else if (-180 < angle_deg && angle_deg <= -90) {
        necc_yaw = -(((angle_deg + 90.0) / 90.0) * 1.6);
    } else {
        necc_yaw = 0.0;
    }
    return necc_yaw;
}
*/

double calculate_angle(double UAV_lat, double UAV_long, double UGV_lat, double UGV_long) {
    double x0 = -UAV_lat;
    double y0 = UAV_long;
    double x1 = -UGV_lat;
    double y1 = UGV_long;
    double delta_x = x1 - x0;
    double delta_y = y1 - y0;
    double angle_rad = atan2(delta_y, delta_x);
    double angle_deg = angle_rad * (180.0 / M_PI);
    angle_deg = angle_deg + 90.0;
    if (angle_deg < 0)
        angle_deg += 360.0;
    else if (angle_deg >= 360.0)
        angle_deg -= 360.0;
    return angle_deg;
}

double calculate_distance(double x1, double y1, double x2, double y2) {
    distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    return distance * 100000;
}
/*
double quaternion_to_yaw(double x, double y, double z, double w) {
	double siny_cosp = 2.0 * (w * z + x * y);
	double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
}
*/
double quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    // Yaw açısını dereceye çevirip 0-360 aralığında tanımlama
    double yaw_degrees = yaw * (180.0 / M_PI);
    if (yaw_degrees < 0) {
        yaw_degrees += 360.0;
    }
    return yaw_degrees;
}

std_msgs__msg__Empty land_msg;
std_msgs__msg__Empty takeoff_msg;
sensor_msgs__msg__Imu imu_data; // İMU verisinin tanımlanması
sensor_msgs__msg__NavSatFix gps_data;  // GPS verisinin tanımlanması
geometry_msgs__msg__Twist cmd_vel_data;  // Motor(odyometri veri yapısı)
sensor_msgs__msg__NavSatFix new_gps_data;  // IMU'nun GPS verisinin tanımlanması
sensor_msgs__msg__Imu new_imu_data; // Yeni IMU verisinin tanımlanması
geometry_msgs__msg__Twist cmd_vel_msg;

void center_coordinates_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Gelen center coordinates verisini global değişkenlere ata
    global_center_x = msg->linear.x;
    global_center_y = msg->linear.y;
}

void send_cmd_vel(float linear_x, float angular_z) {
    cmd_vel_data.linear.x = linear_x;
    cmd_vel_data.linear.y = 0.0;
    cmd_vel_data.linear.z = 0.0;
    cmd_vel_data.angular.x = 0.0;
    cmd_vel_data.angular.y = 0.0;
    cmd_vel_data.angular.z = angular_z;
}

void imu_callback(const void *msgin) {
    const sensor_msgs__msg__Imu *msg = (const sensor_msgs__msg__Imu *)msgin;
    imu_data = *msg;
}

void new_imu_callback(const void *msgin) {
    const sensor_msgs__msg__Imu *msg = (const sensor_msgs__msg__Imu *)msgin;
    new_imu_data = *msg;
}

void new_gps_callback(const void *msgin) {
    const sensor_msgs__msg__NavSatFix *msg = (const sensor_msgs__msg__NavSatFix *)msgin;
    new_gps_data = *msg;
}

void gps_callback(const void *msgin) {
    const sensor_msgs__msg__NavSatFix *msg = (const sensor_msgs__msg__NavSatFix *)msgin;
    gps_data = *msg;
}

void send_drone_cmd_vel(float linear_xd, float linear_yd, float linear_zd, float angular_zd) {
    cmd_vel_msg.linear.x = linear_xd;
    cmd_vel_msg.linear.y = linear_yd;
    cmd_vel_msg.linear.z = linear_zd;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = angular_zd;
}

void StartDefaultTask(void *argument) {
    rmw_uros_set_custom_transport(
        true,
        (void *)&huart2,
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
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    rcl_node_t node;
    rcl_subscription_t imu_subscriber;
    rcl_subscription_t gps_subscriber;
    rcl_subscription_t new_gps_subscriber;
    rcl_subscription_t new_imu_subscriber; // Yeni IMU aboneliği
    rcl_publisher_t cmd_vel_publisher;
    rcl_publisher_t takeoff_publisher;
    rcl_publisher_t land_publisher;
    rcl_publisher_t drone_cmd_vel_publisher;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rclc_executor_t executor;
    rcl_subscription_t center_coordinates_subscriber;

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "cubemx_node", "", &support);

    rclc_subscription_init_default(
        &center_coordinates_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/center_coordinates");

    rclc_subscription_init_default(
        &imu_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/micro_ros_imu");

    rclc_subscription_init_default(
        &gps_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "/micro_ros_gps");

    rclc_subscription_init_default(
        &new_gps_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "/micro_ros_new_gps");

    rclc_subscription_init_default(
        &new_imu_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/micro_ros_new_imu");

    rclc_publisher_init_default(
        &cmd_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    rclc_publisher_init_default(
        &takeoff_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/simple_drone/takeoff");

    rclc_publisher_init_default(
        &land_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/simple_drone/land");

    rclc_publisher_init_default(
        &drone_cmd_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/simple_drone/cmd_vel");

    rclc_executor_init(&executor, &support.context, 5, &allocator);
    rclc_executor_add_subscription(&executor, &imu_subscriber, &imu_data, &imu_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &gps_subscriber, &gps_data, &gps_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &new_gps_subscriber, &new_gps_data, &new_gps_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &new_imu_subscriber, &new_imu_data, &new_imu_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &center_coordinates_subscriber, &cmd_vel_data, &center_coordinates_callback, ON_NEW_DATA);

    for (;;) {

        float float_data0 = (float)data[0];
        float float_data1 = (float)data[1];

        linear_x = (float_data0 - 2030.0f) / 420.0f;
        angular_z = (float_data1 - 2030.0f) / 420.0f;

        send_cmd_vel(linear_x, angular_z);
        rcl_publish(&cmd_vel_publisher, &cmd_vel_data, NULL);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        yaw = quaternion_to_yaw(new_imu_data.orientation.x,
							 new_imu_data.orientation.y,
							 new_imu_data.orientation.z,
							 new_imu_data.orientation.w);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
        {
        	necc_yaw = calculate_angle(new_gps_data.latitude, new_gps_data.longitude, gps_data.latitude, gps_data.longitude);
        	rcl_publish(&takeoff_publisher, &takeoff_msg, NULL);

        	while((yaw < (necc_yaw - add_cnt_2)) || (yaw > (necc_yaw + add_cnt_2)))
        	{
        	   	if((yaw > (necc_yaw - add_cnt_1)) || (yaw < (necc_yaw + add_cnt_1)))
        	   	{
        	   		send_drone_cmd_vel(0.0, 0.0, -0.05, 0.1);
        	   		rcl_publish(&drone_cmd_vel_publisher, &cmd_vel_msg, NULL);
        	   		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        	   		yaw = quaternion_to_yaw(new_imu_data.orientation.x,new_imu_data.orientation.y,new_imu_data.orientation.z,new_imu_data.orientation.w);
        	   	}
        	   	else
        	   	{
        	   		send_drone_cmd_vel(0.0, 0.0, -0.05, 0.05);
        	   		rcl_publish(&drone_cmd_vel_publisher, &cmd_vel_msg, NULL);
        	   		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        	   		yaw = quaternion_to_yaw(new_imu_data.orientation.x,new_imu_data.orientation.y,new_imu_data.orientation.z,new_imu_data.orientation.w);
        	   	}
        	   	correct_yaw = 1;
        	}

        	while(distance > 1.5)
        	{
        		if(new_gps_data.altitude < gps_data.altitude + 3.0)
        		{
					send_drone_cmd_vel(0.5, 0.0, 0.5, 0.0);
					rcl_publish(&drone_cmd_vel_publisher, &cmd_vel_msg, NULL);
					rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
					distance = calculate_distance(new_gps_data.latitude, new_gps_data.longitude, gps_data.latitude, gps_data.longitude);
					correct_yaw = 2;
        		}
        		else
        		{
					send_drone_cmd_vel(0.5, 0.0, -0.05, 0.0);
					rcl_publish(&drone_cmd_vel_publisher, &cmd_vel_msg, NULL);
					rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
					distance = calculate_distance(new_gps_data.latitude, new_gps_data.longitude, gps_data.latitude, gps_data.longitude);
					correct_yaw = 2;
        		}
        	}
        	while(1)
        	{
        		global_center_axis_x = calculate_image_point_x(global_center_x);
        		global_center_axis_y = calculate_image_point_y(global_center_y);
        		send_drone_cmd_vel(global_center_axis_y, global_center_axis_x, -0.1, 0.0);
				rcl_publish(&drone_cmd_vel_publisher, &cmd_vel_msg, NULL);
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
				correct_yaw = 3;
				if(new_gps_data.altitude < 0.6) // (new_gps_data.altitude < gps_data.altitude + 0.3)
				{
		       		rcl_publish(&land_publisher, &land_msg, NULL);
		       		correct_yaw = 4;
		       		distance = 2.0;
					break;
				}
        	}

       	}
        osDelay(10);
    }

    rcl_ret_t ret;
    ret = rcl_subscription_fini(&imu_subscriber, &node);
    ret = rcl_subscription_fini(&gps_subscriber, &node);
    ret = rcl_subscription_fini(&new_gps_subscriber, &node);
    ret = rcl_subscription_fini(&new_imu_subscriber, &node); // Yeni IMU aboneliği sonlandırma
    ret = rcl_publisher_fini(&cmd_vel_publisher, &node);
    ret = rcl_publish(&cmd_vel_publisher, &cmd_vel_data, NULL);
    ret = rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
