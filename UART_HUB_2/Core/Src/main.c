/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"
//#include "bhy2_defs.h"

#include <stdarg.h>
#include <stdio.h>
//#include "Bosch_SHUTTLE_BHI260.fw.h"
#include "Bosch_SHUTTLE_BHI260_2.fw.h"
//#include "Bosch_SHUTTLE_BHI260_BMM150.fw.h"

#include "IMU_funcs.h"
#include <string.h>
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Порт, на котором находится imu
#define BHY2_I2C hi2c2
#define BHY2_SPI hspi1
//Собственный адрес микросхемы
#define BHY2_ADDR (0x28 << 1)

//Размер рабочего буфера
#define BFY2_WORK_BUFFER_SIZE   2048

int mError=10;//!< Последняя ошибка

static uint8_t mWorkBuffer[BFY2_WORK_BUFFER_SIZE];

//�?дентификатор сенсора (микросхема содержит кучу виртуальных сенсоров)
#define QUAT_SENSOR_ID     BHY2_SENSOR_ID_GAMERV

//Дефайны для UART хаба
#define SOM1 0xF0
#define SOM2 0xAA

#define SOM1_nByte    0//!<Порядковый номер байта Start of Message_1
#define SOM2_nByte    1//!<Порядковый номер байта Start of Message_2
#define MsgLen_nByte  2//!<Порядковый номер байта длины передаваемого сообщения
#define AnswLen_nByte 3//!<Порядковый номер байта длины ответного сообщения от KONDO
#define Addr_nByte    4//!<Порядковый номер байта содержащего адрес переферии
#define Msg_nByte     5//!<Порядковый первого байта в сообщении от RPi_CM4

#define UART2_ADDR  0x00
#define UART4_ADDR  0x01
#define UART7_ADDR  0x02
#define UART8_ADDR  0x03
#define I2C2_ADDR   0x04
#define SYNC_ADDR   0x05
#define IMU_CS_ADDR 0x06
#define QUATERNION_ADDR 0x07

#define I2C_TIMEOUT 10

#define BUFFERS_LEN   255//!< Длина всех буферов

#define IMU_REQUEST_KONDO  0xbad
#define IMU_REQUEST_LENGTH 0xbad

//uint8_t txBuf[1] = {0};
uint8_t rxBuf[1] = { 0 };//!<Буфер для байта, полученного при прерывании
uint8_t rxbuf_len = 1;//!<Длина буфера, полученного при прерывании

//kondo
uint8_t rxBufKondo[1] = { 0 };//!<Буфер для одиночного байта от KONDO, полученного при прерывании
uint8_t rxbuf_len_kondo = 1;//!<Длина ,буфера для одиночного байта от KONDO, полученного при прерывании

uint8_t txBufForKondo[BUFFERS_LEN] = { 0 };
uint8_t txbuf_len_kondo = 1;

uint8_t MsgBufRX[BUFFERS_LEN] = { 0 };//!<Буффер для хранения сообщения от мастера(RPi_CM4) интерфейс - STM32_UART3
uint8_t MsgBufLen = BUFFERS_LEN;
uint8_t MsgCnt = 0; //Счётчик текущего количества байт в сообщении

uint8_t MsgBufTX[BUFFERS_LEN] = { 0 };
uint8_t AnswBufLen = BUFFERS_LEN;
uint8_t AnswCnt = 0;//Счётчик текущего количества байт в сообщении в принятом сообщении от KONDO
uint8_t AnswerIsReady = 0;

uint8_t regData = 0; //I2C2 buffer

uint8_t TransmitIsReady = 0; //!<Флаг готовности данных к передаче по интерфейсу
uint8_t Periph_Addr = 0; //!<Переменная для хранения адреса переферии

//rst buff
uint8_t rxBufRST[3] = { 0 };
uint8_t rstbuf_len = 3;
uint8_t i = 0;

uint8_t dmaTxCompleted_uart2 = 1;//!<Флаг окончания передачи DMA по UART2
uint8_t dmaTxCompleted_uart3 = 1;//!<Флаг окончания передачи DMA по UART3

uint8_t IMU_head_tim = 1;

uint8_t kondo_is_IMU = 0;

struct config
{
    uint16_t sensitivity;
    uint16_t range;
    uint32_t latency;
    bhy2_float sample_rate;
};

struct Quaternion
{
    uint64_t mTimestamp; //!< Штамп времени в нс
    uint32_t mTimeS;     //!< Секунды штампа времени
    uint32_t mTimeNS;    //!< Наносекунды штампа времени
    int16_t    mX;       //!< Координаты
    int16_t    mY;
    int16_t    mZ;
    int16_t    mW;

    float    mAcc;       //!< Точность представления

    uint8_t  mSensorId;  //!< �?дентификатор сенсора
};

#define QUATERNION_BYTE_LENGHT 16//!<Длина посылки кватериниона в байтах
uint8_t qt_component_buffer[QUATERNION_BYTE_LENGHT] = { 0 };

static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
  {
    //Результирующий кватернион
    //struct Quaternion *qt = (Quaternion*)callback_ref;
	struct Quaternion *qt = callback_ref;
    //struct Quaternion *qt;
    struct bhy2_data_quaternion data;

    if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
      {
      return;
      }

    //Парсить фрейм и получить кватернион
    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    //Заполнить выходную структуру
    qt->mSensorId = callback_info->sensor_id;
    qt->mTimestamp = *callback_info->time_stamp; /* Store the last timestamp */

    qt->mTimestamp *= 15625; /* Timestamp is now in nanoseconds */
    qt->mTimeS  = (uint32_t)(qt->mTimestamp / UINT64_C(1000000000));
    qt->mTimeNS = (uint32_t)(qt->mTimestamp - (qt->mTimeS * UINT64_C(1000000000)));

    /*
    qt->mX = data.x / 16384.0f;
    qt->mY = data.y / 16384.0f;
    qt->mZ = data.z / 16384.0f;
    qt->mW = data.w / 16384.0f;
    */

    qt->mX = data.x;
    qt->mY = data.y;
    qt->mZ = data.z;
    qt->mW = data.w;

    qt->mAcc = ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f;

    *qt_component_buffer = ((qt->mX)>>8);
    *(qt_component_buffer + 1) = (uint8_t)qt->mX;

    *(qt_component_buffer + 2) = ((qt->mY)>>8);
    *(qt_component_buffer + 3) = (uint8_t)qt->mY;

    *(qt_component_buffer + 4) = ((qt->mZ)>>8);
    *(qt_component_buffer + 5) = (uint8_t)qt->mZ;

    *(qt_component_buffer + 6) = ((qt->mW)>>8);
    *(qt_component_buffer + 7) = (uint8_t)qt->mW;

    *(qt_component_buffer + 8) = ((qt->mTimestamp)>>56);
    *(qt_component_buffer + 9) = ((qt->mTimestamp)>>48);
    *(qt_component_buffer + 10) = ((qt->mTimestamp)>>40);
    *(qt_component_buffer + 11) = ((qt->mTimestamp)>>32);
    *(qt_component_buffer + 12) = ((qt->mTimestamp)>>24);
    *(qt_component_buffer + 13) = ((qt->mTimestamp)>>16);
    *(qt_component_buffer + 14) = ((qt->mTimestamp)>>8);
    *(qt_component_buffer + 15) = (uint8_t)qt->mTimestamp;

    //HAL_UART_Transmit(&huart2, qt_component_buffer, 8, 10);
    //uint8_t str[100];

    //sprintf(str, "Quater: tm=%d x=%f y=%f z=%f w=%f acc=%f\n\r", qt->mTimeS, qt->mX, qt->mY, qt->mZ, qt->mW, qt->mAcc );
    //usbPrintf( "Quater: tm=%d x=%f y=%f z=%f w=%f acc=%f\n\r", qt->mTimeS, qt->mX, qt->mY, qt->mZ, qt->mW, qt->mAcc );
    //uartPrintf( "Quater: tm=%d x=%f y=%f z=%f w=%f acc=%f\n\r", qt->mTimeS, qt->mX, qt->mY, qt->mZ, qt->mW, qt->mAcc );
  }

//-------------------------------------------------------------------------------------------

struct Message
{
	const uint8_t* buffer;
	const uint8_t  size;
};



// Creates copy of message
struct Message copy_construct(const struct Message* src)
{
	assert(src);

	size_t size = src->size;
	uint8_t* new_buffer = (uint8_t*)calloc(size, sizeof(uint8_t));

	assert(new_buffer);
	memcpy(new_buffer, src->buffer, size);

	struct Message msg = {new_buffer, size};
	return msg;
}

// Safely destructs message
void destruct(struct Message* msg)
{
	assert(msg);
	assert(msg->buffer);

	free(msg->buffer);
}

// Sends message to huart
void send(UART_HandleTypeDef *huart, const struct Message* msg, const uint8_t timeout)
{
	assert(msg);
	assert(msg->buffer);

	HAL_UART_Transmit(huart, msg->buffer, msg->size, timeout);
}

//-------------------------------------------------------------------------------------------

/*
typedef uint8_t FIFO_TYPE;

uint8_t copy_construct(const uint8_t* src)
{
	return *src;
}

void destruct(const uint8_t* msg)
{

}

void send(UART_HandleTypeDef *huart, const uint8_t* msg, const uint8_t timeout)
{
	HAL_UART_Transmit(huart, msg, sizeof(uint8_t), timeout);
}
*/
//-------------------------------------------------------------------------------------------

#define FIFO_SIZE 20
typedef struct Message FIFO_TYPE;

struct FIFO
{
	size_t head;
	size_t tail;
	size_t size;
	FIFO_TYPE buf[FIFO_SIZE];
};

// Adds the value to FIFO
void fifo_add(struct FIFO* fifo, FIFO_TYPE value)
{
	assert(fifo);
	assert(fifo->size < FIFO_SIZE);

	FIFO_TYPE value_copy = copy_construct(&value);

	memcpy(&(fifo->buf[fifo->tail]), &value_copy, sizeof(FIFO_TYPE));
	fifo->tail = (fifo->tail + 1) % FIFO_SIZE;

	fifo->size++;
}

// WARNING: don't forget to destruct the value if you use this function instead of fifo_send
FIFO_TYPE fifo_get(struct FIFO* fifo)
{
	assert(fifo);
	assert(fifo->size > 0);

	FIFO_TYPE value = fifo->buf[fifo->head];

	fifo->head = (fifo->head + 1) % FIFO_SIZE;

	fifo->size--;
	return value;
}

// Gets the value from FIFO, sends it to huart and destructs it
void fifo_send(struct FIFO* fifo, UART_HandleTypeDef *huart, const uint8_t timeout)
{
	FIFO_TYPE msg = fifo_get(fifo);
	send(huart, &msg, timeout);
	destruct(&msg);
}

struct FIFO kondo_fifo = {0};

//-------------------------------------------------------------------------------------------
/*
void pack(struct Message* msg, const struct Message kondo_responce, const struct Message quaternion)
{
	assert(msg);
	assert(msg->buffer);

	msg->buffer[0] = kondo_responce.size;
	memcpy(msg->buffer + 1, kondo_responce.buffer, kondo_responce.size);
	msg->buffer[kondo_responce.size + 1] = quaternion.size;
	memcpy(msg->buffer + kondo_responce.size + 2, quaternion.buffer, quaternion.size);
	msg->size = kondo_responce.size quaternion.size + 2;
}
*/
//-------------------------------------------------------------------------------------------
void clear_msg_buf(void)
{
	uint32_t j = 0;
	for (j = 0; j < BUFFERS_LEN; j++)
		MsgBufRX[j] = 0;
	j = 0;
}

void clear_buf(uint8_t *buff, uint8_t buffNumber)
{
	uint8_t j = 0;
	for (j = 0; j < buffNumber; j++)
		buff[j] = 0;
	j = 0;
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, rxBuf, rxbuf_len);
  HAL_UART_Receive_IT(&huart8, rxBufKondo, rxbuf_len_kondo);
  HAL_UART_Receive_IT(&huart4, rxBufRST, rstbuf_len);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);

  struct config conf;
  struct Quaternion mQuaternion; //!< Текущий кватернион
  static uint8_t product_id = 0;
  struct bhy2_dev bhy2;
  static uint16_t bhy2KernelVersion;

  //i2c_init(&BHY2_I2C, BHY2_ADDR);
  spi_init(&BHY2_SPI);

  //uint8_t sensor_error=0;
  //mError = bhy2_init( BHY2_I2C_INTERFACE, &bhy2_i2c_read, &bhy2_i2c_write, bhy2_delay_us, 64, NULL, &bhy2 );
  mError = bhy2_init( BHY2_SPI_INTERFACE, &bhy2_spi_read, &bhy2_spi_write, bhy2_delay_us, 64, NULL, &bhy2 );
  if( mError ) return 1;
  //GPIOA->BSRR = (uint32_t)SPI_CS_Pin << (16U);
  mError = bhy2_soft_reset(&bhy2);
  if( mError ) return 2;
  //HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  //
  //uint8_t ChipControl = 1;


  //bhy2_set_regs(0x05, const uint8_t *reg_data, uint16_t length, struct bhy2_dev *dev);


  mError = bhy2_get_product_id(&product_id, &bhy2);
  if( mError ) return 3;

  if( product_id != BHY2_PRODUCT_ID )
      return 0;
  /* Check the interrupt pin and FIFO configurations. Disable status and debug */
  uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;// | BHY2_ICTL_EDGE | BHY2_ICTL_ACTIVE_LOW;

  mError = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
  if( mError ) return 4;
  //bhy2_get_regs(BHY2_REG_HOST_INTERRUPT_CTRL, &ChipControl, 1, &bhy2);
  mError = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2);
  if( mError ) return 5;

  uint8_t hif_ctrl = 0;
  mError = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
  if( mError ) return 6;

  /* Check if the sensor is ready to load firmware */
  uint8_t boot_status;
  mError = bhy2_get_boot_status(&boot_status, &bhy2);
  if( mError ) return 7;

  //Проверим готовность микросхемы к загрузке firmware
  if( !(boot_status & BHY2_BST_HOST_INTERFACE_READY) )
  return 8;

  uint8_t sensor_error;
  if( boot_status & BHY2_BST_HOST_INTERFACE_READY )
  {
  mError = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), &bhy2);

  }
  if( mError ) return 9;

  mError = bhy2_get_error_value(&sensor_error, &bhy2);
  if( mError || sensor_error )
    return 10;

  //Стартуем программу, загруженную в ram
  mError = bhy2_boot_from_ram(&bhy2);
  if( mError ) return 11;

  mError = bhy2_get_error_value(&sensor_error, &bhy2);
  if( mError || sensor_error )
    return 12;

  mError = bhy2_get_kernel_version(&bhy2KernelVersion, &bhy2);
  if( mError || bhy2KernelVersion == 0 ) return 13;

  //rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2);
  //  print_api_error(rslt, &bhy2);
  //  rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2);
  //  print_api_error(rslt, &bhy2);

  mError = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, &mQuaternion, &bhy2);
  if( mError ) return 14;

  mError = bhy2_get_and_process_fifo( mWorkBuffer, BFY2_WORK_BUFFER_SIZE, &bhy2);
  if( mError ) return 15;

   /*Update the callback table to enable parsing of sensor data*/
  mError = bhy2_update_virtual_sensor_list(&bhy2);
  if( mError ) return 16;

  float sample_rate = 800.0; /* Read out data measured at 100Hz */
  uint32_t report_latency_ms = 0; /* Report immediately */
  mError = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
  if( mError ) return 17;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if (TransmitIsReady == 0 && IMU_head_tim)
	  {
		  uint8_t interruptStatus = 0;
		  bhy2_get_interrupt_status( &interruptStatus, &bhy2 );

		  if( interruptStatus ){
			  bhy2_get_and_process_fifo( mWorkBuffer, BFY2_WORK_BUFFER_SIZE, &bhy2 );
			  IMU_head_tim = 0;
		  }

		  //HAL_UART_Transmit(&huart3, qt_component_buffer, QUATERNION_BYTE_LENGHT, 10);
	  }


      if (AnswerIsReady)
      	{
      	 //__disable_irq();
      	 HAL_UART_Transmit(&huart3, txBufForKondo, AnswBufLen, 10);

      	 /*if (dmaTxCompleted_uart3)
		    {
		   	  HAL_UART_Transmit_DMA(&huart3, txBufForKondo, AnswBufLen);
		   	  dmaTxCompleted_uart3 = 0;
		    }
         */
         AnswerIsReady = 0;
      	 clear_buf(txBufForKondo, AnswBufLen);
      	 //__enable_irq();
      	}

      	  	if (TransmitIsReady)
      		{
      			memcpy(MsgBufTX, MsgBufRX, sizeof(MsgBufRX));
      			clear_msg_buf();
      			MsgCnt = 0;
      			rxBuf[0] = 0;

      			//__disable_irq();
      			switch(Periph_Addr)
      			{
      				case UART2_ADDR:
      					if (dmaTxCompleted_uart2)
    				     {
    				    	 HAL_UART_Transmit_DMA(&huart2, &MsgBufTX[Msg_nByte], MsgBufLen);
    				    	  dmaTxCompleted_uart2 = 0;
    				     }
      				break;

      				case UART4_ADDR:
      					HAL_UART_Transmit(&huart4, &MsgBufTX[Msg_nByte], MsgBufLen, 5);
      				break;

      				case UART7_ADDR:
      					HAL_UART_Transmit(&huart7, &MsgBufTX[Msg_nByte], MsgBufLen, 5);
      				break;

      				case UART8_ADDR:
      				{
      					//for (i = Msg_nByte; i < (Msg_nByte + MsgBufLen); i++)
      					//{
      					//	HAL_UART_Transmit(&huart8, &MsgBufTX[i], 1, 5);
      					//}
      					//HAL_UART_Transmit(&huart8, &MsgBufTX[Msg_nByte], MsgBufLen, 5);
      					struct Message kondo_msg = {&MsgBufTX[Msg_nByte], MsgBufLen};
      					fifo_add(&kondo_fifo, kondo_msg);

      				}
      				break;

      				case I2C2_ADDR:
      				{
      					HAL_I2C_Master_Transmit(&hi2c2, (MsgBufTX[Msg_nByte] << 1), &MsgBufTX[Msg_nByte+1], 1,  I2C_TIMEOUT);
      					HAL_I2C_Master_Receive(&hi2c2, (MsgBufTX[Msg_nByte] << 1), &regData, 1,  I2C_TIMEOUT);
      					if (regData > 0)
      						{
      							//HAL_UART_Transmit(&huart3, &regData, 1, 10);

      						HAL_UART_Transmit_DMA(&huart3, &regData, 1);
      							regData = 0;
      						}
      				}
      				break;

      				case SYNC_ADDR:
      					if (MsgBufTX[Msg_nByte] == 1)
      					{
      						HAL_GPIO_WritePin(CAM_SYNC_GPIO_Port, CAM_SYNC_Pin, GPIO_PIN_SET);
      					}
      					else
      						HAL_GPIO_WritePin(CAM_SYNC_GPIO_Port, CAM_SYNC_Pin, GPIO_PIN_RESET);
      				break;

      				case IMU_CS_ADDR:
      					if (MsgBufTX[Msg_nByte] == 1)
      					{
      						HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
      					}
      					else
      						HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
      				break;

      				case QUATERNION_ADDR:
      					{
      				      if (dmaTxCompleted_uart3)
      				      {

      				    	//uint8_t qt_component_buffer[16] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x0A};
      						//HAL_UART_Transmit(&huart3, qt_component_buffer, QUATERNION_BYTE_LENGHT, 10);
      						HAL_UART_Transmit_DMA(&huart3, qt_component_buffer, QUATERNION_BYTE_LENGHT);
      				    	dmaTxCompleted_uart3 = 0;
      				      }
      					}
      				break;
      			}
      			TransmitIsReady = 0;
      			//__enable_irq();au
      		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		dmaTxCompleted_uart2 = 1;
	if (huart->Instance == USART3)
		dmaTxCompleted_uart3 = 1;
}



uint8_t flag = 0;
uint8_t SOM1isCirrect = 0;
uint8_t SOM2isCirrect = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		MsgBufRX[MsgCnt] = rxBuf[0];
		if (MsgBufRX[SOM1_nByte] == SOM1){
			if (MsgCnt == 0){
			SOM1isCirrect = 1;
			}
		} else {
			clear_msg_buf();
			MsgCnt = 0;
			rxBuf[0] = 0;
			SOM1isCirrect = 0;
			SOM2isCirrect = 0;
			flag = 0;
		}

		if (SOM1isCirrect)
			{
			if (MsgCnt == SOM2_nByte)
				{
				if (MsgBufRX[SOM2_nByte] == SOM2)
					{
					SOM2isCirrect = 1;
					} else SOM2isCirrect = 0;
				if (!SOM1isCirrect || !SOM2isCirrect)
					{
							clear_msg_buf();
							MsgCnt = 0;
							rxBuf[0] = 0;
							SOM1isCirrect = 0;
							SOM2isCirrect = 0;
							flag = 0;
					}
				}
			}
		if (MsgCnt == MsgLen_nByte)
			{
				if (MsgBufRX[MsgLen_nByte] < 255)
				MsgBufLen = MsgBufRX[MsgLen_nByte];
			}
		if (MsgCnt == AnswLen_nByte)
			{
				AnswBufLen = MsgBufRX[AnswLen_nByte];
			}
		if (SOM1isCirrect)
			{
				MsgCnt++;
			}
		if (MsgCnt == (MsgBufLen + 5) && (SOM1isCirrect && SOM2isCirrect))
			{
				TransmitIsReady = 1;
				Periph_Addr = MsgBufRX[Addr_nByte];
				//if (Periph_Addr == 0xFF) __NVIC_SystemReset();
			}
		HAL_UART_Receive_IT(&huart3, rxBuf, rxbuf_len);
	}

	if (huart->Instance == UART8)
	{
		txBufForKondo[AnswCnt] = rxBufKondo[0];
		AnswCnt++;
		if (AnswCnt == AnswBufLen)
		{
		 AnswerIsReady = 1;
		 AnswCnt = 0;
		}
		if (AnswCnt > AnswBufLen)
			AnswCnt = 0;
		HAL_UART_Receive_IT(&huart8, rxBufKondo, rxbuf_len_kondo);
	}

	if (huart->Instance == UART4)
	{
		if ((rxBufRST[0] == 0xF0) & (rxBufRST[1] == 0xAA) & (rxBufRST[2] == 0x0F))
		{
			__NVIC_SystemReset();
		}
		HAL_UART_Receive_IT(&huart4, rxBufRST, rstbuf_len);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    if (kondo_fifo.size > 0)
    {
      fifo_send(&kondo_fifo, &huart8, 5);
    }
  }
  if (htim->Instance == TIM2)
	  IMU_head_tim = 1;
/*
  if (htim->Instance == TIM4)
  	  respond();
*/
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

