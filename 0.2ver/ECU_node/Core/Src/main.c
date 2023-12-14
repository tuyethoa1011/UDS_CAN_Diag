/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @identity		: ECU module
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	IDLE_STATE,
	READREQUEST_STATE,
	SECURITY_ACCESS_STATE,
	WRITEREQUEST_STATE,
}ECU_Status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Diagnostic Message Format*/
#define FT_SINGLEFRAME 0x00
#define FT_FIRSTFRAME 0x01
#define FT_CONSECUTIVEFRAME 0x02
#define FT_FLOWCONTROL 0x03
/*Diagnostic Message Format*/
/*Data length Define*/
#define DL_NONE_DATABYTE 0x00
#define DL_1_DATABYTE 0x10
#define DL_2_DATABYTE 0x20
#define DL_3_DATABYTE 0x30
#define DL_4_DATABYTE 0x40
#define DL_5_DATABYTE 0x50
#define DL_6_DATABYTE 0x60
#define DL_7_DATABYTE 0x70
/*Data length Define*/
/*Response define*/
#define POSITIVE_RESPONSE 0x00U
#define NEGATIVE_RESPONSE 0x01U

//DID: 0x0123 - ECU Name (FF) - Test label (SF)
//#define ECU_MANUFACTURER_NAME "STM32F103C8T6" //For FF testcase
#define TEST_LABEL "TEST"
/*Define IDS - Indetifier Services*/
#define ReadDataByIdentifier 0x22
#define WriteDataByIndentifier 0x2E
#define SecurityAccess 0x27

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef ReadRsp_TxHeader;
CAN_RxHeaderTypeDef ReadRsp_RxHeader;

uint32_t ReadRspTxMailbox;

uint8_t PositiveReadRsp_TxData[8];
uint8_t NegativeReadRsp_TxData[8] = {0x03,0x7F,0x22,0x13,0x00,0x00,0x00,0x00};

uint8_t ReadRsp_RxData[8],Tx_Buffer[55],DataBuffer[1024];

uint8_t FT_String, DL_String;

uint8_t FrameType, DataLength;

uint16_t DID_Val;

uint8_t index_array;

uint8_t error_flag = 0,  ReadRq_flag = 0, security_flag;

ECU_Status ecu_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t SF_N_PCI_FrameTypeHandle(uint8_t byteString);
uint8_t SF_N_PCI_DataLenngthHandle(uint8_t byteString);
uint8_t GetFrameType(uint8_t FT_byteString);
uint8_t GetDataLength(uint8_t DL_byteString);
uint8_t Check_ReadRq_Validation(uint8_t FT, uint8_t DL, uint8_t Data_buf[]);
uint8_t GetCANFrameSize(uint8_t aData[]);
uint16_t GetDID(uint8_t DID_HByteString,uint8_t DID_LByteString);
void ReadRequest_handle(void);
void GetDIDValueFromRecordTableOfDID(uint16_t DID_Val,uint8_t *buf_arr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t GetDID(uint8_t DID_HByteString,uint8_t DID_LByteString)
{
	uint16_t DID_res;
	DID_res = (DID_HByteString << 8) | DID_LByteString;
	return DID_res;
}
uint8_t GetCANFrameSize(uint8_t aData[])
{
	uint8_t SizeCount = 0;
	for(index_array=0;index_array<8;index_array++)
	{
		if(aData[index_array]!=0)
		{
			SizeCount++;
		}
	}
	return SizeCount;
}
uint8_t Check_ReadRq_Validation(uint8_t FT, uint8_t DL, uint8_t Data_buf[])
{
	uint8_t Resp_res;
	/*Negative response case*/
	//case 1
	if(FT > 3)
	{
		//transmit negative response
		Resp_res = NEGATIVE_RESPONSE;
	} else
	{
		switch(FT)
		{
			case 0: //single frame
			{
				if(DL>7)
				{
					Resp_res = NEGATIVE_RESPONSE;
				} else {
					if(DL != (GetCANFrameSize(Data_buf)-1))
					{
						//transmit negative response
						Resp_res = NEGATIVE_RESPONSE;
					} else {
						Resp_res = POSITIVE_RESPONSE;
					}
				}
				break;
			}
			case 1: //flow control
			{
				break;
			}
		}
	}

	return Resp_res;
}
uint8_t GetFrameType(uint8_t FT_byteString) //Frame type co the dung chung cho SF FF
{
	uint8_t FT_res;
	switch(FT_byteString)
	{
		case FT_SINGLEFRAME:
		{
			FT_res = 0;
			break;
		}
		case FT_FIRSTFRAME:
		{
			FT_res = 1;
			break;
		}
		case FT_CONSECUTIVEFRAME:
		{
			FT_res = 2;
			break;
		}
		case FT_FLOWCONTROL:
		{
			FT_res = 3;
			break;
		}
	}
	return FT_res;
}
uint8_t GetDataLength(uint8_t DL_byteString) //Get datalength - co the trong tuong lai cai nay se dung chung duoc
{
	uint8_t DL_res;
	switch(DL_byteString)
	{
		case DL_NONE_DATABYTE:
		{
			DL_res = 0;
			break;
		}
		case DL_1_DATABYTE:
		{
			DL_res = 1;
			break;
		}
		case DL_2_DATABYTE:
		{
			DL_res = 2;
			break;
		}
		case DL_3_DATABYTE:
		{
			DL_res = 3;
			break;
		}
		case DL_4_DATABYTE:
		{
			DL_res = 4;
			break;
		}
		case DL_5_DATABYTE:
		{
			DL_res = 5;
			break;
		}
		case  DL_6_DATABYTE:
		{
			DL_res = 6;
			break;
		}
		case DL_7_DATABYTE:
		{
			DL_res = 7;
			break;
		}
	}
	return DL_res;
}
uint8_t SF_N_PCI_FrameTypeHandle(uint8_t byteString) //SF_PCI: Single Frame Protocol Control Info: FT + DL
{
	return (byteString >> 4) & 0x03;
}
uint8_t SF_N_PCI_DataLenngthHandle(uint8_t byteString) //SF_PCI: Single Frame Protocol Control Info: FT + DL
{
	return (byteString << 4) & 0x70;
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &ReadRsp_RxHeader, ReadRsp_RxData) != HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		ReadRq_flag = 1;
	}
}


void ReadRequest_handle(void)
{
	FT_String = SF_N_PCI_FrameTypeHandle(ReadRsp_RxData[0]);
	DL_String = SF_N_PCI_DataLenngthHandle(ReadRsp_RxData[0]);

	FrameType = GetFrameType(FT_String);
	DataLength = GetDataLength(DL_String);

	if(Check_ReadRq_Validation(FrameType,DataLength, ReadRsp_RxData) != POSITIVE_RESPONSE)
	{
		//transmit negative response CAN frame packet
		HAL_CAN_AddTxMessage(&hcan, &ReadRsp_TxHeader,NegativeReadRsp_TxData,&ReadRspTxMailbox);

	} else { //positive response
		DID_Val = GetDID(ReadRsp_RxData[2], ReadRsp_RxData[3]);

		//uint8_t *DataBuffer = (uint8_t*)malloc(sizeof(uint8_t));
		//GetDIDValueFromRecordTableOfDID(DID_Val,DataBuffer);

		switch(DID_Val)
		{
			case 0x0123:
				uint8_t DataBuffer[sizeof(TEST_LABEL)] = TEST_LABEL;

				if(sizeof(DataBuffer)<8){
					PositiveReadRsp_TxData[0] = ReadRsp_RxData[1] + 0x40;
					PositiveReadRsp_TxData[1] = ReadRsp_RxData[2];
					PositiveReadRsp_TxData[2] = ReadRsp_RxData[3];

					PositiveReadRsp_TxData[3] = FT_SINGLEFRAME + sizeof(DataBuffer); //du lieu data co do dai la 4 - TEST
					for (index_array = 0;index_array<sizeof(DataBuffer);index_array++)
					{
						PositiveReadRsp_TxData[index_array+4] = DataBuffer[index_array];
					}
					//gui single frame - chua toan bo du lieu
					if (HAL_CAN_AddTxMessage(&hcan, &ReadRsp_TxHeader,PositiveReadRsp_TxData,&ReadRspTxMailbox) != HAL_OK)
					{
								error_flag = 1;
					} else //Transmit oke
					{
								error_flag = 0;
					}
					} else {
						//first frame case - long data
					}
					//free(DataBuffer);
			break;
		}
	}
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Init(&hcan);

  //Send notify signal to recieve message from actuator node
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 2;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  canfilterconfig.FilterIdHigh = 0x712 << 5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x712 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  //canfilterconfig.SlaveStartFilterBank = 14;

   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

   HAL_CAN_Start(&hcan);

   //Tx ECU header general
   //Config CAN Tx header
   ReadRsp_TxHeader.DLC = 8;
   ReadRsp_TxHeader.ExtId = 0;
   ReadRsp_TxHeader.IDE = CAN_ID_STD;
   ReadRsp_TxHeader.RTR = CAN_RTR_DATA;
   ReadRsp_TxHeader.StdId = 0x7A2;
   ReadRsp_TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 HAL_Delay(1000);

	 switch(ecu_state)
	 {
	 	 case IDLE_STATE:
	 	 {
	 		 if(ReadRq_flag == 1)
	 		 {
	 			 switch(ReadRsp_RxData[1])
	 			 {
	 			 	 case ReadDataByIdentifier:
	 			 	 {
	 			 		 ecu_state = READREQUEST_STATE;
	 			 		 break;
	 			 	 }
	 			 	 case WriteDataByIndentifier:
	 			 	 {
	 			 		 if(security_flag == 0)
	 			 		 {
	 			 			 //do nothing because there is no definition for this
	 			 			 //just go back to idle state - solution 1
	 			 			 //solution 2 - Security Access then write
	 			 			 ecu_state = IDLE_STATE;
	 			 			 //ecu_state = SECURITY_ACCESS_STATE;
	 			 			 //getwrite_flag = 1;
	 			 		 }
	 			 		 else if(security_flag == 1)
	 			 		 {
	 			 			ecu_state = WRITEREQUEST_STATE;
	 			 		 }
	 			 		 break;
	 			 	 }
	 			 	 case SecurityAccess:
	 			 	 {
	 			 		 ecu_state = SECURITY_ACCESS_STATE;
	 			 		 break;
	 			 	 }
	 			 }
	 		 } else if (ReadRq_flag == 0)
	 		 {
	 			 ecu_state = IDLE_STATE;
	 		 }
	 		 break;
	 	 }
	 	 case READREQUEST_STATE:
	 	 {
	 		 ReadRequest_handle();
	 		 ecu_state = IDLE_STATE;
	 		 break;
	 	 }
	 	 case WRITEREQUEST_STATE:
	 	 {
	 		 ecu_state = IDLE_STATE;
	 		 break;
	 	 }
	 	 case SECURITY_ACCESS_STATE:
	 	 {
	 		 ecu_state = IDLE_STATE;
//	 		 if(getwrite_flag == 1)
//	 		 {
//	 			 ecu_state == WRITEREQUEST_STATE;
//	 			 getwrite_flag = 0;
//	 		 }
	 		 break;
	 	 }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
