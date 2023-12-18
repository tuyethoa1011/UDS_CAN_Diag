/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @identity		: Tester module
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
#include "button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Tester module state define
typedef enum
{
	INIT_STATE,
	REQUEST_READ,
	REQUEST_WRITE,
	REQUEST_SECURITY_ACCESS,
}Tester_Status;

typedef enum
{
	REQUEST_SEED,//can phai co flag de chuong trinh chay request khong lam gi
	SEND_KEY,
	AUTHENTICATE_RESPONSE,
}Security_Status;

typedef enum
{
	FIRSTFRAME_SEND,
	FlOWCONTROL_CHECK,
	CONSECUTIVEFRAME_SEND,
}Multiflow_Status;

typedef enum
{
	CONTINUE_TO_SEND,
	WAIT,
	OVERFLOW,
}Flow_State;

typedef enum
{
	SEND_WRITEREQUEST,
	WRITE_RESPONSE,
}Write_Status;

typedef enum
{
	NOT_PRESS,
	IS_PRESS,
}Button_Status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Diagnostic Message Format*/
#define FT_SINGLEFRAME 0x00
#define FT_FIRSTFRAME 0x10
#define FT_CONSECUTIVEFRAME 0x20
#define FT_FLOWCONTROL 0x30
/*Diagnostic Message Format*/

/*Define Data Length*/
#define KEY_LENGTH 16
/*Define Data Length*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef Tester_TxHeader;
CAN_RxHeaderTypeDef Tester_RxHeader;

uint32_t Tester_TxMailbox;

uint8_t ReadRq_TxData[8] = {0x03,0x22,0x01,0x23,0x00,0x00,0x00,0x00};

uint8_t Tester_RxData[8];

uint8_t SecARq_TxData[8] = {0x02,0x27,0x01,0x00,0x00,0x00,0x00,0x00};
uint8_t UnlockRq_TxData[8];
uint8_t KeySend_TxData[8];
uint8_t WriteRq_TxData[8] = {0x07,0x2E,0x01,0x023,'H','A','N','A'};

uint8_t Tx_Buffer[55];

uint8_t FT_String,FrameType;

uint8_t flag_read_response = 0, security_flag = 0;
uint8_t error_flag = 0;
uint8_t request_seedsend_flag = 0;
uint8_t send_firstframe_flag = 0;
uint8_t continue_tosend_flag = 0;

uint16_t timer_cnt = 0;
uint16_t index_array;
uint16_t prev_index_array = 0;

//---- button handle variable ----
Button_Typedef BTN1;
uint8_t button_sig = 1; //thay doi gia tri bien de doc/ ghi
//button sig = 0 -> Doc du lieu cua ECU
//button sig = 1 -> Ghi du lieu vao ECU theo co che: SecA -> Write

//khai bao bien luu trang thai hien tai cua tester
Tester_Status tester_state;
Security_Status security_state = 0;
Multiflow_Status multiflow_state = 0;
Flow_State flow_state;
Write_Status write_state;
Button_Status button_state;

/*KEY security*/
uint8_t key[16];
/*SEED security recieve*/
uint8_t seed[4];

uint8_t MaxConsecutiveFrame_count = 16/7 + 1;
uint8_t sequence_num=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadFirstFrame_handle(void);
void ConscutiveFrameRead_handle(void);
uint8_t GetFrameType(uint8_t FT_byteString);
uint8_t SF_N_PCI_FrameTypeHandle(uint8_t byteString);
void ReadSingleFrame_handle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void button_longpressing_callback_500ms(Button_Typedef *ButtonX)
{
	if(ButtonX == &BTN1){
		button_sig = 1;
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Tester_RxHeader, Tester_RxData) != HAL_OK)
	{
		error_flag = 1;
	}
	else
	{
		flag_read_response = 1;
		error_flag = 0;
	}
}

uint8_t GetFrameType(uint8_t FT_byteString) //Frame type co the dung chung cho SF FF
{
	uint8_t FT_res;
	switch(FT_byteString)
	{
		case 0x00: //single frame
		{
			FT_res = 0;
			break;
		}
		case 0x01: //first frame
		{
			FT_res = 1;
			break;
		}
		case 0x02: //consecutive frame
		{
			FT_res = 2;
			break;
		}
		case 0x03: //flow control
		{
			FT_res = 3;
			break;
		}
	}
	return FT_res;
}

uint8_t SF_N_PCI_FrameTypeHandle(uint8_t byteString) //SF_PCI: Single Frame Protocol Control Info: FT + DL
{
	return (byteString >> 4) & 0x03;
}


void ReadSingleFrame_handle(void)
{
	//oled debugger - in ra du lieu vua nhan duoc thong qua single frame
	memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
	sprintf((char*)Tx_Buffer,"%s",Tester_RxData);
	HAL_UART_Transmit(&huart1,Tx_Buffer,sizeof(Tx_Buffer), 10);

	HAL_Delay(200);
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Init(&hcan);

  //Send notify signal to recieve message from actuator node
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  canfilterconfig.FilterIdHigh = 0x7A2 << 5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x7A2 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
   //canfilterconfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_CAN_Start(&hcan);

  //Config CAN header
  Tester_TxHeader.DLC = 8;
  Tester_TxHeader.ExtId = 0;
  Tester_TxHeader.IDE = CAN_ID_STD;
  Tester_TxHeader.RTR = CAN_RTR_DATA;
  Tester_TxHeader.StdId = 0x712;
  Tester_TxHeader.TransmitGlobalTime = DISABLE;

  button_Init(&BTN1, GPIOB, GPIO_PIN_0); //BTN1
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_Delay(500);
	  switch(tester_state) //change state
	  {
	 	case INIT_STATE:
		{
			//check buton_state
			if(button_sig==0)
			{
				tester_state = REQUEST_READ;
			} else if(button_sig==1)
			{
				tester_state = REQUEST_WRITE;
				//button_sig = 0;
			}
			break;
		}
	 	case REQUEST_READ: //send to ECU
	 	{
	 		//Gui theo chu ky 1s, tuc la cu 1 giay gui goi tin de doc 1 lan
			HAL_Delay(1000);

			if (HAL_CAN_AddTxMessage(&hcan, &Tester_TxHeader, ReadRq_TxData, &Tester_TxMailbox) != HAL_OK)
			{
				error_flag = 1;
			} else //Transmit oke
			{
				error_flag = 0;
			}

	 		if(flag_read_response == 1) {
	 			FT_String = SF_N_PCI_FrameTypeHandle(Tester_RxData[3]);
	 			FrameType = GetFrameType(FT_String); //check
	 			if(FrameType == FT_SINGLEFRAME)
	 			{
	 				ReadSingleFrame_handle();
	 				tester_state = INIT_STATE;
	 			}
	 			flag_read_response = 0;
	 		}
	 		break;
	 	}
	 	case REQUEST_WRITE:
	 	{
	 		switch(write_state)
	 		{
	 			case SEND_WRITEREQUEST:
	 			{
	 				if(security_flag == 0)
	 				{
	 					 tester_state = REQUEST_SECURITY_ACCESS;
	 				} else if (security_flag == 1)
	 				{
	 					//start transmitting write request to ECU
	 					if(HAL_CAN_AddTxMessage(&hcan,&Tester_TxHeader,WriteRq_TxData,&Tester_TxMailbox)!=HAL_OK)
	 					{
	 						error_flag = 1;
	 					}else{
	 						error_flag = 0;
	 					 	write_state = WRITE_RESPONSE;
	 					}
	 				}
	 				break;
	 			}
	 			case WRITE_RESPONSE:
	 			{
	 				//wait for write response
	 				if(flag_read_response == 1)
	 				{
	 					//check frame type
	 					//check what kind of response through Tester_RxData[1]
	 					FT_String = SF_N_PCI_FrameTypeHandle(Tester_RxData[0]);
	 					FrameType = GetFrameType(FT_String);

	 					if(FrameType==0)
	 					{
	 						switch(Tester_RxData[1])
	 						{
	 							case 0x6E: //positive
	 							{
	 								//do something here to know if write successful or not
	 								tester_state = REQUEST_READ; //positive response -> get back to init state
	 								break;
	 							}
	 							case 0x7F: //negative
								{
									//do something here to know if write successful or not
									//what kind of negative response we receive
									tester_state = INIT_STATE;
					 				//negative response -> get back to init state
									break;
								}
	 						}
	 					}
	 					flag_read_response = 0;
	 				}
	 				break;
	 			}
	 		}

	 		break;
	 	}
	 	case REQUEST_SECURITY_ACCESS:
	 	{
	 		switch(security_state)
	 		{
	 			case REQUEST_SEED:
				{
					switch(request_seedsend_flag)
					{
						case 0:
						{
							if(HAL_CAN_AddTxMessage(&hcan,&Tester_TxHeader,SecARq_TxData, &Tester_TxMailbox) != HAL_OK)
							{
								error_flag = 1;
							} else {
								error_flag = 0;
								request_seedsend_flag = 1; // co bao hieu da gui seed request
							}
							break;
						}
						case 1: //cho goi tin gui ve mang seed
						{
							if(flag_read_response)
							{	//calculate key before sending
				 				//get seed
				 				seed[0] = Tester_RxData[3];
				 				seed[1] = Tester_RxData[4];
				 				seed[2] = Tester_RxData[5];
				 				seed[3] = Tester_RxData[6];

				 				//calc key
				 				key[0] = seed[0] ^ seed[1]; //xor
				 				key[1] = seed[1] + seed[2];
				 				key[2] = seed[2] ^ seed[3];
				 				key[3] = seed[3] + seed[0];

				 				key[4] = seed[0] | seed[1];
				 				key[5] = seed[1] + seed[2];
				 				key[6] = seed[2] | seed[3];
				 				key[7] = seed[3] + seed[0];

				 				key[8] = seed[0] & seed[1];
				 				key[9] = seed[1] + seed[2];
				 				key[10] = seed[2] & seed[3];
				 				key[11] = seed[3] + seed[0];

				 				key[12] = seed[0] - seed[1];
				 				key[13] = seed[1] + seed[2];
				 				key[14] = seed[2] - seed[3];
				 				key[15] = seed[3] + seed[0];

								security_state = SEND_KEY;
								flag_read_response = 0;
								request_seedsend_flag = 0;
							}
							break;
						}
					}
	 				break;
				}
	 			case SEND_KEY:
	 			{

	 				switch(multiflow_state)
	 				{
	 					case FIRSTFRAME_SEND:
	 					{
	 						switch(send_firstframe_flag)
	 						{
	 							case 0:
	 							{
	 								UnlockRq_TxData[0] = 0x10;
	 								UnlockRq_TxData[1] = 0x16; //0-15 KEY DATA
	 								UnlockRq_TxData[2] = 0x27; //securiy SID
	 								UnlockRq_TxData[3] = 0x02; //security key - level 1

	 								for(index_array=0;index_array<4;index_array++)
	 								{
	 									UnlockRq_TxData[index_array+4] = key[index_array];
	 								}

	 								if(HAL_CAN_AddTxMessage(&hcan,&Tester_TxHeader,UnlockRq_TxData,&Tester_TxMailbox) != HAL_OK)
	 								{
	 									error_flag = 1;
	 								} else {
	 									error_flag = 0;
	 									send_firstframe_flag = 1; // co bao hieu da gui first frame
	 								}
	 								break;
	 							}
	 							case 1: //first frame has been sent
	 							{
	 								if(flag_read_response) //recieve flow control
	 								{
	 									multiflow_state = FlOWCONTROL_CHECK;
	 									flag_read_response = 0;
	 									send_firstframe_flag = 0;
	 								}
	 								break;
	 							}
	 						}
	 						break;
	 					}
	 					case FlOWCONTROL_CHECK:
	 					{
	 						FT_String = SF_N_PCI_FrameTypeHandle(Tester_RxData[0]);
	 						FrameType = GetFrameType(FT_String);
	 						//check flow control response data
	 						//phan loai frame
	 						if(FrameType == 3) //Frametype = 3 -> Frame nhan la Frame flow control tu Tester
	 						{
	 							//kiem tra block size
	 							//- ignore
	 							//kiem tra separation time menu
	 							//- ignore
	 							//kiem tra flow state
	 							switch(Tester_RxData[0] & 0x0f)
	 							{
	 							 	case CONTINUE_TO_SEND: //0x0
	 							 		multiflow_state = CONSECUTIVEFRAME_SEND;
	 							 	break;
	 							 	case WAIT: //0x1
	 							 	break;
	 							 	case OVERFLOW: //0x2
	 							 	break;
	 							 }

	 						 } else //neu nhan duoc bat kỳ frame nao khac ngoai Flow Control
	 						 {
	 							 //restart process - plan
	 						 }
	 						break;
	 					}
	 					case CONSECUTIVEFRAME_SEND:
	 					{
	 						//start transmitting remain data
	 						while(1){
	 						if(MaxConsecutiveFrame_count < sequence_num) 	//done transmitting - wait for ecu response
	 						{
	 							security_state = AUTHENTICATE_RESPONSE;
	 							sequence_num = 1;
	 							break;
	 						}
	 						KeySend_TxData[0] = FT_CONSECUTIVEFRAME + sequence_num; //0x20 FT Consecutive Frame
	 						for(index_array = prev_index_array;index_array<prev_index_array+7;index_array++)
	 						{
	 							if(index_array>= 16) {
	 								KeySend_TxData[3] = 0x00;
	 								KeySend_TxData[4] = 0x00;
	 								KeySend_TxData[5] = 0x00;
	 								KeySend_TxData[6] = 0x00;
	 								KeySend_TxData[7] = 0x00;
	 								break;
	 							} else if(index_array<16)
	 							{
	 								KeySend_TxData[(index_array-prev_index_array)+1] = key[index_array];
	 							}
	 						}
	 						prev_index_array = index_array;
	 						//start transmit CF#n. n - sequence_num
	 						if(HAL_CAN_AddTxMessage(&hcan,&Tester_TxHeader,KeySend_TxData,&Tester_TxMailbox) != HAL_OK)
	 						{
	 							error_flag = 1;
	 						} else {
	 								error_flag = 0;
	 								sequence_num++;
	 						}

	 						HAL_Delay(1500); //500ms send once CF#n
	 						}
	 						break;
	 					}
	 				}
	 				break; //break for SEND KEY
	 			}
	 			case AUTHENTICATE_RESPONSE:
	 			{
	 				//wait ecu response
	 				if(flag_read_response==1)
	 				{
	 					FT_String = SF_N_PCI_FrameTypeHandle(Tester_RxData[0]);
	 					FrameType = GetFrameType(FT_String);
	 					//check frame type
	 					if(FrameType == 0) //normal single frame
	 					{
	 						//check what kind of response
	 						if(Tester_RxData[1]==(0x27+0x40)) //positive
	 						{
	 							security_flag = 1;
	 							tester_state = REQUEST_WRITE;
	 						} else if(Tester_RxData[1]==0x7F) //negative with invalid key error
	 						{
	 							security_flag = 0;
	 							//back to init state
	 							tester_state = INIT_STATE;
	 						}
	 					}
	 					ReadSingleFrame_handle(); //for debugging purpose

	 					flag_read_response = 0;
	 				}
	 				break;
	 			}
	 		}

	 		//sau khi nhan duoc du lieu accept security access tien hanh
	 		//quay ve trang thai write
	 		//dong thoi bat den - kich hoat flag o timer
	 		//
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Tester_SecA_LED_GPIO_Port, Tester_SecA_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Tester_SecA_LED_Pin */
  GPIO_InitStruct.Pin = Tester_SecA_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Tester_SecA_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //!WARNING: Don't change
{
	if(htim == &htim4) //behavior led function timer
	{
		if(security_flag == 1)
		{
			//turn on led
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);

			if(timer_cnt++==5000)
			{
				//turn off led
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
				security_flag = 0;
				//reset timer
				timer_cnt = 0; //reset timer
			}
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
