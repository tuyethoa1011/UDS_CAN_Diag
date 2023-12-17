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

//typedef enum
//{
//	READ_SINGLEFRAME, //read seed
//	READ_FIRSTFRAME, //read KEY supported
//	READ_CONSECUTIVEFRAME, //read KEY supported
//	READ_FLOWCONTROL,
//}Frame_Status;

typedef enum
{
	SEND_SEED,
	CALCULATE_ECU_KEY,
	GET_TESTERKEY,
	AUTHENTICATE_TESTERKEY,
}Security_Status;

typedef enum
{
	LEVEL1,
}SecLevel;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Diagnostic Message Format*/
#define FT_SINGLEFRAME 0x00
#define FT_FIRSTFRAME 0x10
#define FT_CONSECUTIVEFRAME 0x20
#define FT_FLOWCONTROL 0x30
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

/*Define security level*/
#define SEC_LEVEL1 0x01

/*Define Data Length*/
#define KEY_LENGTH 16
/*Define Data Length*/

/*Valid key definition*/
#define PASS_KEY 0x1
#define INVALID_KEY 0x0
 /**/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef ECU_TxHeader;
CAN_RxHeaderTypeDef ECU_RxHeader;

uint32_t ECU_TxMailbox;

uint8_t PositiveReadRsp_TxData[8];
uint8_t NegativeReadRsp_TxData[8] = {0x03,0x7F,0x22,0x13,0x00,0x00,0x00,0x00};
uint8_t SecuritySeed_TxData[8];
uint8_t SecurityFlowControl_TxData[8] = {0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t SecurityNegativeResponse_TxData[8] = {0x03,0x7F,0x27,0x13,0x00,0x00,0x00,0x00};
uint8_t SecurityPositiveResponse_TxData[8] = {0x02,0x67,0x02,0x00,0x00,0x00,0x00,0x00};

uint8_t WritePositiveResponse_TxData[8] = {0x01,0x6E,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t WriteNegativeResponse_TxData[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t ECU_RxData[8],Tx_Buffer[55];

uint8_t DataBuffer[sizeof(TEST_LABEL)] = TEST_LABEL; //buffer link to 0x0123 DID

uint8_t FT_String, DL_String;

uint8_t FrameType, DataLength;
uint8_t SecurityLevel;

uint16_t DID_Val;
uint8_t key_level;

uint8_t index_array;

uint8_t error_flag = 0,  ReadRq_flag = 0, security_flag = 0;
uint8_t valid_key = 0;
uint8_t DID_available_flag = 0;
uint16_t timer_cnt = 0;

uint8_t seed[4];
uint8_t key[16];
//uint8_t check_key[16];
uint8_t count_checkkey_fc = 0; //count check key -> send flow control
uint8_t tester_key[16];
uint8_t prev_index_array;
uint8_t count_cf_read=1;

ECU_Status ecu_state;
Security_Status security_state = 0;
//Frame_Status frame_recv_state;
SecLevel level;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t SF_N_PCI_FrameTypeHandle(uint8_t byteString);
uint8_t SF_N_PCI_DataLenngthHandle(uint8_t byteString);
uint8_t GetFrameType(uint8_t FT_byteString);
uint8_t GetDataLength(uint8_t DL_byteString);
uint8_t GetSecurityLevel(uint8_t LV_byteSring, uint8_t *keyLV_byteString);
uint8_t Check_ReadRq_Validation(uint8_t FT, uint8_t DL, uint8_t Data_buf[]);
uint8_t GetCANFrameSize(uint8_t aData[]);
uint16_t GetDID(uint8_t DID_HByteString,uint8_t DID_LByteString);
void ReadRequest_handle(void);
void GetDIDValueFromRecordTableOfDID(uint16_t DID_Val,uint8_t *buf_arr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t GetSecurityLevel(uint8_t LV_byteSring, uint8_t *keyLV_byteString)
{
	uint8_t SecLevel_res;
	uint8_t key_lv;
	switch(LV_byteSring)
	{
		case SEC_LEVEL1:
		{
			key_lv = 0x02;
			keyLV_byteString = &key_lv;
			SecLevel_res = 0;
			break;
		}
	}
	return SecLevel_res;
}
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
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &ECU_RxHeader, ECU_RxData) != HAL_OK)
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
	FT_String = SF_N_PCI_FrameTypeHandle(ECU_RxData[0]);
	DL_String = SF_N_PCI_DataLenngthHandle(ECU_RxData[0]);

	FrameType = GetFrameType(FT_String);
	DataLength = GetDataLength(DL_String);

	if(Check_ReadRq_Validation(FrameType,DataLength, ECU_RxData) != POSITIVE_RESPONSE)
	{
		//transmit negative response CAN frame packet
		HAL_CAN_AddTxMessage(&hcan, &ECU_TxHeader,NegativeReadRsp_TxData,&ECU_TxMailbox);

	} else { //positive response
		DID_Val = GetDID(ECU_RxData[2], ECU_RxData[3]);

		//uint8_t *DataBuffer = (uint8_t*)malloc(sizeof(uint8_t));
		//GetDIDValueFromRecordTableOfDID(DID_Val,DataBuffer);

		switch(DID_Val)
		{
			case 0x0123:
				if(sizeof(DataBuffer)<8){
					PositiveReadRsp_TxData[0] = ECU_RxData[1] + 0x40;
					PositiveReadRsp_TxData[1] = ECU_RxData[2];
					PositiveReadRsp_TxData[2] = ECU_RxData[3];

					PositiveReadRsp_TxData[3] = FT_SINGLEFRAME + sizeof(DataBuffer); //du lieu data co do dai la 4 - TEST
					for (index_array = 0;index_array<sizeof(DataBuffer);index_array++)
					{
						PositiveReadRsp_TxData[index_array+4] = DataBuffer[index_array];
					}
					//gui single frame - chua toan bo du lieu
					if (HAL_CAN_AddTxMessage(&hcan, &ECU_TxHeader,PositiveReadRsp_TxData,&ECU_TxMailbox) != HAL_OK)
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);

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
   ECU_TxHeader.DLC = 8;
   ECU_TxHeader.ExtId = 0;
   ECU_TxHeader.IDE = CAN_ID_STD;
   ECU_TxHeader.RTR = CAN_RTR_DATA;
   ECU_TxHeader.StdId = 0x7A2;
   ECU_TxHeader.TransmitGlobalTime = DISABLE;

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
	 			 switch(ECU_RxData[1])
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
//	 			 			 //do nothing because there is no definition for this - ignore
//	 			 			 //solution 1 - just go back to idle state
//	 			 			 //solution 2 - Security Access then write
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
	 			ReadRq_flag = 0;
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
	 		 //receive write request
	 		 if(ReadRq_flag == 1)
	 		 {
	 			 if(FrameType == 0)
	 			 {
	 				DID_Val = GetDID(ECU_RxData[2], ECU_RxData[3]);

	 				switch(DID_Val)
	 				{
	 					case 0x0123:
	 					{
	 						DID_available_flag = 1;
	 						break;
	 					}
	 				}

	 				if(DID_available_flag == 1)
	 				{
	 					//check data length >= 4 bytes
	 					DL_String = SF_N_PCI_DataLenngthHandle(ECU_RxData[0]);
	 					DataLength = GetDataLength(DL_String);

	 					if(DataLength>=4)
	 					{
	 						//write to data buffer
	 						for(index_array = 0;index_array<4;index_array++)
	 						{
	 							DataBuffer[index_array] = ECU_RxData[index_array+4];
	 						}
	 						//send positive response
	 						if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,WritePositiveResponse_TxData,&ECU_TxMailbox)!=HAL_OK)
	 						{
	 							error_flag = 1;
	 						} else {
	 							error_flag = 0;
	 						}

	 					} else {
	 						//send negative response for data length

	 						WriteNegativeResponse_TxData[0] = 0x7F;
	 						WriteNegativeResponse_TxData[1] = 0x2E;
	 						WriteNegativeResponse_TxData[2] = 0x13;

	 						if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,WriteNegativeResponse_TxData,&ECU_TxMailbox)!=HAL_OK)
	 						{
	 							error_flag = 1;
	 						} else {
	 							error_flag = 0;
	 						}
	 					}

	 					DID_available_flag = 0;
	 					} else { //else no DID
	 						//send negative response - DID is not supported

	 						WriteNegativeResponse_TxData[0] = 0x7F;
	 						WriteNegativeResponse_TxData[1] = 0x2E;
	 						WriteNegativeResponse_TxData[2] = 0x31;

	 						if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,WriteNegativeResponse_TxData,&ECU_TxMailbox)!=HAL_OK)
	 						{
	 							error_flag = 1;
	 						} else {
	 							error_flag = 0;
	 						}
	 				}
	 			 }
	 			 ReadRq_flag = 0;
	 		 }
	 		 ecu_state = IDLE_STATE;
	 		 break;
	 	 }
	 	 case SECURITY_ACCESS_STATE:
	 	 {
	 		 switch(security_state)
	 		 {
	 		 	 case SEND_SEED: //truoc do da nhan duoc goi request security tu tester
	 		 	 {
	 		 		SecurityLevel = GetSecurityLevel(ECU_RxData[2],&key_level);
	 		 		//mac dinh dang xu ly cho Security level 1
	 		 		switch(SecurityLevel)
	 		 		{
	 		 			case LEVEL1:
	 		 			{
	 		 				//format Seed tx buffer
	 		 				SecuritySeed_TxData[0] = 0x06;
	 		 			 	SecuritySeed_TxData[1] = ECU_RxData[1] + 40;
	 		 			 	SecuritySeed_TxData[2] = ECU_RxData[2];
	 		 			 	//format seed before sending
	 		 			 	//random generate seed
	 		 			 	seed[0] = rand()%255;
	 		 			 	seed[1] = rand()%255;
	 		 			 	seed[2] = rand()%255;
	 		 			 	seed[3] = rand()%255;
	 		 			 	//add to seed transmit data
	 		 			 	SecuritySeed_TxData[3] = seed[0];
	 		 			 	SecuritySeed_TxData[4] = seed[1];
	 		 			 	SecuritySeed_TxData[5] = seed[2];
	 		 			 	SecuritySeed_TxData[6] = seed[3];
	 		 			 	//restric random bytes while transmiting
	 		 			 	SecuritySeed_TxData[7] = 0x00;
	 		 			 	SecuritySeed_TxData[8] = 0x00;
	 		 			 	//response seed to Tester
	 		 			 	if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,SecuritySeed_TxData,&ECU_TxMailbox) != HAL_OK)
	 		 			 	{
	 		 			 		error_flag = 1;
	 		 			 	} else {
	 		 			 		error_flag = 0;
	 		 			 	}
	 		 				break;
	 		 			 }
	 		 		}
	 		 		ReadRq_flag = 0;
	 		 		break;
	 		 	   }
	 		 	 case CALCULATE_ECU_KEY:
	 		 	 {
	 		 		if(ReadRq_flag == 1)
	 		 		{
	 		 			FT_String = SF_N_PCI_FrameTypeHandle(ECU_RxData[0]);
	 		 			FrameType = GetFrameType(FT_String);

	 		 			 if(FrameType==1) //first frame detected
	 		 			 {
	 		 			 	//mac dinh dang xu ly cho Security level 1
	 		 				 switch(SecurityLevel)
	 		 				 {
	 		 				 	 case LEVEL1:
	 		 				 	 {
	 		 				 		 if(key_level == ECU_RxData[3])
	 		 				 		 {
	 		 				 			//self calculate key
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
	 		 				 		 }
	 		 				 		 break;
	 		 				 	 }
	 		 				 }
	 		 				}
	 		 				//check key0 - key 3 dung thi gui ve flow control gui tiep
	 		 				tester_key[0] = ECU_RxData[4];
	 		 				tester_key[1] = ECU_RxData[5];
	 		 				tester_key[2] = ECU_RxData[6];
	 		 				tester_key[3] = ECU_RxData[7];
	 		 				//for roi so sanh - se co bien count corrrect -  count correct == 4 -> send flow control
	 		 				for(index_array = 0;index_array<4;index_array++)
	 		 				{
	 		 					if(tester_key[index_array] == key[index_array])
	 		 					{
	 		 					 	count_checkkey_fc++;
	 		 					 }
	 		 				}

	 		 				if(count_checkkey_fc == 4 && ECU_RxData[1]==KEY_LENGTH)//4 key corrrect send back flow control
	 		 				{
	 		 					if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,SecurityFlowControl_TxData,&ECU_TxMailbox)!=HAL_OK)
	 		 					{
	 		 					 	error_flag = 1;
	 		 					}
	 		 					else {
	 		 					 	error_flag = 0;
	 		 					 	security_state = AUTHENTICATE_TESTERKEY;
	 		 					}
	 		 				}
	 		 		}
	 		 		ReadRq_flag = 0;
	 		 		break;
	 		 	 }
	 		 	 case GET_TESTERKEY:
	 		 	 {
	 		 		 //handle consecutive frame recieve - nhan du so frame can nhan thi tien hanh bat co cho phep request write
	 		 		 //step1: get full key from tester
	 		 		 uint8_t max_cf_read = KEY_LENGTH/7+1;
	 		 		 //read = 1 -> check frametype -> count += 1
	 		 		 if(ReadRq_flag==1)
	 		 		 {
	 		 			 if(FrameType==2) //consecutive frame detected
	 		 			 {
	 		 				 if(count_cf_read<=max_cf_read)
	 		 				 {
	 		 					 for(index_array=prev_index_array;index_array<prev_index_array+7;index_array++)
	 		 					 {
	 		 						 tester_key[index_array] = ECU_RxData[(index_array-prev_index_array)+1];
	 		 					 }
	 		 					 prev_index_array = index_array;
	 		 					 count_cf_read++;
	 		 				 }else{
	 		 					 security_state = AUTHENTICATE_TESTERKEY;
	 		 				 }
	 		 			 }
	 		 		 }
	 		 		 //step2: change state to authenticate
	 		 		 break;
	 		 	 }
	 		 	 case AUTHENTICATE_TESTERKEY:
	 		 	 {
	 		 		 //valid key
	 		 		 for(index_array=0;index_array<KEY_LENGTH;index_array++)
	 		 		 {
	 		 			 if(tester_key[index_array]==key[index_array])
	 		 			 {
	 		 				 valid_key = 1;
	 		 			 } else if(tester_key[index_array]!=key[index_array])
	 		 			 {
	 		 				 valid_key = 0;
	 		 				 break;
	 		 			 }
	 		 		 }

	 		 		 switch(valid_key) //valid key state
	 		 		 {
	 		 		 	 case PASS_KEY:
	 		 		 	 {
	 		 		 		 //send positive response to let tester write data to ecu
	 		 		 		 if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,SecurityPositiveResponse_TxData,&ECU_TxMailbox)!=HAL_OK)
	 		 		 		 {
	 		 		 			 error_flag = 1;
	 		 		 		 } else
	 		 		 		 {
	 		 		 			 error_flag = 0;
	 		 		 			 //key pass, turn on security flag
	 		 		 			 security_flag = 1;
	 		 		 			 ecu_state = IDLE_STATE;
	 		 		 		 }
	 		 		 		 break;
	 		 		 	 }
	 		 		 	 case INVALID_KEY:
	 		 		 	 {
	 		 		 		 //send negative response with invalid key error code
	 		 		 		if(HAL_CAN_AddTxMessage(&hcan,&ECU_TxHeader,SecurityNegativeResponse_TxData,&ECU_TxMailbox)!=HAL_OK)
	 		 		 		{
	 		 		 			error_flag = 1;
	 		 		 		} else
	 		 		 		{
	 		 		 			error_flag = 0;
	 		 		 			security_flag = 0;
	 		 		 			ecu_state = IDLE_STATE;
	 		 		 		}
	 		 		 		 break;
	 		 		 	 }
	 		 		 }
	 		 		 break;
	 		 	 }

	 		 }
	 		 break; //break security access state
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
  htim4.Init.Prescaler = 3599;
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
  HAL_GPIO_WritePin(ECU_SecA_LED_GPIO_Port, ECU_SecA_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ECU_SecA_LED_Pin */
  GPIO_InitStruct.Pin = ECU_SecA_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ECU_SecA_LED_GPIO_Port, &GPIO_InitStruct);

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
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
			if(timer_cnt++==5000)
			{
				//turn off led
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_RESET);
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
