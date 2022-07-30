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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "dumo_bglib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AMOUNT_TO_SEND (1 * (250*4 * 50*4) )
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BGLIB_DEFINE();

char tmpbaud[10];
/** Variable for storing function return values. */
int ret;
/** Pointer to wifi packet*/
struct dumo_cmd_packet* pck;
/** Buffer for storing data from the serial port. */
static unsigned char buffer[BGLIB_MSG_MAXLEN];
/** Length of message payload data. */
uint16_t msg_length;

unsigned int amount_to_send;
uint8_t ep;
uint32_t frequency;        // ticks per second
uint32_t t1, t2;           // ticks
double elapsedTime;

uint8_t UartTxClp = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Function called when a message needs to be written to the serial port.
 * @param msg_len Length of the message.
 * @param msg_data Message data, including the header.
 * @param data_len Optional variable data length.
 * @param data Optional variable data.
 */
uint8_t dataframe[300];
uint8_t length;
static void on_message_send(uint8 msg_len, uint8* msg_data, uint16 data_len, uint8* data)
{
    /*1- polling可以使用*/
//    HAL_UART_Transmit(&huart1, msg_data, msg_len, 0xFF);
//    if(data_len && data)
//    {
//        HAL_UART_Transmit(&huart1, data, data_len, 0xFF);
//    }
    /*2- polling 合并可以使用*/
//    for (int i = 0; i < msg_len; i++) {
//        dataframe[i] = *msg_data++;
//    }
//    if(data_len && data)
//    {
//        for (int i = 0; i < data_len; i++) {
//            dataframe[msg_len + i] = *data++;
//        }
//        HAL_UART_Transmit(&huart1, dataframe, msg_len + data_len, 0xFF);
//    }
//    else{
//        HAL_UART_Transmit(&huart1, dataframe, msg_len , 0xFF);
//    }

    /*3- DMA 合并可以使用*/
    for (int i = 0; i < msg_len; i++) {
        dataframe[i] = *msg_data++;
    }
    if(data_len && data)
    {
        for (int i = 0; i < data_len; i++) {
            dataframe[msg_len + i] = *data++;
        }
        if(UartTxClp){
            UartTxClp = 0;
            HAL_UART_Transmit_DMA(&huart1, dataframe, msg_len + data_len);
        }
    }
    else{
        if(UartTxClp){
            UartTxClp = 0;
            HAL_UART_Transmit_DMA(&huart1, dataframe, msg_len);
        }
    }

}
// Function for printing out the Bluetooth address **initially used for debugging purposes
void print_address(uint8_t address[6]){
    printf("%02x:%02x:%02x:%02x:%02x:%02x", address[5],address[4],address[3],address[2],address[1],address[0]);
}
/**
* Send one frame of data
*/
uint8_t send_index;
void send_one_frame(uint8_t ep, unsigned int amount, unsigned int counter)
{
    uint8_t frame[256];

    if (amount > 250) return; // max frame size

//    frame[0] = send_index++;
    for (unsigned int i = 0; i < amount; i++){
        frame[i] = (uint8_t) i;
    }
    dumo_cmd_endpoint_send(ep, amount, (void *) frame);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /*注册重定向的串口�???*/
  RetargetInit(&huart2);
  BGLIB_INITIALIZE(on_message_send);

    dumo_cmd_system_hello();

    unsigned int data_counter = 0;
    int flag_full = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /**
   * Read enough data from serial port for the BGAPI message header.
   */
      HAL_UART_Receive(&huart1, buffer, BGLIB_MSG_HEADER_LEN, 0xFF);
      /**
       * The buffer now contains the message header. See BGAPI protocol definition for details on packet format.
       */
      msg_length = BGLIB_MSG_LEN(buffer);
      /**
       * Read the payload data if required and store it after the header.
       */
      if(msg_length)
      {
          HAL_UART_Receive(&huart1, &buffer[BGLIB_MSG_HEADER_LEN], msg_length, 0xFF);
      }

      pck= BGLIB_MSG(buffer);

      unsigned int frame_size = 250;

      switch(BGLIB_MSG_ID(buffer))
      {

          case dumo_rsp_system_hello_id:
              printf("Hello from BG\r\n");
              dumo_cmd_system_reset(0);
              break;
          case dumo_evt_system_boot_id:
              printf("Rebooting\r\n");
              break;
          case dumo_evt_system_initialized_id:
              printf("Initializing System address: ");
              print_address(pck->evt_system_initialized.address.addr);
              printf("\r\n");

              /*start rfcomm server, use id 2 for sdp entry*/
              dumo_cmd_bt_rfcomm_start_server(2,0);
              break;
          case dumo_rsp_bt_rfcomm_start_server_id:
              if (pck->rsp_bt_rfcomm_start_server.result == 0)
              {
                  printf("SPP server started; ");
                  printf("Setting discoverable, connectable & bondable on\n");
                  /*Set Bluetooth BR/EDR mode to connectable*/
                  dumo_cmd_bt_gap_set_mode(1, 1, 0);
              }
              else
              {
                  printf("SPP server start failed, errorcode %d\n", pck->rsp_bt_rfcomm_start_server.result);
              }
              break;
          case dumo_rsp_bt_gap_set_mode_id:
              if (pck->rsp_bt_gap_set_mode.result == 0)
              {
                  /*Set bondable mode*/
                  printf("Set bondable mode\n");
                  dumo_cmd_sm_set_bondable_mode(1);
              }
              break;
          case dumo_rsp_sm_set_bondable_mode_id:
              if (pck->rsp_sm_set_bondable_mode.result == 0)
              {
                  printf("Waiting for connection...\n");
              }
              break;
          case dumo_evt_sm_bonded_id:
              if (pck->evt_sm_bonded.bonding != 0xff) dumo_cmd_sm_read_bonding(pck->evt_sm_bonded.bonding);
              break;
          case dumo_rsp_sm_read_bonding_id:
              printf("Bonding from ");
              print_address(pck->rsp_sm_read_bonding.address.addr);
              printf(" succesful\n");
              break;
          case dumo_evt_bt_rfcomm_opened_id:
              printf("SPP connection from ");
              print_address(pck->evt_bt_rfcomm_opened.address.addr);
              printf(" at endpoint %d\n",pck->evt_bt_rfcomm_opened.endpoint);
              ep = pck->evt_bt_rfcomm_opened.endpoint;
              amount_to_send = AMOUNT_TO_SEND;
              // start timer
              printf("BT CONNECT\r\n");
              elapsedTime = HAL_GetTick();
              send_one_frame(ep, frame_size, data_counter);
              break;
          case dumo_rsp_endpoint_send_id:
              if (pck->rsp_endpoint_send.result == 0)
              {
                  amount_to_send -= frame_size;
                  data_counter += frame_size;
//                  printf(".");
//                  if (amount_to_send % 10000 == 0) printf("Amount left %d\n", amount_to_send);
                  if (amount_to_send)
                  {
                      if (frame_size > amount_to_send) frame_size = amount_to_send;
                      send_one_frame(ep, frame_size, data_counter);
                  }
                  else
                  {
                      // compute elapsed time in millisec
                      elapsedTime = HAL_GetTick()-elapsedTime;
                      double speed ;
                      speed = (double) AMOUNT_TO_SEND / elapsedTime *1000.;
                      printf("Sending Speed  %7.0f bytes/second\n", speed);
                      /* Wait for data to finish transfering over the BT link before closing connection */
//                      HAL_Delay(500);
//                      dumo_cmd_endpoint_close(ep);
                      //start timer again
                      amount_to_send = AMOUNT_TO_SEND;
                      frame_size = 250;
                      elapsedTime = HAL_GetTick();
                      send_one_frame(ep, frame_size, data_counter);

                  }
              }
              else if (pck->rsp_endpoint_send.result == dumo_err_buffers_full)
              {
                  /* BT121 buffers are full -> wait for endpoint status that indicates there's free space */
                  flag_full = 1;
                  printf("P");
              }
              else printf("err %d\n", pck->rsp_endpoint_send.result);
              break;
          case dumo_evt_endpoint_status_id:
              if (flag_full && !(pck->evt_endpoint_status.flags & ENDPOINT_FLAG_FULL))
              {
                  /* Free space in endpoint -> continue transfer */
                  printf("C");
                  flag_full = 0;
                  if (amount_to_send)
                  {
                      if (frame_size > amount_to_send) frame_size = amount_to_send;
                      send_one_frame(ep, frame_size, data_counter);
                  }
              }
              else if (flag_full && (pck->evt_endpoint_status.flags & ENDPOINT_FLAG_FULL))
              {
                  // Endpoint full, do nothing
                  printf("F");
              }
              else
              {
                  printf("Endpoint status:%x %x\n", pck->evt_endpoint_status.flags, flag_full);
              }
              break;
          case dumo_evt_endpoint_data_id:
              printf("Received data from endpoint %d ", pck->evt_endpoint_data.endpoint);
              printf("\"");
              {
                  int n;
                  char *c = pck->evt_endpoint_data.data.data;
                  for (n = 0; n < pck->evt_endpoint_data.data.len;n++)
                  {
                      printf("%c",*c++);
                  }
              }
              printf("\"\n");
              /*Echo*/
              {
                  char tmp[100];
                  sprintf(tmp, "Received %d bytes\n", pck->evt_endpoint_data.data.len);
                  dumo_cmd_endpoint_send(pck->evt_endpoint_data.endpoint, strlen(tmp), tmp);
              }

              break;
          case dumo_evt_endpoint_closing_id:
              printf("Closing endpoint:%d\n", pck->evt_endpoint_closing.endpoint);
              printf("BT DISCONNECT\r\n");
              /*endpoint is closing, could check for reason here and then close it*/
              dumo_cmd_endpoint_close(pck->evt_endpoint_closing.endpoint);
              break;
          case dumo_rsp_endpoint_close_id:
              if (pck->rsp_endpoint_close.result == 0)
              {
                  /* Finished */
                  printf("lzx:endpoint_closed\n");
              }
              break;
      }
      memset(&buffer, 0 , sizeof(buffer));


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
    //先将时钟源选择为内部时钟
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Set transmission flag: transfer complete*/
    if(huart->Instance == USART1) {
        UartTxClp = 1;
    }

}


////先将时钟源选择为内部时钟
//RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
//RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//{
//      Error_Handler();
//}
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
