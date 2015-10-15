/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Robert Campo
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   This is a modification of the original HID example provided 
  *          by ST in version 2.1.0 of their driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "application.h"
#include "usbh_hid_core.h"
#include "usbh_usr.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"
#include "usb_hal.h"

int led2 = D7;
TCPClient client;

#define KYBRD_FIRST_COLUMN               (uint16_t)319
#define KYBRD_LAST_COLUMN                (uint16_t)7
#define KYBRD_FIRST_LINE                 (uint8_t)120
#define KYBRD_LAST_LINE                  (uint8_t)200

//using TCP client for debugging (that's all I got!)
byte server[] = { 192, 168, 1, 120 };
uint8_t  KeybrdCharXpos           = 0;
uint16_t KeybrdCharYpos           = 0;
extern  int16_t  x_loc, y_loc; 
extern __IO int16_t  prev_x, prev_y;

__IO uint32_t i = 0;
char bufferHeader[512];


void setup() {
  
    Serial1DebugOutput debugOutput(9600, LOG_LEVEL);
	//pinMode(led2, OUTPUT);
  
	client.connect(server, 8001);
	client.read();
  
	//using existing Serial object (for now)
	//initialize usb host with callbacks
	//this never completes IF a USB device is connected upon startup
	Serial.Initialize();
}

int loopCheck = 0;

void loop() {
	
	if(loopCheck == 0)
	{
		client.println("Starting loop");
		loopCheck = 1;
	}

	//call process to check for changes in the host controller
	Serial.Process();
}



#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
  number,ex: printf("Wrong parameters value: file %s on line %d\r\n", 
  file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}

#endif


/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
