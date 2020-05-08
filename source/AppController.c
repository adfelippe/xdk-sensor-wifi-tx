/* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for application development by Licensee.
* Fitness and suitability of the example code for any use within application developed by Licensee need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/

/**
 * @ingroup APPS_LIST
 *
 * @defgroup SEND_ACCELEROMETER_DATA_OVER_UDP_AND_BLE SendAccelDataOverUdpandBle
 * @{
 *
 * @brief Demo application of Transmitting BMA280 Accelerometer data on BLE(Bluetooth Low Energy) and UDP every configured interval (#APP_CONTROLLER_TX_DELAY)
 *
 * @details This example demonstrates how to read sensor values from the BMA280 Acceleration sensor and streams them over Bluetooth Low Energy via custom Bi-Directional Service.<br>
 * Either use your Android or iOS mobile phone (see [Android](https://play.google.com/store/apps/details?id=com.macdom.ble.blescanner&hl=en) or
 * [iOS](https://itunes.apple.com/in/app/lightblue-explorer-bluetooth-low-energy/id557428110?mt=8) App Store) to connect to XDK and receive the data.
 * Send command <b>start (HEX: 0x7374617274)</b>  to XDK via Bluetooth Low Energy, so that streaming of data begins.
 * To stop the streaming send command <b>end (HEX: 656e64)</b>
 *
 * This Application enables the bi-directional service in ble and sends Accelerometer Data over ble and UDP. <br>
 * <b> Bi-Directional Service : </b>
 *
 *  Service             |  Characteristic                |         Attribute-Type             |               UUID                   |
 * ---------------------|--------------------------------|------------------------------------|--------------------------------------|
 * BidirectionalService |    NA                          |         uint8_t                    | b9e875c0-1cfa-11e6-b797-0002a5d5c51b |
 * NA                   |    Rx                          |         uint8_t X[20]              | 0c68d100-266f-11e6-b388-0002a5d5c51b |
 * NA                   |    Tx                          |         uint8_t X[20]              | 1ed9e2c0-266f-11e6-850b-0002a5d5c51b |
 *
 * @file
 **/

/* module includes ********************************************************** */

/* own header files */
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* own header files */
#include "AppController.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "XDK_UDP.h"
#include "XDK_WLAN.h"
#include "XDK_BLE.h"
#include "XDK_Sensor.h"
#include "XDK_Utils.h"
#include "BCDS_BSP_Board.h"
#include "BCDS_WlanNetworkConfig.h"
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "BCDS_Assert.h"
#include "XDK_LED.h"
#include "XDK_Button.h"
#include "em_gpio.h"

/* constant definitions ***************************************************** */
static void Button2Callback(ButtonEvent_T buttonEvent);
static bool button2Pressed = false;

/* local variables ********************************************************** */
static WLAN_Setup_T WLANSetupInfo =
        {
                .IsEnterprise = false,
                .IsHostPgmEnabled = false,
                .SSID = WLAN_SSID,
                .Username = WLAN_PSK,
                .Password = WLAN_PSK,
                .IsStatic = WLAN_STATIC_IP,
                .IpAddr = WLAN_IP_ADDR,
                .GwAddr = WLAN_GW_ADDR,
                .DnsAddr = WLAN_DNS_ADDR,
                .Mask = WLAN_MASK,
        };/**< WLAN setup parameters */

static Button_Setup_T ButtonSetup =
        {
                .CmdProcessorHandle = NULL,
                .InternalButton1isEnabled = false,
                .InternalButton2isEnabled = true,
                //.InternalButton1Callback = Button1Callback,
                .InternalButton2Callback = Button2Callback,
        };/**< Button setup parameters */

/**
 * @brief This is the BLE data receive callback function
 *
 * @param[in]   rxBuffer
 * pointer to the received data buffer
 *
 * @param[in]   rxDataLength
 * Length of the received data
 *
 * @param[in]   param
 * unused
 *
 * @note Do not perform any heavy processing within this function and return ASAP.
 */
static void AppControllerBleDataRxCB(uint8_t *rxBuffer, uint8_t rxDataLength, void * param);
static BaseType_t shockConditionMet(Sensor_Value_T  *sensor);

static BLE_Setup_T BLESetupInfo =
        {
                .DeviceName = APP_CONTROLLER_BLE_DEVICE_NAME,
                .IsMacAddrConfigured = false,
                .MacAddr = 0UL,
                .Service = BLE_BCDS_BIDIRECTIONAL_SERVICE,
                .IsDeviceCharacteristicEnabled = false,
                .CharacteristicValue =
                        {
                                .ModelNumber = NULL,
                                .Manufacturer = NULL,
                                .SoftwareRevision = NULL
                        },
                .DataRxCB = AppControllerBleDataRxCB,
                .CustomServiceRegistryCB = NULL,
        };/**< BLE setup parameters */

static Sensor_Setup_T SensorSetup =
        {
                .CmdProcessorHandle = NULL,
                .Enable =
                        {
                                .Accel = true,
                                .Mag = false,
                                .Gyro = true,
                                .Humidity = false,
                                .Temp = false,
                                .Pressure = false,
                                .Light = false,
                                .Noise = false,
                        },
                .Config =
                        {
                                .Accel =
                                        {
                                                .Type = SENSOR_ACCEL_BMA280,
                                                .IsRawData = false,
                                                .IsInteruptEnabled = false,
                                                .Callback = NULL,
                                        },
                                .Gyro =
                                        {
                                                .Type = SENSOR_GYRO_BMG160,
                                                .IsRawData = false,
                                        },
                                .Mag =
                                        {
                                                .IsRawData = false
                                        },
                                .Light =
                                        {
                                                .IsInteruptEnabled = false,
                                                .Callback = NULL,
                                        },
                                .Temp =
                                        {
                                                .OffsetCorrection = APP_TEMPERATURE_OFFSET_CORRECTION,
                                        },
                        },
        };/**< Sensor setup parameters */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */
static xTaskHandle UdpControllerHandle = NULL;
static xTaskHandle CctOutControllerHandle = NULL;

/* Accelerometer queue */
QueueHandle_t accelQueue = NULL;

static bool AppControllerBleTransmitPayload = false; /**< Boolean representing if BLE needs to be streamed. Validate the repeated start flag */

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief Callback for Button 2.
 *
 * @param[in]    buttonEvent
 * If it is BUTTON_EVENT_PRESSED, then Red and Orange LED's are turned ON
 * If it is BUTTON_EVENT_RELEASED, then Yellow LED is turned ON
 *
 */
static void Button2Callback(ButtonEvent_T buttonEvent)
{
    Retcode_T retcode = RETCODE_OK;

    switch (buttonEvent)
    {
    case BUTTON_EVENT_PRESSED:
    	button2Pressed = true;
        break;

    case BUTTON_EVENT_RELEASED:
    	button2Pressed = false;
        break;

    default:
        printf("Button2Callback : Unsolicited button event occurred for PB2 \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }

    if (RETCODE_OK != retcode)
        Retcode_RaiseError(retcode);
}

/**
 * @brief This will validate the WLAN network connectivity
 *
 * Currently, upon abrupt disconnection the event from the WiFi chip is not
 * propagated / notified to the application. Until then, we manually validate
 * prior to ever HTTP Rest Client POST/GET cycle to make sure that the possibility
 * of software break is reduced. We check if the IP is valid to validate the same.
 *
 * If there is no connectivity we will restart the node after 10 seconds.
 */
static void AppControllerValidateWLANConnectivity(void)
{
    Retcode_T retcode = RETCODE_OK;
    WlanNetworkConfig_IpStatus_T ipStatus = WLANNWCNF_IPSTATUS_IP_NOTAQRD;
    WlanNetworkConfig_IpSettings_T ipAddressOnGetStatus;

    ipStatus = WlanNetworkConfig_GetIpStatus();
    if (ipStatus == WLANNWCNF_IPSTATUS_IPV4_AQRD)
    {
        retcode = WlanNetworkConfig_GetIpSettings(&ipAddressOnGetStatus);
        if ((RETCODE_OK == retcode) && (UINT32_C(0) == (ipAddressOnGetStatus.ipV4)))
        {
            /* Our IP configuration is corrupted somehow in this case. No use in proceeding further. */
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NODE_IPV4_IS_CORRUPTED);
            LED_Off(LED_INBUILT_YELLOW);
            LED_On(LED_INBUILT_RED);
        }
        else
        {
        	LED_On(LED_INBUILT_YELLOW);
        }
    }
    else
    {
        /* Our network connection is lost. No use in proceeding further. */
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NODE_WLAN_CONNECTION_IS_LOST);
        LED_Off(LED_INBUILT_YELLOW);
        LED_On(LED_INBUILT_RED);
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        printf("AppControllerValidateWLANConnectivity : Resetting the device. Check if network is available. Node will do a soft reset in 10 seconds.\r\n\r\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
        BSP_Board_SoftReset();
        assert(false); /* Code must not reach here */
    }
}

/**
 * @brief   This will send start or stop message based on input parameter
 *
 *  @param [in]  param1
 *  Unused, Reserved for future use
 *
 *  @param [in]  param2
 *  Differentiates start and stop command.
 *  APP_CONTROLLER_BLE_START_PAYLOAD for start.
 *  APP_CONTROLLER_BLE_END_PAYLOAD for stop.
 */
static void AppControllerBleStartOrEndMsgSend(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);

    Retcode_T retcode = RETCODE_OK;

    if (param2 == APP_CONTROLLER_BLE_START_TRIGGER)
    {
        retcode = BLE_SendData(((uint8_t*) APP_CONTROLLER_BLE_START_PAYLOAD), ((uint8_t) sizeof(APP_CONTROLLER_BLE_START_PAYLOAD) - 1), NULL, APP_CONTROLLER_BLE_SEND_TIMEOUT_IN_MS);
    }
    else if (param2 == APP_CONTROLLER_BLE_END_TRIGGER)
    {
        retcode = BLE_SendData(((uint8_t*) APP_CONTROLLER_BLE_END_PAYLOAD), ((uint8_t) sizeof(APP_CONTROLLER_BLE_END_PAYLOAD) - 1), NULL, APP_CONTROLLER_BLE_SEND_TIMEOUT_IN_MS);
    }
    else
    {
        /* Do nothing */;
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

/**
 * @brief Callback function called on data reception over BLE
 *
 * @param [in]  rxBuffer : Buffer in which received data to be stored.
 *
 * @param [in]  rxDataLength : Length of received data.
 * @param [in]  param
 * Unused
 */
static void AppControllerBleDataRxCB(uint8_t *rxBuffer, uint8_t rxDataLength, void * param)
{
    BCDS_UNUSED(param);
    BCDS_UNUSED(rxDataLength);

    Retcode_T retcode = RETCODE_OK;
    uint8_t bleReceiveBuff[APP_CONTROLLER_BLE_RX_BUFFER_SIZE];
    memset(bleReceiveBuff, 0, sizeof(bleReceiveBuff));
    memcpy(bleReceiveBuff, rxBuffer, rxDataLength);
    /* validate received data */
    if ((0UL == strcmp((const char *) bleReceiveBuff, APP_CONTROLLER_BLE_START_ID)) &&
            (false == AppControllerBleTransmitPayload))
    {
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerBleStartOrEndMsgSend, NULL, APP_CONTROLLER_BLE_START_TRIGGER);
        AppControllerBleTransmitPayload = true;
    }
    else if ((0UL == strcmp((const char *) bleReceiveBuff, APP_CONTROLLER_BLE_END_ID)) &&
            (true == AppControllerBleTransmitPayload))
    {
        AppControllerBleTransmitPayload = false;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerBleStartOrEndMsgSend, NULL, APP_CONTROLLER_BLE_END_TRIGGER);
    }
    else
    {
        ;/* Do nothing since we are not interested in garbage */
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

static void ActivateCircuitOutput(void* pvParameters)
{
	BCDS_UNUSED(pvParameters);

	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		GPIO_PinOutSet(gpioPortA, 1);
		vTaskDelay(pdMS_TO_TICKS(SHOCK_ACTIVATION_DELAY));
		GPIO_PinOutClear(gpioPortA, 1);
	}
}

static void UdpController(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);
    Retcode_T retcode = RETCODE_OK;
    Retcode_T retcodeUdp = RETCODE_OK;
    int16_t handle = 0;
    uint8_t accelDataBuffer[APP_CONTROLLER_BLE_TX_LEN];
    retcodeUdp = UDP_Open(&handle);

    while(1)
    {
    	/* Wait on queue */
    	xQueueReceive(accelQueue, (uint8_t*)accelDataBuffer, portMAX_DELAY);
    	LED_On(LED_INBUILT_ORANGE);

		if (RETCODE_OK == retcode)
		{
			if (RETCODE_OK == retcodeUdp)
			{
				retcodeUdp = UDP_Send(handle, DEST_SERVER_IP, DEST_SERVER_PORT, (uint8_t *)accelDataBuffer,
									  (APP_CONTROLLER_BLE_TX_LEN * sizeof(uint8_t)));
			}
			/*if (RETCODE_OK == retcodeUdp)
			{
				retcodeUdp = UDP_Close(handle);
			}*/
			if (RETCODE_OK != retcodeUdp)
			{
				Retcode_RaiseError(retcodeUdp);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(UDP_CONTROLLER_TX_DELAY));
    }
}



/**
 * @brief Responsible for controlling the UDP Example application control flow.
 *
 * - Check whether the network connection is available
 * - Triggers a Sensor data sampling
 * - Read the sampled Sensor data
 * - Creates a socket for communication
 * - Sends the data to the remote server.
 * - Closes the socket after successful communication.
 * - Send sensor data via. BLE.
 * - Wait for APP_CONTROLLER_UDP_TX_DELAY before proceeding to redoing the above steps except the first
 *
 * @param[in] pvParameters
 * Unused
 */
static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);

    Retcode_T retcode = RETCODE_OK;
    Sensor_Value_T sensorValue;
    uint8_t accelDataUdpTxBuffer[APP_CONTROLLER_BLE_TX_LEN] = { 0 };

    memset(&sensorValue, 0x00, sizeof(sensorValue));

    // Initialise the xLastWakeTime variable with the current time.
    //xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* Resetting / clearing the necessary buffers / variables for re-use */
        retcode = RETCODE_OK;
        memset(accelDataUdpTxBuffer, 0U, sizeof(accelDataUdpTxBuffer));

        /* Check whether the WLAN network connection is available */
        AppControllerValidateWLANConnectivity();

        /* Read sensors from XDK API */
        bool button = button2Pressed;
        retcode = Sensor_GetData(&sensorValue);

        /* Check condition to apply shock to external circuit */
        if (shockConditionMet(&sensorValue) == pdTRUE)
        {
        	xTaskNotifyGive((xTaskHandle)CctOutControllerHandle);
        }

        /* Store values in the TX buffer */
        if (RETCODE_OK == retcode)
        {
            sprintf((char*) accelDataUdpTxBuffer, "%d | %ld, %ld, %ld, | %ld, %ld, %ld",
            		button, (long int) sensorValue.Accel.X, (long int) sensorValue.Accel.Y, (long int) sensorValue.Accel.Z,
					(long int) sensorValue.Gyro.X, (long int) sensorValue.Gyro.Y, (long int) sensorValue.Gyro.Z);
        }

        /* Send accelerometer data do queue to be sent via UDP */
        xQueueSend(accelQueue, (void*)accelDataUdpTxBuffer, portMAX_DELAY);

        if (RETCODE_OK != retcode)
        {
            Retcode_RaiseError(retcode);
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_COLLECTION_PERIOD));
    }
}

/**
 * @brief To enable the necessary modules for the application
 * - WLAN
 * - UDP
 * - BLE
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T retcode = WLAN_Enable();
    if (RETCODE_OK == retcode)
    {
    	retcode = LED_Enable();
    }
    if (RETCODE_OK == retcode)
    {
    	retcode = Button_Enable();
    }
    if (RETCODE_OK == retcode)
    {
        retcode = UDP_Enable();
    }
    /*if (RETCODE_OK == retcode)
    {
        retcode = BLE_Enable();
    }*/
    if (RETCODE_OK == retcode)
    {
        retcode = Sensor_Enable();
    }
    if (RETCODE_OK == retcode)
    {
    	accelQueue = xQueueCreate(150, sizeof(uint8_t) * APP_CONTROLLER_BLE_TX_LEN);
    	if(accelQueue == NULL)
    	{
    		/* Queue was not created and must not be used. */
    		retcode = RETCODE_FAILURE;
    	}
    }

    if (RETCODE_OK == retcode)
    {
        if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController",
        						  TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
        if (pdPASS != xTaskCreate(UdpController, (const char * const ) "UdpController",
                				  TASK_STACK_SIZE_UDP_CONTROLLER, NULL, TASK_PRIO_UDP_CONTROLLER, &UdpControllerHandle))
        {
        	retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
        if (pdPASS != xTaskCreate(ActivateCircuitOutput, (const char * const ) "CircuitOutput",
                        		  TASK_STACK_SIZE_CCTOUT_CONTROLLER, NULL, TASK_PRIO_UDP_CONTROLLER, &CctOutControllerHandle))
        {
          	retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerEnable : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }

    Utils_PrintResetCause();
}

/**
 * @brief To setup the necessary modules for the application
 * - WLAN
 * - UDP
 * - BLE
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    // Setup PA1 as the output to activate the circuit to trigger the shock
    GPIO_DriveModeSet(gpioPortA, gpioDriveModeStandard);
    GPIO_PinModeSet(gpioPortA, 1, gpioModePushPullDrive, 0);

    Retcode_T retcode = WLAN_Setup(&WLANSetupInfo);
    if (RETCODE_OK == retcode)
    {
    	retcode = LED_Setup();
	}
    if (RETCODE_OK == retcode)
    {
        ButtonSetup.CmdProcessorHandle = AppCmdProcessor;
        retcode = Button_Setup(&ButtonSetup);
    }
    if (RETCODE_OK == retcode)
    {
        retcode = UDP_Setup(UDP_SETUP_USE_CC31XX_LAYER);
    }
    if (RETCODE_OK == retcode)
    {
        retcode = BLE_Setup(&BLESetupInfo);
    }
    if (RETCODE_OK == retcode)
    {
        SensorSetup.CmdProcessorHandle = AppCmdProcessor;
        retcode = Sensor_Setup(&SensorSetup);
    }
    if (RETCODE_OK == retcode)
    {
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/* global functions ********************************************************* */

/** Refer interface header for description */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);
    Retcode_T retcode = RETCODE_OK;

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

static BaseType_t shockConditionMet(Sensor_Value_T  *sensor)
{
	return pdFALSE;
}


/**@} */
/** ************************************************************************* */
