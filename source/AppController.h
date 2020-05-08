/*
 * Copyright (C) Bosch Connected Devices and Solutions GmbH.
 * All Rights Reserved. Confidential.
 *
 * Distribution only to people who need to know this information in
 * order to do their job.(Need-to-know principle).
 * Distribution to persons outside the company, only if these persons
 * signed a non-disclosure agreement.
 * Electronic transmission, e.g. via electronic mail, must be made in
 * encrypted form.
 */
/*----------------------------------------------------------------------------*/

/**
 *  @file
 *
 *  @brief Configuration header for the AppController.c file.
 *
 */

/* header definition ******************************************************** */
#ifndef APPCONTROLLER_H_
#define APPCONTROLLER_H_

/* local interface declaration ********************************************** */
#include "XDK_Utils.h"

/* local type and macro definitions */

/* local module global variable declarations */

/* local inline function definitions */

/* local type and macro definitions */

/* WLAN configurations ****************************************************** */

/**
 * WLAN_SSID is the WIFI network name where user wants connect the XDK device.
 * Make sure to update the WLAN_PSK constant according to your required WIFI network.
 */
#define WLAN_SSID                           "Skynet_2G"

/**
 * WLAN_PSK is the WIFI router WPA/WPA2 password used at the Wifi network connection.
 * Make sure to update the WLAN_PSK constant according to your router password.
 */
#define WLAN_PSK                            "universo@42"

/**
 * WLAN_STATIC_IP is a boolean. If "true" then static IP will be assigned and if "false" then DHCP is used.
 */
#define WLAN_STATIC_IP                      false

/**
 * WLAN_IP_ADDR is the WIFI router WPA/WPA2 static IPv4 IP address (unused if WLAN_STATIC_IP is false)
 * Make sure to update the WLAN_IP_ADDR constant according to your required WIFI network,
 * if WLAN_STATIC_IP is "true".
 */
#define WLAN_IP_ADDR                        XDK_NETWORK_IPV4(0, 0, 0, 0)

/**
 * WLAN_GW_ADDR is the WIFI router WPA/WPA2 static IPv4 gateway address (unused if WLAN_STATIC_IP is false)
 * Make sure to update the WLAN_GW_ADDR constant according to your required WIFI network,
 * if WLAN_STATIC_IP is "true".
 */
#define WLAN_GW_ADDR                        XDK_NETWORK_IPV4(0, 0, 0, 0)

/**
 * WLAN_DNS_ADDR is the WIFI router WPA/WPA2 static IPv4 DNS address (unused if WLAN_STATIC_IP is false)
 * Make sure to update the WLAN_DNS_ADDR constant according to your required WIFI network,
 * if WLAN_STATIC_IP is "true".
 */
#define WLAN_DNS_ADDR                       XDK_NETWORK_IPV4(0, 0, 0, 0)

/**
 * WLAN_MASK is the WIFI router WPA/WPA2 static IPv4 mask address (unused if WLAN_STATIC_IP is false)
 * Make sure to update the WLAN_MASK constant according to your required WIFI network,
 * if WLAN_STATIC_IP is "true".
 */
#define WLAN_MASK                           XDK_NETWORK_IPV4(0, 0, 0, 0)

/* UDP server configurations ************************************************ */

/**
 * DEST_SERVER_IP is the destination server IP address of the web server we will send UDP payloads.
 * If you want to test this example without setting up your own server, you can use publicly available services.
 */
#define DEST_SERVER_IP                      XDK_NETWORK_IPV4(192, 168, 0, 10)

/**
 * DEST_SERVER_PORT is the UDP port to which we will send UDP payloads.
 */
#define DEST_SERVER_PORT                    UINT16_C(5005)

/**
 * APP_CONTROLLER_TX_DELAY is the packet transmit frequency in milli second over UDP & BLE.
 */
#define APP_CONTROLLER_TX_DELAY             UINT32_C(250)
#define UDP_CONTROLLER_TX_DELAY             UINT32_C(50)
#define SENSOR_COLLECTION_PERIOD			UINT32_C(4)

/**
 * APP_CONTROLLER_BLE_DEVICE_NAME is the BLE device name.
 */
#define APP_CONTROLLER_BLE_DEVICE_NAME     "XDK_UDP_BLE"

/**
 * APP_CONTROLLER_BLE_START_ID is the part of the payload to be received from
 * BLE central to start streaming data via. BLE.
 */
#define APP_CONTROLLER_BLE_START_ID         "start"

/**
 * APP_CONTROLLER_BLE_END_ID is the part of the payload to be received from
 * BLE central to stop streaming data via. BLE.
 */
#define APP_CONTROLLER_BLE_END_ID           "end"

/**
 * APP_CONTROLLER_BLE_START_PAYLOAD is the first payload to be sent to the
 * BLE central upon successful reception of APP_CONTROLLER_BLE_START_ID.
 * Must not be more than 20 bytes.
 */
#define APP_CONTROLLER_BLE_START_PAYLOAD    "X      Y      Z"

/**
 * APP_CONTROLLER_BLE_END_PAYLOAD is the last payload to be sent to the
 * BLE central upon successful reception of APP_CONTROLLER_BLE_END_ID.
 * Must not be more than 20 bytes.
 */
#define APP_CONTROLLER_BLE_END_PAYLOAD      "Transfer Terminated!"

#define APP_CONTROLLER_BLE_SEND_TIMEOUT_IN_MS       UINT32_C(1000)/**< Timeout for BLE send */

#define APP_CONTROLLER_BLE_START_TRIGGER            UINT32_C(1)/**< BLE start command send application logic */

#define APP_CONTROLLER_BLE_END_TRIGGER              UINT32_C(0)/**< BLE end command send application logic */

#define APP_CONTROLLER_BLE_TX_LEN                   UINT8_C(50)/**< Size of accelerometer complete payload in bytes with spaces in-between the axis values and NULL terminator */

#define APP_CONTROLLER_TX_AXIS_COUNT                UINT8_C(3)/**< Number of accelerometer axis (x, y & z) */

#define APP_TEMPERATURE_OFFSET_CORRECTION           (-3459)/**< Macro for static temperature offset correction. Self heating, temperature correction factor */

#define APP_CONTROLLER_BLE_RX_BUFFER_SIZE          	UINT8_C(20)/**< Size of BLE receive buffer*/


/* local function prototype declarations */

/**
 * @brief Gives control to the Application controller.
 *
 * @param[in] cmdProcessorHandle
 * Handle of the main command processor which shall be used based on the application needs
 *
 * @param[in] param2
 * Unused
 */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2);

#endif /* APPCONTROLLER_H_ */

/** ************************************************************************* */
