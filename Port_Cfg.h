 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: PORT_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Dina Salah
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                       (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                       (STD_OFF)

/* Pre-compile option for presence of PORT SET PIN MODE API */
#define PORT_SET_PIN_DIRECTION_API                  (STD_ON)

/* Pre-compile option for presence of PORT SET PIN DIRECTION API */
#define PORT_SET_PIN_MODE_API                       (STD_ON)

/* Number of the configured PORT Channels */
#define PORT_CONFIGURED_CHANNLES                    (39U)


#endif /* PORT_CFG_H */
