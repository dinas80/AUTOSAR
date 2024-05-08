 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Dina Salah
 * 
 * 
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR */
#define PORT_VENDOR_ID (1000U)

/* Port Module Id */
#define PORT_MODULE_ID (120U)

/* Port Instance Id */
#define PORT_INSTANCE_ID (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION (1U)
#define PORT_SW_MINOR_VERSION (0U)
#define PORT_SW_PATCH_VERSION (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_AR_RELEASE_PATCH_VERSION (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED (1U)
#define PORT_NOT_INITIALIZED (0U)

#include "Std_Types.h"
/* AUTOSAR checking between Std Types and Port   Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif

#include "Port_Cfg.h"
/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
 #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
 #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C
/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT Init */
#define PORT_INIT_SID                       (uint8)0x00 

/* Service ID for PORT Set Pin Direction */
#define PORT_SetPinDirection_SID            (uint8)0x01

/* Service ID for PORT Refreash Port Direction */
#define PORT_RefreshPortDirection_SID       (uint8)0x02

/* Service ID for Port Get Version Info */
#define PORT_GetVersionInfo_SID             (uint8)0x03

/* Service ID for Port Set Pin Mode */
#define PORT_SetPinMode_SID                 (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN (uint8)0x0A

/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/* DET code to report API Port_Init service called with wrong parameter */
#define PORT_E_INIT_FAILED (uint8)0x0C

/* DET code to report API Port_SetPinMode service called when mode passed is invalid */
#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D

/* DET code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E

/* DET code to report API service called without module initialization */
#define PORT_E_UNINIT (uint8)0x0F

/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER (uint8)0x10

#define PORT_E_PARAM_CONFIG (uint8)0x11     // check 

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Type definition for Port_Num used by the PORT APIs */
typedef enum
{
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    PORTF
}Port_PortNum;

/* Type definition for Port_PinDirectionType used by the PORT APIs */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Type definition for Port_Num used by the PORT APIs */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Type definition for Port_PinModeType used by the PORT APIs */
typedef enum
{
    PORT_PIN_MODE_DIO  ,
    PORT_PIN_MODE_ADC  ,
    PORT_PIN_MODE_UART ,
    PORT_PIN_MODE_SSI  ,
    PORT_PIN_MODE_I2C  ,
    PORT_PIN_MODE_M0PWM,
    PORT_PIN_MODE_M1PWM,
    PORT_PIN_MODE_CAN

}Port_PinModeType;

/* Type definition for Port_LevelType used by the PORT APIs */
typedef enum
{
    LEVEL_LOW,
    LEVEL_HIGH
}Port_LevelType;

/* Type definition for Port_ChangeType used by the PORT APIs */
typedef enum 
{
    PORT_PIN_DIRECTION_CHANGEABLE,
    PORT_PIN_DIRECTION_NOT_CHANGEABLE
}Port_DirectionChangeType; 

typedef enum 
{
    PORT_PIN_MODE_CHANGEABLE,
    PORT_PIN_MODE_NOT_CHANGEABLE
}Port_ModeChangeType; 


typedef enum
{
    PIN0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7
}Port_PinNum;

/* Data Structure required for initializing the PORT Driver */
typedef struct 
{
    Port_PortNum port_num;
    Port_PinNum pin_num;
    Port_PinModeType mode;
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    Port_LevelType initial_value;
    Port_DirectionChangeType pin_direction_changable;
    Port_ModeChangeType pin_mode_changable;
}Port_ConfigChannel;

/* Data Structure required for initializing the Port Driver */
typedef struct Port_ConfigType
{
	Port_ConfigChannel Channels[PORT_CONFIGURED_CHANNLES];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

 /* Function for PORT Set GPIO Pin API */
void Port_SetupGpioPin(
    const Port_ConfigChannel
    *ConfigPtr
     );

 /* Function for PORT Init API */
void Port_Init(
    const Port_ConfigType* ConfigPtr
    );

 /* Function for PORT Set Pin Direction API */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,
Port_PinDirectionType Direction
);
#endif

/* Function for PORT Refreash Port Direction API */
void Port_RefreshPortDirection(
    void
    );

/* Function for PORT Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(
    Std_VersionInfoType* versioninfo
    );
#endif

/* Function for PORT Set Pin Mode API */
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
    Port_PinType Pin,
    Port_PinModeType Mode
    );
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by PORT and other modules */
extern const Port_ConfigType PORT_Configuration;

#endif /* PORT_H */
