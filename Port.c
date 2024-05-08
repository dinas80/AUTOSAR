 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 *
 * Edited and Updated by: Dina Salah Salem
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"


#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigChannel * Port_PortChannels = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): confifPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None\
* Description: Initializes the Port Driver module.
************************************************************************************/

void Port_Init( const Port_ConfigType* ConfigPtr )
{
    Port_ConfigChannel* Current_ConfigPtr;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
         */
        Port_Status       = PORT_INITIALIZED;
        Port_PortChannels = ConfigPtr->Channels; /* address of the first Channels structure --> Channels[0] */
    }
    volatile uint32 delay = 0;
    Port_PinType i = 0;
    for(i = 0; i < PORT_CONFIGURED_CHANNLES; i++)
    {
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

        Current_ConfigPtr = &(ConfigPtr->Channels[i]);
           switch(Current_ConfigPtr->port_num)
           {
               case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                        break;
               case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                        break;
               case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                        break;
               case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                        break;
               case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                        break;
               case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                        break;
           }

           /* Enable clock for PORT and allow time for clock to start*/
           SYSCTL_RCGC2_REG |= (1<<Current_ConfigPtr->port_num);
           delay = SYSCTL_RCGC2_REG;
           if( ((Current_ConfigPtr->port_num == 3) && (Current_ConfigPtr->pin_num == 7)) || ((Current_ConfigPtr->port_num == 5) && (Current_ConfigPtr->pin_num == 0)) ) /* PD7 or PF0 */
           {
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
           }
           else if( (Current_ConfigPtr->port_num == 2) && (Current_ConfigPtr->pin_num <= 3) ) /* PC0 to PC3 */
           {
               /* Do Nothing ...  this is the JTAG pins */
           }
           else
           {
               /* Do Nothing ... No need to unlock the commit register for this pin */
           }

           if (Current_ConfigPtr -> mode == PORT_PIN_MODE_DIO )
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num * 4));     /* Clear the PMCx bits for this pin */
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
           }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_ADC)
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
           }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_UART)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_SSI)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_I2C)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_M0PWM)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_M1PWM)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Current_ConfigPtr -> mode == PORT_PIN_MODE_CAN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else
           {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Current_ConfigPtr->pin_num);
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Current_ConfigPtr->pin_num* 4));     /* Clear the PMCx bits for this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Current_ConfigPtr->pin_num); //check
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Current_ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */

           }

           if(Current_ConfigPtr->direction == PORT_PIN_OUT)
           {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Current_ConfigPtr->pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

               if(Current_ConfigPtr->initial_value == STD_HIGH)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Current_ConfigPtr->pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
               }
               else
               {
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Current_ConfigPtr->pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
               }
           }
           else if(Current_ConfigPtr->direction == PORT_PIN_IN)
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Current_ConfigPtr->pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

               if(Current_ConfigPtr->resistor == PULL_UP)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Current_ConfigPtr->pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
               }
               else if(Current_ConfigPtr->resistor == PULL_DOWN)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Current_ConfigPtr->pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
               }
               else
               {
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Current_ConfigPtr->pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Current_ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
               }
           }
           else
           {
               /* Do Nothing */
           }


    }


}

/************************************************************************************
* Service Name: Port_SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigChannel * ConfigPtr )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

    switch(ConfigPtr->port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		         break;
     	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		         break;
	    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		         break;
	    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		         break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		         break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		         break;
    }
    
    if( ((ConfigPtr->port_num == 3) && (ConfigPtr->pin_num == 7)) || ((ConfigPtr->port_num == 5) && (ConfigPtr->pin_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (ConfigPtr->port_num == 2) && (ConfigPtr->pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    

  /*  Port_PinModeType mode;
    Port_InternalResistor resistor;
    uint8 initial_value;
    STD pin_direction_changable;
    STD pin_mode_changable;*/
    if(ConfigPtr->direction == PORT_PIN_IN)
    {
	    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(ConfigPtr->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(ConfigPtr->direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(ConfigPtr->resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(ConfigPtr->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

    /* Setup the pin mode as GPIO */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->pin_num * 4));     /* Clear the PMCx bits for this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

}

#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
 volatile uint32 delay = 0;
    Port_PinType i = 0;
    for(i = 0; i < PORT_CONFIGURED_CHANNLES; i++)
    {
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

           switch(Port_PortChannels->port_num)
           {
               case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                        break;
               case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                        break;
               case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                        break;
               case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                        break;
               case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                        break;
               case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                        break;
           } 

    if(Direction == PORT_PIN_OUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);    /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      }
      else
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);   /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
      }
        }

}
#endif


void Port_RefreshPortDirection(void)
{

        uint8 i;
  for(i = 0; i < PORT_CONFIGURED_CHANNLES; i++)
  {
    if( (i > 15) && (i < 20) )
    {
      /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
      if( (Port_PortChannels[i].pin_direction_changable) == PORT_PIN_DIRECTION_CHANGEABLE)
      {
        /* Do Nothing  */
      }
      else
      {
      volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
      volatile uint32 delay = 0;
    
        switch(Port_PortChannels[i].port_num)
        {
          case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	  	 break;
	  case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	    	 break;
      	  case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	  case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
          case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
          case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
        }
        if((Port_PortChannels[i].direction) == PORT_PIN_OUT)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);    /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        }
        else
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);    /* Clear the corresponding bit in the GPIODIR register to configure it as Input pin */
        }
        
      }
    
    }
        
  }
}

#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
    /************************************************ DET Error checks ****************************************/
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if (NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif

#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
        
        if((Pin > 15) && (Pin < 20))
        {
          /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
        
          volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
          volatile uint32 delay = 0;
    
          switch(Port_PortChannels[Pin].port_num)
          {
            case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                        break;
               case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                        break;
               case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                        break;
               case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                        break;
               case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                        break;
               case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                        break;
           }

           if (Port_PortChannels[Pin].mode == PORT_PIN_MODE_DIO )
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
           }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_ADC)
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
           }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_UART)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_SSI)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_I2C)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_M0PWM)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_M1PWM)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else if(Port_PortChannels[Pin].mode == PORT_PIN_MODE_CAN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
           else
           {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num* 4));     /* Clear the PMCx bits for this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); //check
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */

           }

           if(Port_PortChannels[Pin].mode == PORT_PIN_OUT)
           {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

               if(Port_PortChannels[Pin].initial_value == STD_HIGH)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[Pin].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
               }
               else
               {
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[Pin].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
               }
           }
           else if(Port_PortChannels[Pin].direction == PORT_PIN_IN)
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

               if(Port_PortChannels[Pin].resistor == PULL_UP)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[Pin].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
               }
               else if(Port_PortChannels[Pin].resistor == PULL_DOWN)
               {
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[Pin].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
               }
               else
               {
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[Pin].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
               }
           }
           else
           {
               /* Do Nothing */
           }


    }


}
#endif
