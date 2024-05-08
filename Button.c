/******************************************************************************
 *
 * Module: Button
 *
 * File Name: Button.c
 *
 * Description: Source file for Button Module.
 *
 * Author: Mohamed Tarek
 *
 * Edited by: Dina Salah Salem
 ******************************************************************************/
#include "Dio.h"
#include "Port.h"
#include "Button.h"

/* Button Configurations Structure */
static Port_ConfigChannel g_Button_Config;

/* Global variable to hold the button state */
static uint8 g_button_state = BUTTON_RELEASED;

/*******************************************************************************************************************/
/* Description: Called by the Button_Init function (only) used to fill the Button configurations structure */
static void Button_FillConfigurations(void)
{
    g_Button_Config.port_num  = DioConf_SW1_PORT_NUM;             /* Set Button PORT value */
    g_Button_Config.pin_num   = DioConf_SW1_CHANNEL_NUM;          /* Set Button PIN Number value */
    g_Button_Config.direction = PORT_PIN_IN;                   /* Set Button as INPUT pin */
    g_Button_Config.resistor  = PULL_UP;                 /* Enable Internal pull up at this pin */
    g_Button_Config.initial_value  = LEVEL_HIGH;    /* Button is released */
}

/*******************************************************************************************************************/
void Button_Init(void)
{
    Button_FillConfigurations();
    Port_SetupGpioPin(&g_Button_Config);
}

/*******************************************************************************************************************/
uint8 Button_GetState(void)
{
    return g_button_state;
}

/*******************************************************************************************************************/
void Button_RefreshState(void)
{
    uint8 state = Dio_ReadChannel(DioConf_SW1_CHANNEL_ID_INDEX);

    /* Count the number of Pressed times increment if the switch pressed for 20 ms */
    static uint8 g_Pressed_Count  = 0;

    /* Count the number of Released times increment if the switch released for 20 ms */
    static uint8 g_Released_Count = 0;

    if(state == BUTTON_PRESSED)
    {
        g_Pressed_Count++;
        g_Released_Count = 0;
    }
    else
    {
        g_Released_Count++;
        g_Pressed_Count = 0;
    }

    if(g_Pressed_Count == 3)
    {
        g_button_state = BUTTON_PRESSED;
        g_Pressed_Count       = 0;
        g_Released_Count      = 0;
    }
    else if(g_Released_Count == 3)
    {
        g_button_state = BUTTON_RELEASED;
        g_Released_Count      = 0;
        g_Pressed_Count       = 0;
    }
}
/*******************************************************************************************************************/
