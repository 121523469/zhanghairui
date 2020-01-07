/**
******************************************************************************
* File Name          : battery.c
* Description        : This file provides code for the battery detect.
******************************************************************************/
#include <stdint.h>
#include "battery.h"
#include <string.h>
#include <stdio.h>
#include "common_interface.h"
#include "SrvErrorNo.h"
#include "ADCDriver.h"
#include "main.h"

#define TEMPERATURE_N150    (-150)/*-15 ℃*/
#define TEMPERATURE_N350    (-350)/*-35 ℃*/

typedef struct 
{
    uint32_t Per100vol;
    uint32_t Per80vol;
    uint32_t Per60vol;
    uint32_t Per40vol;
    uint32_t Per20vol;
}StBatPerMatrix_t;

typedef enum
{
    EN_BAT_TEMP_NORMAL,
    EN_BAT_TEMP_N_15,
    EN_BAT_TEMP_N_35,
    EN_BAT_TEMP_BUT,
}EnBatTempIndex_t;

static StBatPerMatrix_t BatPerMatrix[EN_BAT_TEMP_BUT] =
{
    {33000, 32500, 32000, 31500, 30000},/*-15度以上电量矩阵,normal*/
    {32000, 31500, 30500, 30000, 29000},/*-15度以下电量矩阵*/
    {31000, 30500, 29500, 28500, 28000},/*-35度以下电量矩阵*/
};

static StBatPerMatrix_t GetBatPerMatrix(int16_t temp)
{
    if(TEMPERATURE_N150 < temp)
    {
        return BatPerMatrix[EN_BAT_TEMP_NORMAL];
    }
    else if(TEMPERATURE_N350 > temp)
    {
        return BatPerMatrix[EN_BAT_TEMP_N_35];
    }
    else
    {
        return BatPerMatrix[EN_BAT_TEMP_N_15];
    }
}

static uint8_t CalculateBatPer(StBatPerMatrix_t Matrix, uint16_t voltage)
{
    if((voltage > Matrix.Per100vol) || (voltage == Matrix.Per100vol))
    {
        return 100;
    }
    else if((voltage > Matrix.Per80vol) || (voltage == Matrix.Per80vol))
    {
        return 80;
    }
    else if((voltage > Matrix.Per60vol) || (voltage == Matrix.Per60vol))
    {
        return 60;
    }
    else if((voltage > Matrix.Per40vol) || (voltage == Matrix.Per40vol))
    {
        return 40;
    }
    else if((voltage > Matrix.Per20vol) || (voltage == Matrix.Per20vol))
    {
        return 20;
    }
    else
    {
        return 5;
    }
}


static void UpdateBatLeftPer(uint8_t u8Per)
{
    if (u8Per < stSysParaData.u8BatteryLeftPercent)
    {
        stSysParaData.u8BatteryLeftPercent = u8Per;
    }
}

/**
* @brief   Battery para calculate.
* @param voltage--battery voltage, temp--tempsensor temperature
* @return battery percentage.
*/
/* */
uint16_t BatParaCalculate(uint16_t voltage, int16_t temp)
{
    uint8_t BatLeftPer = 0;
    uint16_t BatVoltage = 0;
    StBatPerMatrix_t CurBatPerMatrix;
    
    memset((char*)&CurBatPerMatrix, 0, sizeof(StBatPerMatrix_t));
    BatVoltage = (uint16_t)(voltage * 6.9 / 2.2);
    CurBatPerMatrix = GetBatPerMatrix(temp);
    BatLeftPer = CalculateBatPer(CurBatPerMatrix, BatVoltage);
    printf("BatParaCalculate BatVoltage = %d, BatLeftPer = %d!\r\n", BatVoltage, BatLeftPer);
    
    return BatLeftPer;
}

