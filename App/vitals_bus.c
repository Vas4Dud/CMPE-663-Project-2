/**
 ******************************************************************************
 * @file    vitals_bus.c
 * @brief   YOUR FILE — the sensor bus IO layer (refer to vitals_bus.h for
 *          the concept).
 *
 *          SWEN 563 / CMPE 663 — P2 "DG-30 DecuGuard" starter.
 *
 *  WHAT YOU WRITE HERE (spec R3/R4/R13 groundwork):
 *
 *   1. The five IO callbacks that each component driver needs.
 *      STTS22H_IO_t and ISM330DHCX_IO_t give their shapes. Read the
 *      headers in Drivers/Components, because they ARE the documentation.
 *        - Init:     nothing to do, because CubeMX ran MX_I2C3_Init()
 *                    before your code starts. Return 0. (RegisterBusIO
 *                    refuses a NULL pointer.)
 *        - DeInit:   also a no-op that returns 0.
 *        - ReadReg:  HAL_I2C_Mem_Read (&hi2c3, DevAddr, Reg,
 *                    I2C_MEMADD_SIZE_8BIT, pData, Len, timeout). Map
 *                    HAL_OK to 0, and map each other value to -1.
 *        - WriteReg: HAL_I2C_Mem_Write, with the same mapping.
 *        - GetTick:  (int32_t)HAL_GetTick().
 *      The IO struct of the IMU has one more member, Delay. Connect it to
 *      a HAL_Delay wrapper. The ST BSP forgets this member, and a garbage
 *      function pointer is a defect, also while no code calls it today.
 *
 *   2. vitals_bus_init(), for each sensor:
 *        a. fill the IO struct. The Address comes from the component
 *           header (STTS22H_I2C_ADD_H / ISM330DHCX_I2C_ADD_H). The DK
 *           straps the two address pins high. These are 8-bit forms, so
 *           give them to the driver without a change,
 *        b. <sensor>_RegisterBusIO(),
 *        c. <sensor>_ReadID() and CHECK the id (STTS22H_ID /
 *           ISM330DHCX_ID). Report a mismatch by name with printf and
 *           return a negative value (R3),
 *        d. <sensor>_Init(), then TEMP_Enable or ACC_Enable. The drivers
 *           power up DISABLED, and no data streams before you enable one,
 *        e. set the rates with STTS22H_TEMP_SetOutputDataRate(
 *           &temp_sensor, 1.0f). The IMU default after ACC_Enable is
 *           104 Hz at ±2 g, which is correct for R4.
 *
 *  Check your work with the D2 demo row. POST must print the two WHO_AM_I
 *  ids. Then single-step ONE ReadReg call down into HAL_I2C_Mem_Read, and
 *  see that it uses the same I2C3 registers that you programmed by hand
 *  in P1. That path, from your call down to the registers and back, is
 *  the point of this project.
 ******************************************************************************
 */
#include <stdio.h>
#include "main.h"
#include "vitals_bus.h"

extern I2C_HandleTypeDef hi2c3;          /* CubeMX-generated (main.c)     */

#define BUS_TMO_MS 100u

STTS22H_Object_t    temp_sensor;
ISM330DHCX_Object_t imu;

/* ===================== STUDENT CODE BEGIN — IO glue ===================== */

/* TODO: write the IO callbacks that the header describes, for example:
 *
 *   static int32_t bus_init(void)   { return 0; }
 *   static int32_t bus_read(uint16_t addr, uint16_t reg,
 *                           uint8_t *p, uint16_t len) { ... }
 *   ...
 */

/* ====================== STUDENT CODE END — IO glue ====================== */
static int32_t bus_init(void)   { return 0; }

static int32_t bus_deinit(void)   { return 0; }

static int32_t bus_read(uint16_t addr, uint16_t reg,
                            uint8_t *p, uint16_t len) 
 {
    if (HAL_I2C_Mem_Read (&hi2c3, addr, reg,
                     I2C_MEMADD_SIZE_8BIT, p, len, BUS_TMO_MS) == HAL_OK)
                     return 0;
    else
    {
        return -1;
    }
 }

 static int32_t bus_write(uint16_t addr, uint16_t reg,
                            uint8_t *p, uint16_t len) 
 {
    if (HAL_I2C_Mem_Write(&hi2c3, addr, reg,
                     I2C_MEMADD_SIZE_8BIT, p, len, BUS_TMO_MS) == HAL_OK)
    {
        return 0;
    }
    else
    {
        return -1;
    }
 }

static int32_t bus_tick(void)   { return (int32_t)HAL_GetTick(); }

static void bus_delay(uint32_t ms)   { HAL_Delay(ms); }

int32_t vitals_bus_init(void)
{
    /* =================== STUDENT CODE BEGIN — binding =================== */
    //INITIALIZE TEMP SENSOR
    STTS22H_IO_t temp_io;
    temp_io.Init = bus_init;
    temp_io.DeInit = bus_deinit;
    temp_io.BusType = 0;
    temp_io.Address = STTS22H_I2C_ADD_H;
    temp_io.WriteReg = bus_write;
    temp_io.ReadReg = bus_read;
    temp_io.GetTick = bus_tick;

    int32_t temp_pass = 0;
    int32_t fail_found_temp = 0;
    
    temp_pass = STTS22H_RegisterBusIO(&temp_sensor, &temp_io);
    if (temp_pass != 0){fail_found_temp = 1;}

    uint8_t id_temp = 0;
    temp_pass = STTS22H_ReadID(&temp_sensor, &id_temp);
    if (temp_pass != 0){fail_found_temp = 1;}
    //add print statement here
    if (id_temp != STTS22H_ID)
    {
        fail_found_temp = 1;
    }


    //INITIALIZE ACC
    ISM330DHCX_IO_t acc_io;
    acc_io.Init = bus_init;
    acc_io.DeInit = bus_deinit;
    acc_io.BusType = 0;
    acc_io.Address = ISM330DHCX_I2C_ADD_H;
    acc_io.WriteReg = bus_write;
    acc_io.ReadReg = bus_read;
    acc_io.GetTick = bus_tick;
    acc_io.Delay = bus_delay;
    
    int32_t acc_pass = 0;
    int32_t fail_found_acc = 0;

    acc_pass = ISM330DHCX_RegisterBusIO(&imu, &acc_io);
    if (acc_pass != 0){fail_found_acc = 1;}

    //add print statement here
    uint8_t id_acc = 0;
    acc_pass = ISM330DHCX_ReadID(&imu, &id_acc);
    if (acc_pass != 0){fail_found_acc = 1;}
    if (id_acc != ISM330DHCX_ID)
    {
        fail_found_acc = 1;
    }

    //print found or missing
    if (fail_found_temp == 0)
    {
        printf("STTS22H 0x%X Found!\n", id_temp);
    }
    else {
        printf("STTS22H 0x%X missing!\n", id_temp);
    }

    if (fail_found_acc == 0)
    {
        printf("ISM330DHCX 0x%X Found!\n", id_acc);
    }
    else {
        printf("ISM330DHCX 0x%X missing!\n", id_acc);
    }
    
    if (fail_found_acc == 0 && fail_found_temp == 0)
    {
        //Enable the sensors
        STTS22H_Init(&temp_sensor); 
        STTS22H_TEMP_Enable(&temp_sensor);
        STTS22H_TEMP_SetOutputDataRate(&temp_sensor, 1.0f);

        ISM330DHCX_Init(&imu); 
        ISM330DHCX_ACC_Enable(&imu);
        return 0;
    }
    else {
        return -1;
    }

    /* ==================== STUDENT CODE END — binding ==================== */
}
