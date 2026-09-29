/**
 ******************************************************************************
 * @file    app.c
 * @brief   Starter smoke test. REPLACE the body of this module with your
 *          DG-30 code (spec §3/§4).
 *
 *          SWEN 563 / CMPE 663 — P2 "DG-30 DecuGuard" starter.
 *
 *  What this smoke test does, and all that it does:
 *   1. OLED banner and console banner (the provided HAL-port drivers).
 *   2. Scans I2C3 with the raw HAL_I2C_IsDeviceReady() call and reports
 *      the two sensors by address. This shows that the BUS works before
 *      you write one line of the IO layer. Your POST does more: it reads
 *      WHO_AM_I through the component drivers (spec R3,
 *      App/vitals_bus.c).
 *   3. Acquires the TS1 touch key about 10 times each second and shows
 *      the raw TSC counts. Touch the pad, and see the number DROP. This
 *      acquisition sequence is the worked example for your presence gate
 *      (R1). The hysteresis and the presence rules stay yours.
 *   4. Counts B1 and B2 presses through HAL_GPIO_EXTI_Callback (the
 *      worked example for R17: latch the event in the ISR, use it in the
 *      loop, and do no heavy work here).
 *   5. Calls vitals_bus_init() one time at boot. As shipped, that call
 *      prints NOT IMPLEMENTED. Your first task removes that line.
 *
 *  Flash it without a change first. You then get the banner on the OLED
 *  and in PuTTY, and "found 0x38 0x6B" on the console. The raw touch
 *  counts move when you touch the pad, and the button counts increase.
 *  Then start to replace the smoke test.
 *
 *  All the code here obeys the loop discipline that the rubric grades.
 *  app_service() does not block. Each periodic step runs on a
 *  HAL_GetTick() elapsed-time check, which is the P0 and P1 idiom, and
 *  HAL does not change it.
 ******************************************************************************
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "app.h"
#include "console.h"
#include "oled.h"
#include "stm32wb5mxx.h"
#include "vitals_bus.h"

extern I2C_HandleTypeDef hi2c3;          /* CubeMX-generated handles      */
extern TSC_HandleTypeDef htsc;

/* Sensor 7-bit addresses as this board straps them (UM2825 tbl 11), in the
 * left-shifted HAL notation (addr << 1). The macros in the component
 * headers, STTS22H_I2C_ADD_H (0x71) and ISM330DHCX_I2C_ADD_H (0xD7), are
 * the same strappings with bit 0 (the R/W bit) set. The I2C peripheral
 * ignores bit 0 in 7-bit mode, thus each notation addresses the same
 * device.                                                              */
#define ADDR_STTS22H    (0x38u << 1)
#define ADDR_ISM330DHCX (0x6Bu << 1)

#define TOUCH_PERIOD_MS   100u
#define STATUS_PERIOD_MS  500u
#define TEMP_PERIOD_MS    1000u
#define CLOCK_RESET       10000u
#define MONITOR_CLOCK     120000u

#define PATIENT_PRESENT       2400
#define PATIENT_ABSENT        2500
/* EXTI press counters. The ISR writes them, and the loop reads them (R23). */
static volatile uint32_t b1_presses, b2_presses, imu_int1_event;
static uint32_t angle_offset = 0;

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    /* ISR rule (R23): latch the event and return. No I2C, no printf, and
     * no OLED work here. */
    if (pin == User_B1_Pin)  { b1_presses++; }
    if (pin == User_B2_Pin)  { b2_presses++; }
    if (pin == INT1_Pin)     { imu_int1_event = 1; }
}
/*
 * Non-blocking TSC acquisition of the TS1 key (group 6, and group 4 is
 * the shield electrode). Call it in each loop cycle. It returns the new
 * raw charge-transfer count after an acquisition ends, or -1. A lower
 * count means a finger on the pad. The finger adds capacitance, thus
 * fewer transfer cycles fill the sampling capacitor. Measure YOUR pad
 * and see this effect (R1).
 *
 * The function has three phases, and no phase waits. It discharges the
 * electrodes (>=1 ms), starts the acquisition, then polls until the
 * acquisition ends. This is the R1 worked example, and your presence
 * gate adds hysteresis above it.
 */
static int32_t touch_read_raw(void)
{
    static enum { T_IDLE, T_DISCHARGE, T_ACQUIRE } phase = T_IDLE;
    static uint32_t t_phase;
    uint32_t now = HAL_GetTick();

    switch (phase) {
    case T_IDLE:
        HAL_TSC_IODischarge(&htsc, ENABLE);
        t_phase = now;
        phase = T_DISCHARGE;
        return -1;

    case T_DISCHARGE:
        if ((uint32_t)(now - t_phase) < 2u) {
            return -1;                   /* let the electrodes drain      */
        }
        HAL_TSC_IODischarge(&htsc, DISABLE);
        HAL_TSC_Start(&htsc);
        phase = T_ACQUIRE;
        return -1;

    case T_ACQUIRE:
    default:
        /* A max-count error (MCE) stops the acquisition and does NOT set
         * the group-complete flag. Code that polls only for the end of an
         * acquisition stops here forever after one such error. Your R1
         * presence gate must also survive a stopped acquisition.      */
        if (__HAL_TSC_GET_FLAG(&htsc, TSC_FLAG_MCE)) {
            HAL_TSC_Stop(&htsc);         /* clears EOA/MCE, back to READY */
            phase = T_IDLE;
            return -1;
        }
        if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP6_IDX)
            != TSC_GROUP_COMPLETED) {
            return -1;                   /* still counting: come back     */
        }
        {
            int32_t v = (int32_t)HAL_TSC_GroupGetValue(&htsc, TSC_GROUP6_IDX);
            HAL_TSC_Stop(&htsc);
            phase = T_IDLE;
            return v;
        }
    }
}

void app_init(void)
{
    console_init();
    oled_init();
    printf("\nDG-30 DecuGuard -- P2 starter smoke test\n");
    printf("SWEN 563 / CMPE 663. Type: it echoes. B1/B2: counted.\n\n");

    oled_write_line(0, "DG-30  P2 STARTER");
    oled_write_line(2, "smoke test running");
    oled_write_line(7, "touch TS1 pad...");

    /* Raw bus proof: do the two sensors answer at all? (Your POST
     * identifies each one by WHO_AM_I through the component drivers.)   */
    printf("I2C3 scan:");
    if (HAL_I2C_IsDeviceReady(&hi2c3, ADDR_STTS22H, 2u, 10u) == HAL_OK) {
        printf("  STTS22H @0x38 OK");
    } else {
        printf("  STTS22H @0x38 MISSING");
    }
    if (HAL_I2C_IsDeviceReady(&hi2c3, ADDR_ISM330DHCX, 2u, 10u) == HAL_OK) {
        printf("  ISM330DHCX @0x6B OK");
    } else {
        printf("  ISM330DHCX @0x6B MISSING");
    }
    printf("\n");

    /* Your first task: make this call succeed (App/vitals_bus.c). */
    if (vitals_bus_init() == 0) {
        printf("component drivers bound: WHO_AM_I verified\n");
    }
}

void app_service(void)
{
    static uint32_t touch_last, status_last;
    static int32_t  touch_raw = -1;
    static uint8_t bed_used = 0;
    static uint8_t touch_detected = 0;
    uint32_t now = HAL_GetTick();
    static uint8_t time_status = 0;
    static uint8_t previous_state = 0;
    static enum {     STANDBY, MONITOR, ALERT, CONFIG } Mode = STANDBY;
    static enum { SUPINE, LEFT_30, RIGHT_30 } Posture;
    static struct {uint8_t turn_due; uint8_t hob_high; uint8_t temp_arise} Alerts = {0};
    /* Touch sampling — non-blocking, ~10 Hz (R1 groundwork). */
    if ((uint32_t)(now - touch_last) >= TOUCH_PERIOD_MS) {
        int32_t v = touch_read_raw();
        if (v >= 0) {
            touch_raw = v;
            touch_last = now;
            if (v <= PATIENT_PRESENT)
            {
                touch_detected = 1;
            }
            else if (v >= PATIENT_ABSENT)
            {
                touch_detected = 0;
            }
        }
    }
    
    static uint32_t timer_start;
    static uint8_t first_state = 0;
    static uint8_t stand_to_mon = 0;
    if (touch_detected != bed_used)
    {
        if (!first_state)
        {
            timer_start = now;
            first_state = 1;
        }
        else {
            uint32_t time_diff = (uint32_t) now - timer_start;
            if (bed_used)
            {
                if (time_diff > 3000)
                {
                    bed_used = 0;
                    Mode = STANDBY;
                    
                    first_state = 0;
                    printf("PATIENT ABSENT\n");
                }
            }
            else {
                if (time_diff > 1000)
                {
                    bed_used = 1;
                    Mode = MONITOR;
                    stand_to_mon = 1;
                    first_state = 0;
                    printf("[%d.%d] PATIENT PRESENT\n", now / 1000, now % 1000);
                }
            }

        }
    }
    else {
        first_state = 0;
    }
    
    static uint32_t prev_temp = 0;
    static int32_t prev_angle = 0;
    static uint32_t prev_angle_time = 0;
    static uint8_t temp_changed = 0;
    static uint8_t angle_changed = 0;
    static uint32_t start_clock = 0;
    static uint8_t clock_flag = 0;

    uint32_t temp = read_temp();
    if (temp != prev_temp)
    {
        temp_changed = 1;
    }
    else
    {
        temp_changed = 0;
    }
    prev_temp = temp;
    

    int32_t angle = read_angle();
    uint8_t posture_flag = 0;
    if (angle <= 100 && angle >= -100)
    {
        if (Posture != SUPINE)
        {
            posture_flag = 1;
        }
        else
        {
            posture_flag = 0;
        }
        Posture = SUPINE;
    }
    else if (angle <= 400 && angle >= 200)
    {
        if (Posture != RIGHT_30)
        {
            posture_flag = 1;
        }
        else
        {
            posture_flag = 0;
        }
        Posture = RIGHT_30;
    }
    else if (angle <= -200 && angle >= -400)
    {
        if (Posture != LEFT_30)
        {
            posture_flag = 1;
        }
        else
        {
            posture_flag = 0;
        }
        Posture = LEFT_30;
    }
    else if (angle > 400 || angle < -400)
    {
        Alerts.hob_high = 1;
        Mode = ALERT;
    }
    
    if (angle != prev_angle)
    {
        angle_changed = 1;
        prev_angle_time = now;
    }
    else
    {
        angle_changed = 0;
    }
    prev_angle = angle;
    switch (Mode)
    {
        case MONITOR: {
            //oled live view
            //device logs events
            //reposoitning clock runs
            static int32_t baseline = 0;
            if (clock_flag == 0)
            {
                start_clock = now;
                clock_flag = 1;
            }
            if (stand_to_mon == 1)
            {
                
                static uint8_t get_ten_temps = 0;
                
                if (get_ten_temps < 10)
                {
                    baseline += temp;
                    get_ten_temps ++;
                }
                else
                {
                    baseline /= 10;
                    get_ten_temps = 0;
                    stand_to_mon = 0;
                }
            }
            else
            {
                if (abs(temp - baseline) > 2)
                {
                    Alerts.temp_arise = 1;
                    Mode = ALERT; //USE QUALIFY . C HERE????
                }
            }
            if ((uint32_t)(now - start_clock) >= MONITOR_CLOCK)
            {
                Alerts.turn_due = 1;
                Mode = ALERT;
                printf("TURN PATIENT");
            }
            
            if (posture_flag == 1 && 
                        (now - prev_angle_time) >= CLOCK_RESET)
            {
                clock_flag = 0;
            }
            break;
        }
        case ALERT:
        {
            
            break;
        }
    }
    


    
    /* Status line — on change cadence, cheap (R18 discipline). */
    //OONLY OLED SHOULD PRINT EVERYTHING HERE, live updates
    if ((uint32_t)(now - status_last) >= STATUS_PERIOD_MS) {
        status_last = now;
        oled_printf(4, "touch %5ld", (long)touch_raw);
        oled_printf(5, "B1 x%lu  B2 x%lu",
                    (unsigned long)b1_presses, (unsigned long)b2_presses);
        if (temp_changed)
        {
            printf("temperature: %d.%d   \n", temp / 10, temp % 10);
        }
        if (angle_changed)
        {
            if (angle < 0)
            {
                angle *= -1;
                printf("current angle: -%d.%d   \n", angle / 10, angle % 10);
            } 
            else {
                printf("current angle: %d.%d   \n", angle / 10, angle % 10);
            }
        }
    }

    /* Console echo — the one non-blocking console call (R20). */
    static char buffer[16];
    char *argv[8];
    static uint8_t buffer_length = 0;
    int ch = console_poll();
    {
        
        if (ch >= 0x20 && ch <= 0x7E) {
            buffer[buffer_length++] = (char) ch;
            putchar(ch);
            fflush(stdout);
        } 
        else if (ch == '\r') {
            buffer[buffer_length] = '\0';
            buffer_length = 0;
            printf("\n");
            int argc = console_tokenize(buffer, argv, 8);
            if (argc == 0) {
                continue;
            }
            process_user_input(argv);
        }
    }
}

uint32_t read_temp(void)
{
    uint8_t temp_status = 0;
    float temp = 0;
    STTS22H_TEMP_Get_DRDY_Status(&temp_sensor, &temp_status);
    if (temp_status)
    {
        STTS22H_TEMP_GetTemperature(&temp_sensor, &temp);
        uint32_t temperature = (uint32_t) temp * 10;
        return temperature;
        
    }
    return -1;
}

uint32_t angle = 0;
int32_t read_angle()
{
    float pitch = 0;
    ISM330DHCX_Axes_t current_acc;
    static ISM330DHCX_Axes_t four_acc[4];
    static ISM330DHCX_Axes_t smooth_acc;
    smooth_acc.x = 0;
    smooth_acc.y = 0;
    smooth_acc.z = 0;
    
    static uint8_t smooth_count = 0;
    if (imu_int1_event == 1)
    {
        uint32_t check = ISM330DHCX_ACC_GetAxes(&imu, &current_acc);
        if (check == 0)
        {
            four_acc[smooth_count] = current_acc;
            smooth_count++;
            imu_int1_event = 0;
        }
    }
    if (smooth_count == 4)
    {
        for (int i = 0; i < 4; i++)
        {
            smooth_acc.x += four_acc[i].x;
            smooth_acc.y += four_acc[i].y;
            smooth_acc.z += four_acc[i].z;
        }
        smooth_acc.x /= 4;
        smooth_acc.y /= 4;
        smooth_acc.z /= 4;
        float within_sqr = (float)(smooth_acc.y * smooth_acc.y) + (float)(smooth_acc.z * smooth_acc.z);
        pitch = atan2f((float)smooth_acc.x, sqrtf(within_sqr)) * (180.0 / M_PI);
        angle = lroundf(pitch * 10) - angle_offset;

        smooth_count = 0;
        return angle;
    }
}

void process_user_input(char* command[])
{
    uint32_t v;
    if (strcmp(command[0], "status") == 0) 
    {
        //PRINT LOG
    } 
    else if (strcmp(command[0], "cal") == 0) 
    {
        angle_offset = angle;
    }
    else if (strcmp(command[0], "cal") == 0) {
        if (argc < 2 || !parse_u32(command[1], &v)) {
            printf("rejected: usage 'cal <microseconds>' (digits only)\n");
        } else if (validate_e(v, &settings)) {
            settings.E = v;
            show_settings();
        }
}

//Need to figure out graduate requriement still!!!
//example Plain Text, this is all Putyty needs to send
/*
[ 0.000] POST START
[ 0.035] STTS22H FOUND ID=0xA0
[ 0.040] ISM330DHCX FOUND ID=0x6B
[ 0.050] POST PASS
 
[ 15.322] PATIENT PRESENT
[ 15.322] MODE MONITOR
 
[ 25.521] TEMP BASELINE 24.3C
 
[ 140.000] TURN-DUE INTERVAL EXCEEDED
 
[ 150.100] ACK TURN-DUE OVERDUE 10S
 
[ 210.100] TURN-DUE REARM
 
[ 250.201] POSTURE CHANGE SUPINE->LEFT_30
[ 250.202] CLOCK RESET
 
[ 320.500] HOB-HIGH SET 32.1
 
[ 345.800] HOB-HIGH CLEAR
 
[ 380.100] TEMP-RISE SET
 
[ 400.000] PATIENT ABSENT
[ 400.000] MODE STANDBY
[ 400.000] ALERTS CLEARED*/



/*LIVE Page

Default page.

Shows current measurements.

Example:

Plain Text
LIVE
 
Present: YES
 
Angle: 14.8
Temp: 26.1
Base: 24.0
 
NORMAL
 
Show more lines

or

Plain Text
LIVE
 
Present: YES
 
Angle: 32.1
Temp: 25.2
Base: 24.0
 
HOB-HIGH
 
Show more lines

Requirements say LIVE should show:

Plain Text
presence
angle
temperature
baseline temperature*/

/*CLOCKS Page

Shows timing.

Example:

Plain Text
CLOCKS
 
Turn: 01:43
Limit: 02:00
 
Mode: MONITOR
 
State: 01:43
Show more lines

The nurse can directly see:

Plain Text
how long since last reposition
Show more lines

After TURN-DUE:

Plain Text
CLOCKS
 
OVERDUE
 
02:36
Show more lines
SESSION Page

Statistics.

Example:

Plain Text
SESSION
 
Events: 24
 
Alerts: 3
 
Posture:
SUPINE
Show more lines

For GR-A you might also show:

Plain Text
SUP 50%
LEFT 30%
RIGHT 20%*/ //BUTTOn 2 cycles these pages
