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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "config_table.h"
#include "main.h"
#include "app.h"
#include "console.h"
#include "oled.h"
#include "qualify.h"
#include "stm32wb5mxx.h"
#include "stts22h.h"
#include "vitals_bus.h"
#include "ui_pages.h"
static Posture posture = UNCLEAR;
static bool read_angle(uint32_t now);
static bool read_temp(int32_t *new_temp);
int console_tokenize(char *line, char *argv[], int max_tokens);
static void render_live(void);
static void render_clocks(void);
static void render_session(void);
static void set_config(void);
static void alerts_service(uint32_t now, bool b1);
static void alert_clear(int i);
static void alert_found(int i, uint32_t now);
static void set_config(void);
static Posture posture_check(int32_t roll);
static void supervise_turn(uint32_t now);
static void clock_reset(uint32_t now);
static void monitor_angle(uint32_t now);
static void monitor_temp(uint32_t now);
void print_status(void);

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

#define PATIENT_PRESENT       2400
#define PATIENT_ABSENT        2500
#define TEMP_HYSTERISIS       20  
/* EXTI press counters. The ISR writes them, and the loop reads them (R23). */
static volatile uint32_t b1_presses, b2_presses, imu_int1_event;
static int32_t angle_offset = 0;
static int32_t roll_offset = 0;
static int32_t roll_calib = 0;
static qualify_t hob;
static qualify_t temp_qualify;
static int32_t  ref_angle;
static uint32_t clock_start = 0; //posture clock
static uint32_t turn_clock = 0; //turning clock
const char *const pos_names[] = { "SUPINE", "LEFT_30", "RIGHT_30", "UNCLEAR" };
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    /* ISR rule (R23): latch the event and return. No I2C, no printf, and
     * no OLED work here. */
    if (pin == User_B1_Pin)  { b1_presses = 1; }
    if (pin == User_B2_Pin)  { b2_presses = 1; }
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

static int32_t prev_temp = 0;
static int32_t temp = 0;
static int32_t angle = 0;
static int32_t prev_angle = 0;
static uint32_t prev_angle_time = 0;
static uint8_t temp_changed = 0;
static uint8_t angle_changed = 0;
static uint32_t start_clock = 0; //time within minotr
static uint8_t clock_flag = 0;
static uint8_t bed_used = 0;
static uint32_t event_count = 0;
static uint32_t alert_count = 0;
static uint8_t monitor_flag = 0;
static uint8_t baseline_ready = 0;
static int32_t baseline = 0;


typedef struct {
    const char *name;
    bool active;
    bool ack;
    uint32_t time_found;
    uint32_t t_ack;
} Alerts;

static Alerts alert_list[3] = {{"TURN-DUE"}, {"HOB-HIGH"}, {"TEMP-RISE"}};

void app_init(void)
{
    console_init();
    oled_init();

    ui_pages_init();
    ui_pages_register(UI_PAGE_LIVE,    "LIVE",    render_live);
    ui_pages_register(UI_PAGE_CLOCKS,  "CLOCKS",  render_clocks);
    ui_pages_register(UI_PAGE_SESSION, "SESSION", render_session);

    printf("\nDG-30 DecuGuard\n");

    oled_write_line(0, "DG-30");
    oled_write_line(2, "test running");

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

float pitch = 0;
float roll = 0;

void app_service(void)
{
    static uint32_t touch_last, status_last;
    static int32_t  touch_raw = -1;
    
    static uint8_t touch_detected = 0;
    uint32_t now = HAL_GetTick();
    static uint8_t time_status = 0;
    static uint8_t state_changed = 0;
    



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
        else 
        {
            uint32_t time_diff = (uint32_t) now - timer_start;
            if (bed_used)
            {
                if (time_diff > cfg_absent_ms)
                {
                    bed_used = 0;
                    monitor_flag = 0;
                    alert_clear(0);
                    alert_clear(1);
                    alert_clear(2);
                    first_state = 0;
                    printf("[%4lu.%03lu] PATIENT ABSENT\n", now / 1000, now % 1000);
                    event_count++;
                    
                }
            }
            else 
            {
                if (time_diff > cfg_present_ms)
                {
                    clock_start = now;
                    bed_used = 1;
                    monitor_flag = 1;
                    baseline = 0;
                    baseline_ready = 0;
                    qualify_init(&hob, 
                            cfg_hob_limit_tenths, (uint32_t) cfg_t_grace_s * 1000, 20, now);
                    stand_to_mon = 1;
                    first_state = 0;
                    printf("[%4lu.%03lu] PATIENT PRESENT\n", now / 1000, now % 1000);
                    event_count++;
                }
            }

        }
    }
    else {
        first_state = 0;
        stand_to_mon = 0;
    }
    
    static uint32_t temp_last_time = 0;
    if ((uint32_t)(now - temp_last_time) >= 100)
    {
        temp_last_time = now;
        int32_t t;
        bool is_temp_smoothed = read_temp(&t);
        if (is_temp_smoothed)
        {
            temp = t;
            temp_changed = 1;
        }
        angle_changed = read_angle(now);
    }
   
    prev_angle = angle;

    if (monitor_flag == 1) 
    { 
        if (stand_to_mon == 1)
        {
            start_clock = now;
            stand_to_mon = 0;
            ref_angle = angle;
        }
        if (temp_changed)
        {
            monitor_temp(now);
        }
        if (angle_changed)
        {
            monitor_angle(now);
        }
        supervise_turn(now);
    }
    


    if (b1_presses)
    {
        bool b1 = b1_presses;
        b1_presses = 0;
        alerts_service(now, b1);
    }
    
    /* Status line — on change cadence, cheap (R18 discipline). */
    //OONLY OLED SHOULD PRINT EVERYTHING HERE, live updates
    if ((uint32_t)(now - status_last) >= STATUS_PERIOD_MS) {
        status_last = now;
        if (b2_presses)
        {
            b2_presses = 0;
            ui_pages_next();
        }

        if (temp_changed || angle_changed)
        {
            ui_pages_mark_dirty();
        }
        ui_pages_service();
    }

    /* Console echo — the one non-blocking console call (R20). */
    static char buffer[50];
    static uint8_t buffer_length = 0;
    int ch = console_poll();
        
    if (ch >= 0x20 && ch <= 0x7E) {
        buffer[buffer_length++] = (char) ch;
        putchar(ch);
        fflush(stdout);
    } 
    else if (ch == '\r') {
        buffer[buffer_length] = '\0';
        buffer_length = 0;
        printf("\n");
        process_user_input(buffer);
    }
}

static bool read_temp(int32_t *new_temp)
{
    uint8_t temp_status = 0;
    float temp = 0;
    if (STTS22H_TEMP_Get_DRDY_Status(&temp_sensor, &temp_status) != STTS22H_OK)
    {
        return false;
    }

    STTS22H_TEMP_GetTemperature(&temp_sensor, &temp);
    *new_temp = (int32_t) lroundf((temp * 10));
    return true; 
}

static void monitor_temp(uint32_t now)
{
    static uint8_t smooth = 0;
    if (!baseline_ready)
    {
        baseline += temp;
        smooth++;
        if (((uint32_t) now - start_clock) >= 10000 && smooth > 0)
        {
            baseline /= (uint32_t) smooth;
            baseline_ready = 1;
            qualify_init(&temp_qualify, cfg_delta_t_tenths, 3000, 5, now);
            printf("TEMP BASELINE %d.%d", baseline / 10, baseline % 10);
            event_count++;
            smooth = 0;
        }
        return;
    }
    switch (qualify_feed(&temp_qualify, temp - baseline, now)) {
        case QUALIFY_SET:
        {
            alert_found(2, now);
            printf("TEMP-RISE SET %d.%d", temp / 10, temp % 10); 
            event_count++; 
            break;
        }
        case QUALIFY_CLEAR:
        {
            alert_clear(2); 
            printf("TEMP-RISE CLEAR");
            break;
        } 
        default: break;
    }
}

static void monitor_angle(uint32_t now)
{
    static bool turn_state = 0;
    
    switch (qualify_feed(&hob, angle, now)) {
    case QUALIFY_SET: { 
        alert_found(1, now);
        printf("HOB-HIGH"); 
        break; 
    }
    case QUALIFY_CLEAR: 
    {
        alert_clear(1);
        printf("HOB-HIGH CLEAR");
        break;
    }
    default: break;
    }

    if (labs(angle - ref_angle) >= cfg_delta_turn_tenths) 
    {
        if (!turn_state) 
        { 
            turn_state = true; 
            turn_clock = now; 
        }
        else if ((uint32_t)(now - turn_clock) >= (uint32_t)cfg_t_hold_s * 1000u) {
            turn_state = false; 
            ref_angle = angle;
            printf("POSTURE CHANGE");
            event_count++;
            clock_reset(now);
        }
    } else {
        turn_state = false;                       
    }
    Posture pos = posture_check(roll_calib);
    static bool pos_change = false;
    static Posture prev_posture;
    static uint32_t posture_time = 0;
    if (pos == UNCLEAR || pos == posture) 
    { 
        pos_change = false; 
    }
    else if (!pos_change || pos != prev_posture)    
    { 
        prev_posture = pos; 
        posture_time = now; 
        pos_change = true; 
    }
    else if ((uint32_t)(now - posture_time) >= (uint32_t)cfg_t_hold_s * 1000u) {
        printf("POSTURE CHANGE %s->%s", pos_names[posture], pos_names[pos]);
        posture = pos; 
        pos_change = false; 
        event_count++;
        clock_reset(now);
    }
}


static void clock_reset(uint32_t now)
{
    clock_start = now;
    printf("CLOCk RESET");
    alert_list[0].active = false;
    printf("TURN-DUE CLEARED");
}

static void supervise_turn(uint32_t now)
{
    if ((!alert_list[0].active) &&
        ((uint32_t)(now - clock_start) >= (uint32_t)cfg_turn_interval_s * 1000u)) {
        alert_found(0, now);
        printf("TURN-DUE interval %ld s exceeded", (long)cfg_turn_interval_s);
    }
}

static Posture posture_check(int32_t roll)
{
    if (labs(roll) <= 100)
    {
        return SUPINE;
    }
    if (roll >= 200 && roll <= 400)
    {
        return RIGHT_30;
    }
    if (roll <= -200 && roll >= -400)
    {
        return LEFT_30;
    }
    return UNCLEAR;
}

static bool read_angle(uint32_t now)
{

    ISM330DHCX_Axes_t current_acc;
    static ISM330DHCX_Axes_t four_acc[4];
    static ISM330DHCX_Axes_t smooth_acc;
    smooth_acc.x = 0;
    smooth_acc.y = 0;
    smooth_acc.z = 0;
    
    static uint8_t smooth_count = 0;
    if (imu_int1_event == 1)
    {
        imu_int1_event = 0;
        uint32_t check = ISM330DHCX_ACC_GetAxes(&imu, &current_acc);
        if (check == 0)
        {
            if (smooth_count < 4)
            {
                four_acc[smooth_count] = current_acc;
                smooth_count++;
                return false;
            }
            else 
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
                roll = atan2f((float)smooth_acc.y, smooth_acc.z) * (180.0 / M_PI);
                angle = lroundf(pitch * 10) - angle_offset;
                roll_calib = lroundf(roll * 10) - roll_offset;
                smooth_count = 0;
                return true;
            }
        }
    }
    else 
    {
        return false;
    }
    
}

void process_user_input(char* command)
{
    const config_entry_t *set_entry;
    config_cmd_result_t result = config_table_command(command, &set_entry);
    if (result == CFG_CMD_SET_OK)
    {
        char buffer[16];
        config_table_format(set_entry, buffer, 16);
        set_config();
        printf("CONFIG ");
        event_count++;
    }


    char *argv[8];
    int argc = console_tokenize(command, argv, 8);
    if (argc == 0)
    {
        return;
    }

    if (strcmp(argv[0], "status") == 0)
    {
        print_status();  //IMPLEMENT THIS
    }
    else if(strcmp(argv[0], "cal") == 0)
    {
        angle_offset = angle;
        roll_offset = roll_calib;
        printf("CAL complete");
        event_count++;
    }
    else
    {
        printf("wrong command");
    }

}

void print_status(void)
{
    if (bed_used)
    {
        printf("PATIENT PRESENT");
    }
    else
    {
        printf("PATIENT ABSENT");
    }
    
    printf("angle(pitch): %d.%d\n", angle / 10, angle % 10);
    printf("angle(roll): %d.%d\n", roll_calib / 10, roll_calib % 10);

    //temperature 
    printf("Temperature: %d.%d\n", temp / 1000, temp % 1000);
    //posture 
    printf("Posture %s\n", pos_names[posture]);
    //clocks active 
    //alerts 
    printf("total alerts %d\n", alert_count);
    //config values
}

int console_tokenize(char *line, char *argv[], int max_tokens)
{
    int argc = 0;

    while (argc < max_tokens) {
        while (*line == ' ' || *line == '\t') {  /* skip separators */
            *line++ = '\0';
        }
        if (*line == '\0') {
            break;
        }
        argv[argc++] = line;
        while (*line != '\0' && *line != ' ' && *line != '\t') {
            line++;
        }
    }
    return argc;
}

static void set_config(void)
{
    qualify_config(&hob, cfg_hob_limit_tenths, 
        1000 * (uint32_t) cfg_t_grace_s, 20);

    if (baseline_ready)
    {
        qualify_config(&temp_qualify, cfg_delta_t_tenths, 3000, 5);
    }
}

static void render_live(void)
{
    char buf[16]; 
    
    if (bed_used)
    {
        snprintf(buf, sizeof(buf), "yes");
        ui_row_put(2, "Present", buf);
    }
    else
    {
        snprintf(buf, sizeof(buf), "no");
        ui_row_put(2, "Present", buf);
    }
    snprintf(buf, sizeof(buf), "%d.%d", angle / 10, angle % 10);
    if (angle < 0)
    {
        ui_row_put(3, "Angle", buf);
    } 
    else {
        ui_row_put(3, "Angle", buf);
    }
    snprintf(buf, sizeof(buf), "%d.%d", temp / 10, temp % 10);
    ui_row_put(4, "Temp", buf);
    snprintf(buf, sizeof(buf), "%d.%d", baseline / 10, baseline % 10);
    ui_row_put(5, "baseline", buf);
}

static void render_clocks(void)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long) HAL_GetTick() / 1000);
    ui_row_put(2, "Time", buf);

}

static void render_session(void)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)event_count);
    ui_row_put(2, "Events", buf);

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)alert_count);
    ui_row_put(3, "Alerts", buf);
}

static void alert_found(int i, uint32_t now)
{
    alert_list[i].active = true;
    alert_list[i].ack = false;
    alert_list[i].time_found = now;
    alert_list[i].ack = false;
    alert_count++;
    ui_pages_mark_dirty();
}

static void alert_clear(int i)
{
    alert_list[i].active = false;
    alert_list[i].ack = false;
    ui_pages_mark_dirty();
}

static void alerts_service(uint32_t now, bool b1)
{
    if (monitor_flag != 1) return;
    if (b1) {
       int8_t index_high = -1;
        for (int i = 0; i < 3; i++)
        {
            if (alert_list[i].active)
            {
                index_high = i;
                break;
            }
        }
        if (index_high >= 0) {
            alert_list[index_high].ack = true;
            alert_list[index_high].t_ack = now;
            printf("ACK %s overdue %lu s", alert_list[index_high].name,
                      (unsigned long)((now - alert_list[index_high].time_found) / 1000));
            ui_pages_mark_dirty();
        }
    }

    for (int i = 0; i < 3; i++) {
        if (alert_list[i].active && alert_list[i].ack &&
            (uint32_t)(now - alert_list[i].t_ack) >= (uint32_t)cfg_rearm_s * 1000u) {
            alert_list[i].ack = false;
            printf("%s REARM", alert_list[i].name);
            ui_pages_mark_dirty();
        }
    }
}
