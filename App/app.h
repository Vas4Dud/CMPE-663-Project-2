/**
 ******************************************************************************
 * @file    app.h
 * @brief   Application entry points that the generated main.c calls from
 *          its USER CODE sections. They keep main.c thin, and they stay in
 *          place after each regeneration.
 *
 *          SWEN 563 / CMPE 663 — P2 "DG-30 DecuGuard" starter.
 *
 *  The generated Core/Src/main.c calls exactly two of your functions:
 *      app_init()      one time, after all MX_*_Init() calls
 *      app_service()   in each cycle of the infinite loop (do not block,
 *                      R22)
 *  The stm32wbxx HAL sends the button and IMU EXTI events to the shared
 *  callback HAL_GPIO_EXTI_Callback(), which app.c holds (R17/R23).
 *
 *  As shipped, app_init and app_service run the SMOKE TEST in app.c: OLED
 *  banner, console echo, raw I2C bus scan, raw touch-key values, and
 *  button press counters. Replace the smoke test with the DG-30 state
 *  machine as you build the spec. The TSC acquisition sequence and the
 *  bus scan of the smoke test are worked examples for R1 and R3.
 ******************************************************************************
 */
#ifndef APP_H
#define APP_H
#include <stdbool.h>
typedef enum { SUPINE, LEFT_30, RIGHT_30, UNCLEAR } Posture;
//extern const char* pos_names[] = {"SUPINE", "LEFT_30", "RIGHT_30", "UNCLEAR"}; 
void app_init(void);
void app_service(void);
void process_user_input(char* command);
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

#endif /* APP_H */
