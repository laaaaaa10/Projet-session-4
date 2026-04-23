/*********app_tasks.c **********/
/*Jonathan Marois 20/03/2026*/

#include "app_tasks.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "control.h"
#include "protocol.h"
#include "joystick.h"
#include "button.h"

#include "bt_spp_client.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

static QueueHandle_t s_ctrl_queue = NULL;

// Lecture des entrées
static void task_control(void *arg)
{
    control_cmd_t cmd;
    char msg[128];
    int raw_x = 2048;
    int raw_y = 3500;
    bool btn = true;


    while (1)
    {
        // TODO: 7.8
        raw_x = joystick_read_x();
        raw_y = joystick_read_y();
        btn = button_is_pressed();

        control_compute(&cmd, raw_x, raw_y, btn);
      
        protocol_format_ctrl(msg, sizeof(msg), &cmd);

        printf("%s", msg);                     
        
        vTaskDelay(pdMS_TO_TICKS(100));                     // On fait une lecture toutes les 100ms
    }
}



void app_tasks_init(void)
{
    
    xTaskCreate(
        task_control,
        "task_control",
        4096,
        NULL,
        5,              
        NULL);
    
}