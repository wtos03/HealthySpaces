
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"


#include "core2forAWS.h"
#include "m5-dcmotor.h"
#include "sterilizeRobot.h"





#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4) | (1ULL<<GPIO_INPUT_IO_5))
// Cannot use port G1,G3 don't know why ???? (Maybe TX, RX for Debug )
int ip_port[] = {GPIO_NUM_13,GPIO_NUM_35,GPIO_NUM_36,GPIO_NUM_33,GPIO_NUM_26,GPIO_NUM_19};

// Only Port B,C  
//int ip_port[] = {GPIO_NUM_13,GPIO_NUM_14,GPIO_NUM_36,GPIO_NUM_26};

static const char *TAG = "Sterilize Robot";


void port_init ()
{
/*
    gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
  /*/ 
   int i = 0;
    for ( i = 0; i < NUM_INPUT_PORT ; i++)
    {
        gpio_reset_pin(ip_port[i]);
        gpio_set_direction(ip_port[i],GPIO_MODE_INPUT);
        gpio_set_pull_mode(ip_port[i],GPIO_PULLUP_ONLY);
    }

    gpio_reset_pin(UV_LIGHT);
    gpio_set_direction(UV_LIGHT,GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(UV_LIGHT,GPIO_PULLUP_ONLY);
    gpio_set_level (UV_LIGHT,1);

    gpio_reset_pin(ALCOHOL_SPRAY);
    gpio_set_direction(ALCOHOL_SPRAY,GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(ALCOHOL_SPRAY,GPIO_PULLUP_ONLY);
     gpio_set_level (ALCOHOL_SPRAY,1);
    
}

/* Get GPIO port number for input port
    Input  input port number  0 -> MAX_INPUT_NUM
   Output  GPIO number
*/
int  get_input_port (int i)
{
    return  ip_port[i];
}

// Start cleaning by turn on Alcoho spray and UV-C
void  start_clean (char * status)
{    
    int direction = FORWARD;
    
    gpio_set_level(UV_LIGHT,ON);
    ESP_LOGI(TAG, "Turn on UV-C Light");

    // For Alcohol we need to start/ stop intermittent for not too much spray
    gpio_set_level(ALCOHOL_SPRAY,ON);
    ESP_LOGI(TAG, "Turn on Alcohol Spray"); 
    
     while (!strcmp (status, "Start Clean"))
     {
        move_forward(255,CONTINUOUS);

        // Found object or risk to fall We did not implemet back sensor yet
       if (gpio_get_level(FRONT_COLLISION_SENSOR) && gpio_get_level (FRONT_FALL_SENSOR))
       {
           if ( direction ==  FORWARD)
           {
            move_backward(255,BACK_OFF);
            turn_right(U_TURN); //After U Turn direction is opposite
            direction = BACKWARD;
           }
           else
           {
             move_backward(255,BACK_OFF);
            turn_left(U_TURN); //After U Turn direction is opposite
            direction = FORWARD;
           }     
       }    
     }
}
// Need to implement to go back to start point at this momnet just stop 
// and turn off Alcohol and UV-C
void stop_clean()
{
    gpio_set_level(UV_LIGHT,OFF);
    ESP_LOGI(TAG, "Turn off UV-C Light");
    gpio_set_level(ALCOHOL_SPRAY,OFF);
    ESP_LOGI(TAG, "Turn off Alcohol Spray");    
    stop_motor();      
}

void show_status_port() // For checking input
{
   int i,p,level;
   for (i = 0 ; i < NUM_INPUT_PORT ;i ++)
     {
        p = get_input_port(i);
        level = gpio_get_level(p); 
        ESP_LOGI(TAG, "Sensor plugged in port  %d  (0=no / 1=yes): %d \r" ,p, level);  
    }
    ESP_LOGI(TAG, "--- \r");
 
}

/* Play sound  in buffer !!! It's not work yet

void play_sound (void)
{
   uint8_t sound[16] = {0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff};
   ESP_LOGI(TAG, "Play Sound ");
   
   Microphone_Deinit(); // If the microphone was initialized, be sure to deinit it first.
   Speaker_Init();
    Core2ForAWS_Speaker_Enable(1);
     Speaker_WriteBuff(sound, 16, portMAX_DELAY);
    Core2ForAWS_Speaker_Enable(0);
    Speaker_Deinit();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
*/
/* Have not implement yet. Still have problems */
/*void play_sound (void *arg){
   // If the microphone was initialized, be sure to call Microphone_Deinit() first. 
   Speaker_Init();
   Core2ForAWS_Speaker_Enable(1);
 
   const unsigned char sound[320] = {0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff,
                                    };
   Speaker_WriteBuff((uint8_t*)sound, 320, portMAX_DELAY);
 
   Core2ForAWS_Speaker_Enable(0);
   Speaker_Deinit();
//   vTaskDelete(NULL);
}
*/