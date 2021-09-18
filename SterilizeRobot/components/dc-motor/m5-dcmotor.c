/* Refer to m5-dcmotor.h for more information and details
*/

#include <freertos/FreeRTOS.h>
#include <i2c_device.h>
#include "m5-dcmotor.h"



static I2CDevice_t dcmotor_device;
static const char *TAG = "DC Motor";


void DCMOTOR_Init(void)
{
    dcmotor_device = i2c_malloc_device(I2C_NUM_1, 21, 22, 40000, DCMOTOR_MODULE_ADDRESS); ///???????

}

void DCMOTOR_SetSpeed(int16_t num, int16_t speed)
{

 i2c_write_bytes(dcmotor_device, MOTOR_CTRL_ADDR +(num*MOTOR_CTRL_LEN),(uint8_t *)&speed, MOTOR_CTRL_LEN);

}


// Read Encoder return 4 bytes then need to multiply 4
int32_t DCMOTOR_ReadEncoder(int16_t num)
{
   uint8_t buf[4]; 
   i2c_read_bytes(dcmotor_device, ENCODER_READ_ADDR+(num*ENCODER_READ_LEN), buf, ENCODER_READ_LEN);
   return *((int32_t *)buf);
}

void stop_motor(void)
{
  DCMOTOR_SetSpeed(MOTOR_1,0);
  DCMOTOR_SetSpeed(MOTOR_2,0);
}

// If angle =0  move continuous
// It's depend how you connect motor in your project. 
void move_backward ( int16_t speed ,int16_t angle)
{    
    int16_t start_angle, current_angle;
    if (!angle)
    {
        DCMOTOR_SetSpeed(MOTOR_1,speed);
        DCMOTOR_SetSpeed(MOTOR_2,speed);
        ESP_LOGI(TAG, "DC Motor move forward");
    }
    else 
    {
      start_angle = DCMOTOR_ReadEncoder(MOTOR_1);
      DCMOTOR_SetSpeed(MOTOR_1,speed);
      DCMOTOR_SetSpeed(MOTOR_2,speed);
      do
      {
          current_angle =  DCMOTOR_ReadEncoder(MOTOR_1);

      } while ((current_angle -start_angle) < angle);
      stop_motor();
      ESP_LOGI(TAG, "DC Motor move forward  Angle = %d \n",angle);
    }

}
// If angle =0  move continuous
void move_forward ( int16_t speed, int16_t angle)
{
    int16_t start_angle, current_angle;
    if (!angle)
    {
        DCMOTOR_SetSpeed(MOTOR_1,0-speed);
        DCMOTOR_SetSpeed(MOTOR_2,0-speed);
        ESP_LOGI(TAG, "DC Motor move backward");
    }
    else 
    {
      start_angle = DCMOTOR_ReadEncoder(MOTOR_1);
      DCMOTOR_SetSpeed(MOTOR_1,0-speed);
      DCMOTOR_SetSpeed(MOTOR_2,0-speed);
      do
      {
          current_angle =  DCMOTOR_ReadEncoder(MOTOR_1);

      } while ((current_angle -start_angle) < angle);
      stop_motor();

      ESP_LOGI(TAG, "DC Motor move forward  Angle = %d \n",angle);
    }

}


void turn_left (int16_t angle)
{
  int16_t start_angle, current_angle;
  if ( angle == 0 )
        angle = 90;  
  start_angle = DCMOTOR_ReadEncoder(MOTOR_2);
  DCMOTOR_SetSpeed(MOTOR_2,-255);
  DCMOTOR_SetSpeed(MOTOR_1,0);
  do
  {
    current_angle =  DCMOTOR_ReadEncoder(MOTOR_2);
    ESP_LOGI(TAG, "DC Motor current Angle = %d \n",current_angle);
     ESP_LOGI(TAG, "DC Motor turn left Angle = %d \n",(current_angle-start_angle));

  } while ((current_angle - start_angle) < angle); // Need to test for number
  stop_motor();
  ESP_LOGI(TAG, "DC Motor turn left Angle = %d \n",angle);
}


void turn_right (int16_t angle)
{
  int16_t start_angle, current_angle;
  if (angle == 0 )
        angle = 90;  
  start_angle = DCMOTOR_ReadEncoder(MOTOR_1);
  DCMOTOR_SetSpeed(MOTOR_1,-255);
  DCMOTOR_SetSpeed(MOTOR_2,0);
  do
  {
    current_angle =  DCMOTOR_ReadEncoder(MOTOR_1);
    ESP_LOGI(TAG, "DC Motor current Angle = %d \n",current_angle);
    ESP_LOGI(TAG, "DC Motor turn right Angle = %d \n",(current_angle-start_angle));
   } while ((current_angle -start_angle) < angle); // Need to test for number
   stop_motor();
   ESP_LOGI(TAG, "DC Motor turn right Angle = %d \n",angle);
}


