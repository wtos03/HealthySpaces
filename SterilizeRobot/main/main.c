/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Smart Thermostat v1.2.0
 * main.c
 * 
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file main.c
 * @brief simple MQTT publish, subscribe, and device shadows for use with AWS IoT EduKit reference hardware.
 *
 * This example takes the parameters from the build configuration and establishes a connection to AWS IoT Core over MQTT.
 *
 * Some configuration is required. Visit https://edukit.workshop.aws
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"
// #include "mpu6886.h"
#include "m5-dcmotor.h"

#include "wifi.h"
#include "fft.h"
#include "ui.h"
#include "sterilizeRobot.h"
#include "ntptime.h"



#define STARTING_ROOMTEMPERATURE 0.0f
/*
#define HEATING "HEATING"
#define COOLING "COOLING"
#define STANDBY "STANDBY"

#define STARTING_SOUNDLEVEL 0x00
#define STARTING_HVACSTATUS "STANDBY"
#define STARTING_ROOMOCCUPANCY false
*/

// Number of slices to split the microphone sample into
#define AUDIO_TIME_SLICES 60
#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 400

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

/*
Semaphore for sound levels
*/
SemaphoreHandle_t xMaxNoiseSemaphore;


/*
uint8_t soundBuffer = STARTING_SOUNDLEVEL;
uint8_t reportedSound = STARTING_SOUNDLEVEL;
char hvacStatus[7] = STARTING_HVACSTATUS;
bool roomOccupancy = STARTING_ROOMOCCUPANCY;
*/


static const char *TAG = "MAIN";
char userName[MAX_NAME_LENGTH ] = "Name";
float temperature = STARTING_ROOMTEMPERATURE;
char  startUsageTime[MAX_DATE_TIME_LENGTH] = "Day:Month:Date:Hour:Min:Sec:Year";
char  stopUsageTime[MAX_DATE_TIME_LENGTH] = "Day:Month:Date:Hour:Min:Sec:Year";
char  durationTime[MAX_DATE_TIME_LENGTH] = "HH:MM";
bool   wsOccupancy = false;
char  robotStatus[MAX_STATUS_STRING] = "Not Cleaned";

time_t now;
struct tm startTime;
struct tm stopTime;
struct tm diffTime;
int    shadowUpdateFlag = true;






void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);

    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

static bool shadowUpdateInProgress;

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
                                const char *pReceivedJsonDocument, void *pContextData) {
    IOT_UNUSED(pThingName);
    IOT_UNUSED(action);
    IOT_UNUSED(pReceivedJsonDocument);
    IOT_UNUSED(pContextData);

    shadowUpdateInProgress = false;

    if(SHADOW_ACK_TIMEOUT == status) {
        ESP_LOGE(TAG, "Update timed out");
    } else if(SHADOW_ACK_REJECTED == status) {
        ESP_LOGE(TAG, "Update rejected");
    } else if(SHADOW_ACK_ACCEPTED == status) {
        ESP_LOGI(TAG, "Update accepted");
    }
}


void occupancy_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);
    if (*(bool *)(pContext->pData)) //True mean workspace is used
    {
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0xFF0000);
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0xFF0000);
        Core2ForAWS_Sk6812_Show();
        // Add starttime
        getTimeStructure(&startTime);
        strftime(startUsageTime, sizeof(startUsageTime), "%c", &startTime);
        ESP_LOGI(TAG, "The current date/time in Thailand is: %s", startUsageTime); 
        strcpy(robotStatus,"Stop Clean");
    }
    else // False workspace is free
    {
        Core2ForAWS_Sk6812_Clear();
        Core2ForAWS_Sk6812_Show();
        // Add stoptime
       
        getTimeStructure(&stopTime);
        strftime(stopUsageTime, sizeof(stopUsageTime), "%c", &stopTime);
        ESP_LOGI(TAG, "The current date/time in Thailand is: %s", stopUsageTime); 
        // Find time difference 
         diffTimePeriod (stopTime,startTime,&diffTime); 
         sprintf(durationTime,"%d Hr: %d Min: %d Sec",diffTime.tm_hour,diffTime.tm_min,diffTime.tm_sec);
     
         ESP_LOGI(TAG, "The workspace usage time  is: %s \n", durationTime);    
         strcpy(robotStatus,"Start Clean");
    }
    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - wsOccupancy state changed to %d", *(bool *) (pContext->pData));
    }
}




// helper function for working with audio data
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    long divisor = (in_max - in_min);
    if(divisor == 0){
        return -1; //AVR returns -1, SAM returns 0
    }
    return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

// --------- Callback for Sterilize Robot
void userName_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    char * userName = (char *) (pContext->pData);
    if(pContext != NULL) {
        ESP_LOGI(TAG, "User name  %s", userName);
    }
}




//----------------------------------------
/*

void microphone_task(void *arg) {
    static int8_t i2s_readraw_buff[1024];
    size_t bytesread;
    int16_t *buffptr;
    double data = 0;

    Microphone_Init();
    uint8_t maxSound = 0x00;
    uint8_t currentSound = 0x00;

    for (;;) {
        maxSound = 0x00;
        fft_config_t *real_fft_plan = fft_init(512, FFT_REAL, FFT_FORWARD, NULL, NULL);
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 1024, &bytesread, pdMS_TO_TICKS(100));
        buffptr = (int16_t *)i2s_readraw_buff;
        for (uint16_t count_n = 0; count_n < real_fft_plan->size; count_n++) {
            real_fft_plan->input[count_n] = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -1000, 1000);
        }
        fft_execute(real_fft_plan);

        for (uint16_t count_n = 1; count_n < AUDIO_TIME_SLICES; count_n++) {
            data = sqrt(real_fft_plan->output[2 * count_n] * real_fft_plan->output[2 * count_n] + real_fft_plan->output[2 * count_n + 1] * real_fft_plan->output[2 * count_n + 1]);
            currentSound = map(data, 0, 2000, 0, 256);
            if(currentSound > maxSound) {
                maxSound = currentSound;
            }
        }
        fft_destroy(real_fft_plan);

        // store max of sample in semaphore
        xSemaphoreTake(xMaxNoiseSemaphore, portMAX_DELAY);
        soundBuffer = maxSound;
        xSemaphoreGive(xMaxNoiseSemaphore);
    }
}
*/

void aws_iot_task(void *param) {
    IoT_Error_t rc = FAILURE;

    char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
    size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);



 //  Sterilize Robot  JSON structure --------------
    jsonStruct_t nameActuator;
    nameActuator.cb = userName_Callback;
    nameActuator.pKey = "userName";
    nameActuator.pData = &userName;
    nameActuator.type = SHADOW_JSON_STRING;
    nameActuator.dataLength =  MAX_NAME_LENGTH;

    jsonStruct_t temperatureHandler;
    temperatureHandler.cb = NULL;
    temperatureHandler.pKey = "temperature";
    temperatureHandler.pData = &temperature;
    temperatureHandler.type = SHADOW_JSON_FLOAT;
    temperatureHandler.dataLength = sizeof(float);

    jsonStruct_t startUsageTimeHandler;
    startUsageTimeHandler.cb = NULL;
    startUsageTimeHandler.pKey = "startUsageTime";
    startUsageTimeHandler.pData = &startUsageTime;
    startUsageTimeHandler.type = SHADOW_JSON_STRING;
    startUsageTimeHandler.dataLength =  MAX_DATE_TIME_LENGTH;

    jsonStruct_t stopUsageTimeHandler;
    stopUsageTimeHandler.cb = NULL;
    stopUsageTimeHandler.pKey = "stopUsageTime";
    stopUsageTimeHandler.pData = &stopUsageTime;
    stopUsageTimeHandler.type = SHADOW_JSON_STRING;
    stopUsageTimeHandler.dataLength =  MAX_DATE_TIME_LENGTH; 

    jsonStruct_t durationHandler;
    durationHandler.cb = NULL;
    durationHandler.pKey = "durationTime";
    durationHandler.pData = &durationTime;
    durationHandler.type = SHADOW_JSON_STRING;
    durationHandler.dataLength = MAX_DATE_TIME_LENGTH;
    
    jsonStruct_t workSpaceActuator;
    workSpaceActuator.cb = occupancy_Callback;
    workSpaceActuator.pKey = "wsOccupancy";
    workSpaceActuator.pData = &wsOccupancy;
    workSpaceActuator.type = SHADOW_JSON_BOOL;
    workSpaceActuator.dataLength = sizeof(bool);  

    jsonStruct_t robotStatusHandler;
    robotStatusHandler.cb = NULL;
    robotStatusHandler.pKey = "robotStatus";
    robotStatusHandler.pData = &robotStatus;
    robotStatusHandler.type = SHADOW_JSON_STRING;
    robotStatusHandler.dataLength =  MAX_STATUS_STRING; 


//------------------------------------------------
    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    // initialize the mqtt client
    AWS_IoT_Client iotCoreClient;

    ShadowInitParameters_t sp = ShadowInitParametersDefault;
    sp.pHost = HostAddress;
    sp.port = port;
    sp.enableAutoReconnect = false;
    sp.disconnectHandler = disconnect_callback_handler;

    sp.pRootCA = (const char *)aws_root_ca_pem_start;
    sp.pClientCRT = "#";
    sp.pClientKey = "#0";
    
    #define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS){
        ESP_LOGE(TAG, "Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    ui_textarea_add("\n\nWorkspace Id:\n>> %s <<\n", client_id, CLIENT_ID_LEN);

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,false, true, portMAX_DELAY);
    setTimeFromNTP();
    ESP_LOGI(TAG, "Shadow Init");

    rc = aws_iot_shadow_init(&iotCoreClient, &sp);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_init returned error %d, aborting...", rc);
        abort();
    }

    ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
    scp.pMyThingName = client_id;
    scp.pMqttClientId = client_id;
    scp.mqttClientIdLen = CLIENT_ID_LEN;

    ESP_LOGI(TAG, "Shadow Connect");
    rc = aws_iot_shadow_connect(&iotCoreClient, &scp);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_connect returned error %d, aborting...", rc);
        abort();
    }
    ui_textarea_add("Connected to AWS IoT Device Shadow service", NULL, 0);

//    xTaskCreatePinnedToCore(&microphone_task, "microphone_task", 4096, NULL, 1, NULL, 1);

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_shadow_set_autoreconnect_status(&iotCoreClient, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d, aborting...", rc);
        abort();
    }

   

    // register delta callback for workSpaceActuator and nameActuator

    rc = aws_iot_shadow_register_delta(&iotCoreClient, &nameActuator);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Shadow Register userName Delta Error");
    }

    rc = aws_iot_shadow_register_delta(&iotCoreClient, &workSpaceActuator);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Shadow Register wsOccupancy Delta Error");
    }

    // =========================loop and publish changes============

    while(NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc || shadowUpdateFlag)
    {
        rc = aws_iot_shadow_yield(&iotCoreClient, 200);
        if(NETWORK_ATTEMPTING_RECONNECT == rc || shadowUpdateInProgress) {
            rc = aws_iot_shadow_yield(&iotCoreClient, 1000);
            // If the client is attempting to reconnect, or already waiting on a shadow update,
            // we will skip the rest of the loop.
           ESP_LOGI(TAG, "****** Wait for reconnect or shadow update ****** ");
           continue;
        }

// START get sensor readings
// sample temperature, convert to fahrenheit
    //  MPU6886_GetTempData(&temperature);
    //  temperature = temperature -10; //  Celcius


// sample from soundBuffer (latest reading from microphone)
  //      xSemaphoreTake(xMaxNoiseSemaphore, portMAX_DELAY);
  //      reportedSound = soundBuffer;
  //      xSemaphoreGive(xMaxNoiseSemaphore);

        ESP_LOGI(TAG, "*****************************************************************************************");
        ESP_LOGI(TAG, "User name :  %s", userName);
        ESP_LOGI(TAG, "On Robot  : Workspace Occupancy %s", wsOccupancy ? "true" : "false");
        ESP_LOGI(TAG, "On Robot  : Robot Status %s", robotStatus);
        ESP_LOGI(TAG, "On Robot  : temperature %f", temperature);
        ESP_LOGI(TAG, "On Robot  : Start Usage time %s", startUsageTime);
        ESP_LOGI(TAG, "On Robot  : Stop Usage time %s", stopUsageTime);
        ESP_LOGI(TAG, "On Robot  : Usage duration time  %s", durationTime);
        
        
        if (shadowUpdateFlag)
        {
            rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
            if(SUCCESS == rc) {
                rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 7, &nameActuator,
                                             &temperatureHandler, &startUsageTimeHandler, &stopUsageTimeHandler,&durationHandler,
                                             &workSpaceActuator,&robotStatusHandler);
                if(SUCCESS == rc) {
                    rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
                    if(SUCCESS == rc) {
                        ESP_LOGI(TAG, "Update Shadow: %s", JsonDocumentBuffer);
                        rc = aws_iot_shadow_update(&iotCoreClient, client_id, JsonDocumentBuffer,
                                               ShadowUpdateStatusCallback, NULL, 4, true);
                        shadowUpdateInProgress = true;
                    }
                }
            }
           shadowUpdateFlag = false;
        }

        ESP_LOGI(TAG, "*****************************************************************************************");
        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }   
    //================================== END While loop  =============================================

    ESP_LOGI(TAG,"******** Out of AWS Task loop **********");

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "An error occurred in the loop %d", rc);
    }

    ESP_LOGI(TAG, "Disconnecting");
    rc = aws_iot_shadow_disconnect(&iotCoreClient);

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Disconnect error %d", rc);
    }

    vTaskDelete(NULL);
}


void robot_task (void *parm)
{
    for (;;)
    {    
     
     if (!strcmp(robotStatus,"Start Clean")) 
     {  // Start robot cleaning
         start_clean(robotStatus); 
     }

     if (!strcmp(robotStatus,"Stop Clean")) 
     {  // Start robot cleaning
         stop_clean();
         strcpy (robotStatus,"Stop Clean");
     }
     
     // Update status to shadow
     shadowUpdateFlag = true;
  }
}

void app_main()
{   
 
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(80);
    Core2ForAWS_LED_Enable(1);
    xMaxNoiseSemaphore = xSemaphoreCreateMutex();

    ui_init();
    initialise_wifi();
    vTaskDelay(pdMS_TO_TICKS(2000));  
    DCMOTOR_Init();
    port_init();  


    xTaskCreatePinnedToCore(&robot_task, "robot_task", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 4096*2, NULL, 5, NULL, 0);
    

}
