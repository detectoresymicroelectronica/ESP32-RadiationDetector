#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdlib.h>
#include "spi_flash_mmap.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

#define CONFIG_ESPECTROSCOPIA

//Puntero auxiliar no implementado
int cuentas = 0;

// Camara
static const char *TAG = "Espectrometro";

//Main
void app_main(void)
{

    vTaskDelay(2000 / portTICK_RATE_MS);

    if(ESP_OK != init_camera(&cuentas)) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
}