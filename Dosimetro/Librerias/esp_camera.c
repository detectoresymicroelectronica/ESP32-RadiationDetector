// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sensor.h"
#include "sccb.h"
#include "cam_hal.h"
#include "esp_camera.h"
#include "xclk.h"
#include "ov2640.h"
#include "esp_log.h"
static const char *TAG = "esp_camera";

typedef struct {
    sensor_t sensor;
    camera_fb_t fb;
} camera_state_t;

static camera_state_t *s_state = NULL;

#define CAMERA_ENABLE_OUT_CLOCK(v) camera_enable_out_clock((v))
#define CAMERA_DISABLE_OUT_CLOCK() camera_disable_out_clock()

typedef struct {
    int (*detect)(int slv_addr, sensor_id_t *id);
    int (*init)(sensor_t *sensor);
} sensor_func_t;

static const sensor_func_t g_sensors[] = { {ov2640_detect, ov2640_init},};

static const camera_config_t camera_config = {
    .pin_pwdn = 32,
    .pin_reset = -1,
    .pin_xclk = 0,
    .pin_sccb_sda = 26,
    .pin_sccb_scl = 27,
    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 21,
    .pin_d2 = 19,
    .pin_d1 = 18,
    .pin_d0 = 5,
    .pin_vsync = 25,
    .pin_href = 23,
    .pin_pclk = 22,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_UXGA,

    .jpeg_quality = 0,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t camera_probe(camera_model_t *out_camera_model)
{
    esp_err_t ret = ESP_OK;
    *out_camera_model = CAMERA_NONE;
    if (s_state != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state = (camera_state_t *) calloc(sizeof(camera_state_t), 1);
    if (!s_state) {
        return ESP_ERR_NO_MEM;
    }

    if (camera_config.pin_xclk >= 0) {
        ESP_LOGD(TAG, "Enabling XCLK output");
        CAMERA_ENABLE_OUT_CLOCK(&camera_config);
    }

    if (camera_config.pin_sccb_sda != -1) {
        ESP_LOGD(TAG, "Initializing SCCB");
        ret = SCCB_Init(camera_config.pin_sccb_sda, camera_config.pin_sccb_scl);
    } else {
        ESP_LOGD(TAG, "Using existing I2C port");
        ret = SCCB_Use_Port(camera_config.sccb_i2c_port);
    }

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "sccb init err");
        goto err;
    }

    if (camera_config.pin_pwdn >= 0) {
        ESP_LOGD(TAG, "Resetting camera by power down line");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << camera_config.pin_pwdn;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefull, logic is inverted compared to reset pin
        gpio_set_level(camera_config.pin_pwdn, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(camera_config.pin_pwdn, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (camera_config.pin_reset >= 0) {
        ESP_LOGD(TAG, "Resetting camera");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << camera_config.pin_reset;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(camera_config.pin_reset, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(camera_config.pin_reset, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_LOGD(TAG, "Searching for camera address");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t slv_addr = SCCB_Probe();

    if (slv_addr == 0) {
        ret = ESP_ERR_NOT_FOUND;
        goto err;
    }

    ESP_LOGI(TAG, "Detected camera at address=0x%02x", slv_addr);
    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = camera_config.xclk_freq_hz;

    sensor_id_t *id = &s_state->sensor.id;
    for (size_t i = 0; i < sizeof(g_sensors) / sizeof(sensor_func_t); i++) {
        if (g_sensors[i].detect(slv_addr, id)) {
            camera_sensor_info_t *info = esp_camera_sensor_get_info(id);
            if (NULL != info) {
                *out_camera_model = info->model;
                ESP_LOGI(TAG, "Detected %s camera", info->name);
                g_sensors[i].init(&s_state->sensor);
                break;
            }
        }
    }

    if (CAMERA_NONE == *out_camera_model) { //If no supported sensors are detected
        ESP_LOGE(TAG, "Detected camera not supported.");
        ret = ESP_ERR_NOT_SUPPORTED;
        goto err;
    }

    ESP_LOGI(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
             id->PID, id->VER, id->MIDH, id->MIDL);

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    return s_state->sensor.reset(&s_state->sensor);
err :
    CAMERA_DISABLE_OUT_CLOCK();
    return ret;
}


esp_err_t esp_camera_deinit()
{
    esp_err_t ret = cam_deinit();
    CAMERA_DISABLE_OUT_CLOCK();
    if (s_state) {
        SCCB_Deinit();

        free(s_state);
        s_state = NULL;
    }

    return ret;
}

esp_err_t esp_camera_init(int *cuentas)
{   
    esp_err_t err;
    err = cam_init(&camera_config, cuentas);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    camera_model_t camera_model = CAMERA_NONE;
    err = camera_probe(&camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x(%s)", err, esp_err_to_name(err));
        goto fail;
    }

    framesize_t frame_size = (framesize_t) camera_config.frame_size;
    pixformat_t pix_format = (pixformat_t) camera_config.pixel_format;

    if (frame_size > camera_sensor[camera_model].max_size) {
        ESP_LOGW(TAG, "The frame size exceeds the maximum for this sensor, it will be forced to the maximum possible value");
        frame_size = camera_sensor[camera_model].max_size;
    }

    err = cam_config(&camera_config, frame_size, s_state->sensor.id.PID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera config failed with error 0x%x", err);
        goto fail;
    }

    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;

    ESP_LOGD(TAG, "Setting frame size to %dx%d", resolution[frame_size].width, resolution[frame_size].height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }
    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);
    s_state->sensor.init_status(&s_state->sensor);

    cam_start();

    return ESP_OK;

fail:
    esp_camera_deinit();
    return err;
}

#define FB_GET_TIMEOUT (4000 / portTICK_PERIOD_MS)

sensor_t *esp_camera_sensor_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    return &s_state->sensor;
}

esp_err_t init_camera(int *cuentas)
{
    esp_err_t err = esp_camera_init(cuentas);
    if (err != ESP_OK)
    {
        return err;
    }
    else{
        sensor_t * s = esp_camera_sensor_get();
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_aec2(s, 0);
        s->set_aec_value(s, 1200);
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, 0);
        s->set_bpc(s, 0);
        s->set_wpc(s, 0);
        s->set_raw_gma(s, 0);
        s->set_lenc(s, 0);
        s->set_dcw(s, 0);
    }

    return ESP_OK;
}