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
#include <esp_http_server.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

//Resultados
int cuentas = 0;

// Camara
static const char *TAG = "dosimeter";

//Web Page
const char on_resp[] = "<!DOCTYPE html> <html> <script src='https://www.gstatic.com/charts/loader.js'></script> <head><link rel='shortcut icon' href='about:blank'></head> <body> <h1> ESP32-Detector</h1> <div id='dosis-chart'></div> <div class='a'><p>*The values expressed are approximate and their use is only recommended for educational purposes.</p></div> </body> <style> h1 { text-align: center; font: Arial; font-size: 24px;} div.a { text-align: center; font: Arial; font-size: 12px;} </style> <script> const CONV = 0.1386; google.charts.load('current', { callback: function () { var chart2 = new google.visualization.LineChart(document.getElementById('dosis-chart')); var options2 = { title : 'Dose Rate*', animation: { duration: 500, easing: 'out', startup: true }, hAxis: { title: 'Time', 'format':'hh:mm:ss' }, vAxis: { title: 'Dose Rate (uSv/h)' }, height: 400, legend: 'bottom', pointSize: 10, series: { 0: {curveType: 'line', lineDashStyle: [2, 2]}, 1: {curveType: 'function'} }, enableInteractivity: false }; function getData(){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/counts', true); xhr.send(null); xhr.onreadystatechange = function(data){ if (xhr.readyState == 4 && xhr.status == 200){ var cuentas = parseFloat(xhr.responseText); var tasadosis = cuentas * CONV; drawChart(tasadosis); } } } var data2 = new google.visualization.DataTable(); data2.addColumn('datetime', 'Time'); data2.addColumn('number', 'Dose Rate'); data2.addColumn('number', 'Mean'); var data3 = new google.visualization.DataTable(); data3.addColumn('number', 'Dose Rate'); getData(); setInterval(getData, 700); function drawChart(tasadosis) { var tiempo = new Date(); data3.addRow([tasadosis]); if(data3.getNumberOfRows() > 32){ data3.removeRow(0); } var mean = 0; for (var i = 0; i < data3.getNumberOfRows(); i++) { mean = mean + data3.getValue(i, 0); } mean = mean / data3.getNumberOfRows(); data2.addRow([tiempo, tasadosis, mean]); if(data2.getNumberOfRows() > 128){ data2.removeRow(0); } chart2.draw(data2, options2); } }, packages:['corechart'] }); </script> </html>";


#define WIFI_SSID "SSID"
#define WIFI_PASS "PASS"
#define WIFI_MAXIMUM_RETRY 5

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Connecting.");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection fail.");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to ap SSID:%s", WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect");
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t get_req_handler(httpd_req_t *req)
{
    return httpd_resp_send(req, on_resp, HTTPD_RESP_USE_STRLEN);
}

esp_err_t valor_handler(httpd_req_t *req)
{   
    char str[10];
    sprintf(str, "%d", cuentas);
    return httpd_resp_send(req, str, HTTPD_RESP_USE_STRLEN);
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};

httpd_uri_t uri_valor = {
    .uri = "/counts",
    .method = HTTP_GET,
    .handler = valor_handler,
    .user_ctx = NULL};

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_valor);
    }

    return server;
}

//Main
void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    connect_wifi();
    setup_server();
    vTaskDelay(6000 / portTICK_RATE_MS);

    if(ESP_OK != init_camera(&cuentas)) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
}