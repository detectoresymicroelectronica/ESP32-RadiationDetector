// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
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
#include <string.h>
#include "esp_heap_caps.h"
#include "ll_cam.h"
#include "cam_hal.h"

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf
#define CAM_TASK_STACK             2048

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;

//Parametros de configuracion
#define VECTOR_LENGTH 16384     //Longitud del vector donde se almacenan pixeles defectuosos
#define UMBRAL_D 24             //Umbral de deteccion para dosimetria
#define CAPTURAS 50             //Cantidad de capturas iniciales descartadas antes de operar

//Variables globales
typedef union {
    struct {
        uint32_t sample2:8;
        uint32_t unused2:8;
        uint32_t sample1:8;
        uint32_t unused1:8;
    };
    uint32_t val;
} dma_elem_t;               //Estructura para manejo de buffer

int histograma[256];        //Histograma principal
int histograma_copia[256];  //Copia del histograma principal para permitir el envio 
int semaforo = 0;           //Variable para operar con procesos paralelos
int pixeles = 0;            //Determina la posicion del pixel actual respecto al arreglo
int umbral_c = 0;           //Umbral para el descarte de pixeles rotos
int cuentas_m = 0;          //Almacena la cantidad de cuentas detectadas
int espera = 0;             //Almacena la cantidad de capturas realizadas

int *cuentas_v;             //Puntero a la variable cuentas utilizada por el servidor web

struct {
  int Vect[VECTOR_LENGTH];  //Vector utilizado para almacenar la posicion de los pixeles rotos
  int Cant;                 //Cantidad de pixeles descartados
  int Pos;                  //Variable que almacena la posicion del vector
} Pixel;

//Copiar el contenido del histograma principal al secundario
//para poder enviarlo por una tarea que corre en el core secundario
void copiar_histograma(){
    for(int i = 0; i < 256; i++) {
        histograma_copia[i] = histograma[i];
        histograma[i] = 0;
    }
    semaforo = 1;
}

//Eliminar el contenido del histograma principal para cargar nuevos datos
void borrar_histograma(){

    for(int i = 0; i < 256; i++) {
        histograma[i] = 0;
    }
}

//Envia el histograma por el puerto serie mediante una tarea que corre
//en el core secundario
static void mostrar_histograma(void *arg){

    while(1){
        if(semaforo == 1){;
            for(int i = 0; i < 256; i++) {
                printf("%d ", histograma_copia[i]);
            }
            printf("\n");
            semaforo = 0;
        }
    }
}

//Inicializa el vector de pixeles descartados
void borrar_vector(){

    for(int i = 0; i < VECTOR_LENGTH; i++) {
        Pixel.Vect[i] = 0;
    }
}

//Calcular el umbral de descarte
void calcular_umbral(void){

    int max = 0;
    int loc = 0;

    for(int i = 0; i < 255; i++) {
        if(histograma_copia[i] > max ){
            max = histograma_copia[i]; loc=i;

            if(histograma_copia[i+1] <= max){
                break;
            }
        }
    }
    umbral_c = (int) ( (float) histograma_copia[loc] / (float)( histograma_copia[loc] - histograma_copia[loc+1])) + loc;
}

//Determina los pixeles a descartar y almacena su ubicacion en el
//vector de descarte.
void descartar_pixeles(const uint8_t* src, size_t len){

    const dma_elem_t* dma_el = (const dma_elem_t*)src;
    size_t elements = len / sizeof(dma_elem_t);
    size_t end = elements / 8;
    for (size_t i = 0; i < end; ++i) {
        
        int d1 = dma_el[0].sample1;
        int d2 = dma_el[2].sample1;
        int d3 = dma_el[4].sample1;
        int d4 = dma_el[6].sample1;
        
        dma_el += 8;

            if(d1 > umbral_c && Pixel.Pos < VECTOR_LENGTH ){
                Pixel.Vect[Pixel.Pos] = pixeles;
                Pixel.Pos++;
            }
            pixeles++;

            if(d2 > umbral_c && Pixel.Pos < VECTOR_LENGTH ){
                Pixel.Vect[Pixel.Pos] = pixeles;
                Pixel.Pos++;
            }
            pixeles++;

            if(d3 > umbral_c && Pixel.Pos < VECTOR_LENGTH ){
                Pixel.Vect[Pixel.Pos] = pixeles;
                Pixel.Pos++;
            }
            pixeles++;

            if(d4 > umbral_c && Pixel.Pos < VECTOR_LENGTH ){
                Pixel.Vect[Pixel.Pos] = pixeles;
                Pixel.Pos++;
            }
            pixeles++;
            
            if(pixeles == 1920000){
                Pixel.Cant = Pixel.Pos;
                Pixel.Pos = 0;
            }   
    }

}

//Lectura de los valores del buffer considerando los pixeles
//a descartar
void leer_buffer(const uint8_t* src, size_t len){

    const dma_elem_t* dma_el = (const dma_elem_t*)src;
    size_t elements = len / sizeof(dma_elem_t);
    size_t end = elements / 8;
    for (size_t i = 0; i < end; ++i) {

        int d1 = dma_el[0].sample1;
        int d2 = dma_el[2].sample1;
        int d3 = dma_el[4].sample1;
        int d4 = dma_el[6].sample1;
        
        dma_el += 8;

        if(pixeles == Pixel.Vect[Pixel.Pos]){
            Pixel.Pos++;
        }
        else{
            if(d1 > UMBRAL_D ){cuentas_m++;}
            histograma[d1]++;
        }
        pixeles++;
        
        if(pixeles == Pixel.Vect[Pixel.Pos]){
            Pixel.Pos++;
        }
        else{
            if(d2 > UMBRAL_D ){cuentas_m++;}
            histograma[d2]++;
        }
        pixeles++;

        if(pixeles == Pixel.Vect[Pixel.Pos]){
            Pixel.Pos++;
        }
        else{
            if(d3 > UMBRAL_D ){cuentas_m++;}
            histograma[d3]++;
        }
        pixeles++;

        if(pixeles == Pixel.Vect[Pixel.Pos]){
            Pixel.Pos++;
        }
        else{
            if(d4 > UMBRAL_D ){cuentas_m++;}
            histograma[d4]++;
        }
        pixeles++;
    }

}

static bool cam_get_next_frame(int * frame_pos)
{   
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int * frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if(ll_cam_start(cam_obj, *frame_pos)){
            // Vsync the frame manually
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }

    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken)
{
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("error\n"), cam_event==CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));

        pixeles = 0;
        cuentas_m = 0;
        Pixel.Pos = 0;
        borrar_histograma();
    }

}

static void cam_task(void *arg)
{
    int frame_pos = 0;
    cam_obj->state = CAM_STATE_IDLE;
    cam_event_t cam_event = 0;

    Pixel.Cant = 0;
    Pixel.Pos = 0;
    pixeles = 0;
    espera = 0;
    umbral_c = 0;
    cuentas_m = 0;

    long buf = 0;
    bool cambia = 0;

    borrar_vector();
    borrar_histograma();
    copiar_histograma();

    xQueueReset(cam_obj->event_queue);

    while (1) {
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);

        switch (cam_obj->state) {

            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {

                    if(cam_start_frame(&frame_pos)){
                        cam_obj->frames[frame_pos].fb.len = 0;
                        cam_obj->state = CAM_STATE_READ_BUF;
                    }
                    cambia = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                camera_fb_t * frame_buffer_event = &cam_obj->frames[frame_pos].fb;

                if (cam_event == CAM_IN_SUC_EOF_EVENT) {

                    if (cam_obj->fb_size < (frame_buffer_event->len + 1600)) {
                        ll_cam_stop(cam_obj);
                    }

                    cambia = !cambia;

                    if(cambia == 0){
                        buf = 0;
                    }
                    else{
                        buf = cam_obj->dma_half_buffer_size;
                    }

                    if(Pixel.Cant == 0 && espera == CAPTURAS){
                        descartar_pixeles(&cam_obj->dma_buffer[buf], cam_obj->dma_half_buffer_size);
                    }
                    else {
                        leer_buffer(&cam_obj->dma_buffer[buf], cam_obj->dma_half_buffer_size);
                    }

                } else if (cam_event == CAM_VSYNC_EVENT) {

                    ll_cam_stop(cam_obj);

                    if(!cam_start_frame(&frame_pos)){
                        cam_obj->state = CAM_STATE_IDLE;
                    } else {
                        cam_obj->frames[frame_pos].fb.len = 0;
                    }

                    copiar_histograma();
                    *cuentas_v = cuentas_m;

                    pixeles = 0;
                    cuentas_m = 0;
                    Pixel.Pos = 0;

                    if (espera < CAPTURAS){
                        espera ++;

                        if(espera == (CAPTURAS-1) ){
                            calcular_umbral();
                            printf("\n-umb: %d\n",umbral_c);
                        }
                    }
                    
                }
            }
            break;
        }
    }
}

static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }

    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{   
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int) cam_obj->dma_buffer_size, (int) cam_obj->dma_half_buffer_size, (int) cam_obj->dma_node_buffer_size,
             (int) cam_obj->dma_node_cnt, (int) cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_calloc(1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    size_t fb_size = cam_obj->fb_size;

    /* Allocate memory for frame buffer */
    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(16, alloc_size, _caps);
#else
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);

        cam_obj->frames[x].en = 1;
    }

    cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
    if(NULL == cam_obj->dma_buffer) {
        ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        return ESP_FAIL;
    }

    cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
    CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config, int* cuentas)
{
    cuentas_v = cuentas;

    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{

    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj);

    cam_obj->psram_mode = false;
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;

    cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
    cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = cam_obj->frame_cnt - 1;
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t*));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);

    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES-2, &cam_obj->task_handle, 1);
    xTaskCreatePinnedToCore(mostrar_histograma, "mostrar_histograma", 2048, NULL, 0, NULL, 0);

    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{   

    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }

    ll_cam_deinit(cam_obj);
    
    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            free(cam_obj->frames[x].fb.buf - cam_obj->frames[x].fb_offset);
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{   
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{   
    ll_cam_vsync_intr_enable(cam_obj, true);
}
