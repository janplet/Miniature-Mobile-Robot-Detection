#include <ToF_library_inferencing.h>

#include <Arduino.h>
#include <Wire.h>
#include <vl53l5cx_class.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "arducampico.h"

uint8_t image[162*162]={0};
uint8_t header[1] = {0x00};
struct arducam_config config;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

size_t feature_ix = 0;

/***********************************************************************************/

#define DEV_I2C Wire
#define SerialPort Serial

#define LedPin 25

#define LPN_PIN 19
#define I2C_RST_PIN 12
#define PWREN_PIN -1
#define INT_PIN -1
#define RESOLUTION 64

// Components.
VL53L5CX sensor_vl53l5cx_sat(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

/************************************************************************************/

void setup() {

  pinMode(LedPin, OUTPUT);

  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  SerialPort.begin(115200);
  SerialPort.println("Initialize... Please wait, it may take few seconds...");

  config.sccb = i2c0;
  config.sccb_mode = I2C_MODE_16_8;
  config.sensor_address = 0x24;
  config.pin_sioc = PIN_CAM_SIOC;
  config.pin_siod = PIN_CAM_SIOD;
  config.pin_resetb = PIN_CAM_RESETB;
  config.pin_xclk = PIN_CAM_XCLK;
  config.pin_vsync = PIN_CAM_VSYNC;
  config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;
  config.pio = pio0;
  config.pio_sm = 0;
  config.dma_channel = 0;
  arducam_init(&config);

  
  DEV_I2C.setSDA(4);
  DEV_I2C.setSCL(5);
  DEV_I2C.begin();

  sensor_vl53l5cx_sat.begin();

  digitalWrite(LedPin, HIGH);

  sensor_vl53l5cx_sat.init_sensor();

  if (sensor_vl53l5cx_sat.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8)) {
    Serial.print("error frequency");
  }

  sensor_vl53l5cx_sat.vl53l5cx_set_ranging_frequency_hz(15); 

  sensor_vl53l5cx_sat.vl53l5cx_set_sharpener_percent(20);

  sensor_vl53l5cx_sat.vl53l5cx_set_ranging_mode(VL53L5CX_RANGING_MODE_AUTONOMOUS);

  sensor_vl53l5cx_sat.vl53l5cx_set_integration_time_ms(10);
 
  // Start Measurements
  sensor_vl53l5cx_sat.vl53l5cx_start_ranging();
}

//Median
const uint8_t bufferSize = 9;
int filterBuffer[bufferSize][RESOLUTION];
uint8_t indexFilterBuffer = 0;
int pixelBuffer[bufferSize];
int median[RESOLUTION];


void loop() {
  arducam_capture_frame(&config,image);
  
  GetTofData();
  
  // features buffer full? then classify!
  if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    ei_impulse_result_t result;

    // create signal from features frame
    signal_t signal;
    numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    // run classifier
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    
    if (res != 0) return;  

    // print the predictions
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      //ei_printf("TOF%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
    }

    uint16_t mask   = B11111111; 
    uint16_t score16 = int((result.classification[1].value)*100);
    uint8_t score8   = score16 & mask;

    Serial.write(score8);
    Serial.write(header,1);
    
    // reset features frame
    feature_ix = 0;
  }
}

void GetTofData() {
  static uint8_t loop_count = 0;
  VL53L5CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  char report[64];
  uint8_t status;

  while (indexFilterBuffer < bufferSize) {
    do {
      status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
    } while (!NewDataReady);

    digitalWrite(LedPin, HIGH);

    if ((!status) && (NewDataReady != 0)) {
    
      status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

      for (int i = 0 ; i < RESOLUTION ; i++) {
#ifdef VL53L5CX_USE_RAW_FORMAT
        Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] /= 4;                      
#endif
        if ((Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] > 510) || (Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i] == 0)) {
          Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] = 510;                 
        }
        filterBuffer[indexFilterBuffer][i] = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] / 2;
      }
    }
    indexFilterBuffer++;
  }

  for (uint8_t pixel = 0; pixel < RESOLUTION; pixel++) {
    for (uint8_t frame = 0; frame < bufferSize; frame++) {
      pixelBuffer[frame] = filterBuffer[frame][pixel];
    }
    int pixelBufferLength = sizeof(pixelBuffer) / sizeof(pixelBuffer[0]);
    qsort(pixelBuffer, pixelBufferLength, sizeof(pixelBuffer[0]), compare);
    median[pixel] = pixelBuffer[bufferSize/2+1];
    features[feature_ix++] = median[pixel];
  }
  for (int i = 0; i < RESOLUTION; i++) {  
    Serial.write(median[i]);
  }
    Serial.write(config.image_buf,162*162);

  for (int row = 0; row < bufferSize - 1; row++)
    {
        for (int col = 0; col < RESOLUTION; col++)
            filterBuffer[row][col] = filterBuffer[row+1][col];
    }     
      indexFilterBuffer = bufferSize-1;

  digitalWrite(LedPin, LOW);
}


//function to sort the array
int compare (const void *a, const void *b)
{
  int s1 = *((int *)a);
  int s2 = *((int *)b);

  return s1 - s2;
}
