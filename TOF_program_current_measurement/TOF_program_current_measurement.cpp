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

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

/***********************************************************************************/
// Settings for RP2040 microcontroller

#define DEV_I2C Wire
#define SerialPort Serial

#define LedPin 25

#define LPN_PIN 19
#define I2C_RST_PIN 12
#define RESOLUTION 64

/**************************************************************************************/

// Components.
VL53L5CX sensor_vl53l5cx_sat(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

/************************************************************************************/

void setup() {

  // Led.
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);

  SerialPort.begin(115200);
  SerialPort.println("Initialize... Please wait, it may take few seconds...");


  // set SDA and SCL PIN:
  DEV_I2C.setSDA(4);
  DEV_I2C.setSCL(5);
  DEV_I2C.begin();

  // Configure VL53L5CX.
  sensor_vl53l5cx_sat.begin(); 
  sensor_vl53l5cx_sat.init_sensor();

  if (sensor_vl53l5cx_sat.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8)) {
    Serial.print("error frequency");
  }

  sensor_vl53l5cx_sat.vl53l5cx_set_ranging_frequency_hz(15); //max : 4x4 = 60; 8x8 = 15
  sensor_vl53l5cx_sat.vl53l5cx_set_sharpener_percent(20);
  sensor_vl53l5cx_sat.vl53l5cx_set_ranging_mode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  sensor_vl53l5cx_sat.vl53l5cx_set_integration_time_ms(10);
  sensor_vl53l5cx_sat.vl53l5cx_set_power_mode(VL53L5CX_POWER_MODE_SLEEP);

  digitalWrite(LedPin, HIGH);
  delay(1000);
  digitalWrite(LedPin, LOW);
}

void loop() {
  sensor_vl53l5cx_sat.vl53l5cx_set_power_mode(VL53L5CX_POWER_MODE_WAKEUP);
  sensor_vl53l5cx_sat.vl53l5cx_start_ranging();
   
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
      ei_printf("TOF%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
    }

    uint16_t mask   = 0b11111111; 
    uint16_t score16 = int((result.classification[1].value)*100);

    uint8_t score8   = score16 & mask;
            
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

  
    do {
      status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
    } while (!NewDataReady);
       
    sensor_vl53l5cx_sat.vl53l5cx_stop_ranging();
    if ((!status) && (NewDataReady != 0)) {
      
      status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);
    
      for (int i = 0 ; i < RESOLUTION ; i++) {

#ifdef VL53L5CX_USE_RAW_FORMAT

        Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] /= 4;                      
#endif
        if ((Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] > 510) || (Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i] == 0)) {
          Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] = 510;           
        }
        features[feature_ix++] = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] / 2;
      }
    }

    sensor_vl53l5cx_sat.vl53l5cx_set_power_mode(VL53L5CX_POWER_MODE_SLEEP);        
  }
  
int compare (const void *a, const void *b)
{
  int s1 = *((int *)a);
  int s2 = *((int *)b);

  return s1 - s2;
}
