# Miniature-Mobile-Robot-Detection-using-an-Ultra-Low-Resolution-Time-of-Flight-Sensor

### Source files for ToF paper: *Miniature Mobile Robot Detection using an Ultra-Low Resolution Time-of-Flight Sensor*.

The dataset can be accessed via IEEEDataport https://ieee-dataport.org/documents/miniature-mobile-robot-detection-using-ultra-low-resolution-time-flight-sensor-dataset. The Dataset DOI is 10.21227/28ha-8921.

The ToF_library_inferencing.zip file contains the EdgeImpulse platform library with the best selected model for robot classification. The platform can be accessed at https://www.edgeimpulse.com.

The STM32duino_VL53L5CX file contains the library used for the VL53L5CX ToF sensor from ST Microelectronics. Original files can be accessed at: https://github.com/stm32duino/VL53L5CX.

The TOF_program_current_measurement file contains the source code for intermittent ToF sensor operation in order to measure current of ToF image acquisiton and inference separately. The TOF_program_ToF_CAM_classification file contains the source code for ToF and camera view of the robot during classification as seen in the two demonstration videos. The camera HM01B0 library files can be obtained at https://github.com/ArduCAM/pico-tflmicro. All the program files are modified to run with the Arduino platform.

The file Application_video contains 2 videos of the miniature mobile robot application.
