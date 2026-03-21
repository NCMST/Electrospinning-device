# Software Architecture

PlatformIO Project Structure using FREERTOS Operating System

Architecture of the Electrospinning-device based on ESP32 S3 describe the task for each component of the device. And the interaction between the components.

* the project shuld be layed to provide easy integration of new components and functionality
* use paralell tasks for better performance
* implement the stack and heap memory analisis of the device 
* use the previus defined web platform to provide an easy access for the user


```mermaid
---
config:
  theme: 'base'
  themeVariables:
    primaryColor: '#45bb25'
    primaryTextColor: '#fff'
    primaryBorderColor: '#7C0000'
    lineColor: '#F8B229'
    secondaryColor: '#006100'
    tertiaryColor: '#fff'
---
classDiagram
    HAL --> Driver
    Driver --> FreertosTask
    ExternalLibrary --> Driver

    FreertosTask --> Core0
    FreertosTask --> Core1

    Core0 --> WifiTask
    Core0 --> UITask

    Core1 --> SensoresTask
    Core1 --> PumpTask
    Core1 --> ColectorTask

    HAL : Hardware Abstraction Layer
    Driver : Drivers of the device
    ExternalLibrary : External Library

    FreertosTask : Real time operating system

    Core0 : Physical comunication ESP32-S3 core
    Core0 : implement UI functionality
    Core1 : Physical core for sensors and control

    WifiTask : MQTT comunication
    UITask : Display, Encoder

    SensoresTask : Thermistor, GXHT30 Humidity-temperature sensor
    PumpTask : Siringe pump
    ColectorTask : DC motor driver, for rotary colector
```

## Extra features
 * upload the code to the ESP32-S3 via OTA 
 * set the pump speed and direction with the encoder
 * set the pump flow RPM
 * set the pump flow mL/min
 * Set the Colector speed
 * provide a menu for the user (with the encoder and dislaplay)