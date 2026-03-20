# Electrospinning-device based on ESP32 S3 Hardware arhitecture

## Key components

```mermaid
---
config:
  theme: 'base'
  themeVariables:
    primaryColor: '#BB2528'
    primaryTextColor: '#fff'
    primaryBorderColor: '#7C0000'
    lineColor: '#F8B229'
    secondaryColor: '#006100'
    tertiaryColor: '#fff'
---
classDiagram

    atx24 --> esp32s3 : Power supply
    esp32s3 --> atx24 : Power supply control signal
    esp32s3 --> A4988 : Siringe pump
    GXHT30 --> esp32s3 : Sensor
    Encoder --> esp32s3 : UI
    esp32s3 --> SSD1306 : UI
    esp32s3 --> Termoresistor : safety sensor
    esp32s3 --> Triac1 : Temperature control
    esp32s3 --> Triac2 : Humidity control
    esp32s3 --> L298N : Motor driver
    Voltage --> esp32s3 : Power supply
    Voltage2 --> esp32s3 : HVPS

    atx24 --> A4988

    atx24 : power supply
    atx24 : PC power supply
    GXHT30 : temperature and humidity sensor
    Encoder : Push button encoder
    SSD1306 : LCD display
    Termoresistor : Thermistor
    A4988 : Stepper motor driver
    esp32s3 : Super mini 
    Triac1 : 220V Temperature control
    Triac2 : 220V Humidity control
    L298N : DC 2-10V 1.5A motor driver
    Voltage : 12V
    Voltage2 : High voltage power supply

```
## Components Details

- Power supply 350W : ATX24 
- Stepper motor driver performance up to 35V and ± 1A, motors in fuji, half, 1/4, 1/8, and 1/16 step modes : A4988 
- Temperature and humidity sensor : GXHT30
- Push button encoder : Encoder
- LCD display : SSD1306
- Thermistor : Termoresistor
- Temperature control BTA24 220V : Triac1
- Humidity control BTA24 220V : Triac2
- DC 2-10V 1.5A motor driver : L298N
- 12V : Voltage
- High voltage power supply : Voltage2
