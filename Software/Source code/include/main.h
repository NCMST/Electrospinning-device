#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

#include <Arduino_FreeRTOS.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <AFMotor.h>

#include <mwc_stepper.h>
#include <GyverEncoder.h>

#include <avr/wdt.h>


#define ADDRESS          0x27 // I2C address
#define SCL_PIN          21
#define SDA_PIN          20

// Definește pinii pentru TB6600 Stepper motor 1
#define DIR_PIN1          46    // DIR+
#define PUL_PIN1          44    // PUL+
#define ENA_PIN1          48   // ENA+

// Define pinii pentru TB6600 Stepper motor 2
#define DIR_PIN2          47    // DIR+
#define PUL_PIN2          45    // PUL+
#define ENA_PIN2          49   // ENA+

#define CLOCKWISE          1
#define COUNTERCLOCKWISE   0

#define FULL_STEP          200 
#define HALF_STEP          400 
#define QUARTER_STEP       800
#define EIGHTH_STEP        1600
#define SIXTEENTH_STEP     3200
#define THIRTYTWO_STEP     6400

// ADC constants
#define ADC_PIN           A9 // ADC pin
#define ADC_RESOLUTION    5.0 / 1024.0
#define R2                10151.5 // R2 resistor value in ohms
#define R1                48300000 // R1 resistor value in ohms
#define R3                7200 // R3 resistor value in ohms 
#define R4                1500 // R4 resistor value in ohms

// Calculate voltage gain between two resistors
#define VOLTAGE_GAIN ((R1 + R2) / R2)

// Time constants
#define PRINT_DELAY           2000
#define COMMUNICATION_DELAY   100
#define ENCODER_DELAY         100

// Helper functions
#define MillisecondsToMinutes(milliseconds) ((milliseconds) / 60000.0)

// Encoder constants
#define CLK               26
#define DT                24
#define SW                22

// Relay constants
#define RELAY_PIM         15

#define EN_HIGH_VOLTAGE   13 

// display state
enum MenuState {
    MAIN_MENU,
    SELECT_DIRECTION,
    SELECT_MOTOR1_SPEED,
    SELECT_MOTOR2_SPEED,
    SENSOR_VALUES,
};

MenuState currentState = MAIN_MENU;

int voltageValue(uint16_t adcValue)
{
    // Calculate Vout1 from the first divider
    float Vout1 = (ADC_RESOLUTION * adcValue) * ((R1 + R2) / R2);

    // Calculate Vout2 from the second divider
    float Vout2 = Vout1 * ((R3 + R4) / R4);

    return Vout2; // Return the final scaled voltage
}

