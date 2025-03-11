/**
 * @file main.h
 * @author Creciunel Catalin
 * @brief Definition and declaration of the main program
 * @version 0.1
 * @date 2025-03-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
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

/**
 * @brief FreeRTOS tasks time constants 
 * - TASK_DELAY_MOTOR - Delay for the motor task
 * - TASK_DELAY_LCD - Delay for the LCD task
 * 
 */
#define TASK_DELAY_MOTOR 20
#define TASK_DELAY_LCD  500
#define TASK_DELAY_ENCODER 30
#define TASK_DELAY_COMMUNICATION 100
#define TASK_DELAY_SENSOR 2000


/**
 * @brief OLED display constants pins
 * - ADDRESS - I2C address
 * - SCL_PIN - SCL pin for the I2C communication
 * - SDA_PIN - SDA pin for the I2C communication
 * - DISPLAY_HEIGHT - Height of the display
 * - DISPLAY_WIDTH - Width of the display
 */
#define ADDRESS 0x27 // I2C address
#define SCL_PIN 21
#define SDA_PIN 20

#define DISPLAY_HEIGHT 2
#define DISPLAY_WIDTH 16

/**
 * @brief Constants for stepper driver (TB6600) pins Pump
 * - DIR_PIN1 - Direction pin for the first motor
 * - PUL_PIN1 - Pulse pin for the first motor
 * - ENA_PIN1 - Enable pin for the first motor
 */
#define DIR_PIN1 46 // DIR+
#define PUL_PIN1 44 // PUL+
#define ENA_PIN1 48 // ENA+

/**
 * @brief Constants for stepper driver (TB6600) pins Colector 
 * - DIR_PIN2 - Direction pin for the second motor
 * - PUL_PIN2 - Pulse pin for the second motor
 * - ENA_PIN2 - Enable pin for the second motor
 */
#define DIR_PIN2 47 
#define PUL_PIN2 45 
#define ENA_PIN2 49 

/**
 * @brief Stepper motor driver constants
 * - CLOCKWISE - Clockwise direction
 * - COUNTERCLOCKWISE - Counterclockwise direction
 * - FULL_STEP - Full step mode
 * - HALF_STEP - Half step mode
 * - QUARTER_STEP - Quarter step mode
 * - EIGHTH_STEP - Eighth step mode
 * - SIXTEENTH_STEP - Sixteenth step mode
 * - THIRTYTWO_STEP - Thirtytwo step mode
 */
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0

#define FULL_STEP 200
#define HALF_STEP 400
#define QUARTER_STEP 800
#define EIGHTH_STEP 1600
#define SIXTEENTH_STEP 3200
#define THIRTYTWO_STEP 6400

/**
 * @brief ADC constants
 * - ADC_PIN - Analog to digital conversion pin
 * - ADC_RESOLUTION - Resolution of the ADC
 * - R2 - Resistor value for the first voltage divider (R2 = 10k)
 * - R1 - Resistor value for the second voltage divider (R1 = 47k)
 * - R3 - Resistor value for the third voltage divider (R3 = 7.2k)
 * - R4 - Resistor value for the fourth voltage divider (R4 = 1.5k)
 * 
 */
#define ADC_PIN A9 
#define ADC_RESOLUTION 5.0 / 1024.0
#define R2 10151.5  
#define R1 48300000 
#define R3 7200     
#define R4 1500     

/**
 * @brief Macro to calculate the voltage gain
 * 
 */
#define VOLTAGE_GAIN ((R1 + R2) / R2)

/**
 * @brief Time constants
 * - PRINT_DELAY - Delay for printing the values on the serial monitor
 * - COMMUNICATION_DELAY - Delay for the communication task
 * - ENCODER_DELAY - Delay for the encoder task
 * 
 */
#define PRINT_DELAY 2000
#define COMMUNICATION_DELAY 100
#define ENCODER_DELAY 100

/**
 * @brief Macro to convert milliseconds to seconds
 * 
 */
#define MillisecondsToMinutes(milliseconds) ((milliseconds) / 60000.0)

/**
 * @brief Encoder pins
 *  - CLK - Encoder Clock pin
 *  - DT - Encoder Data transfer pin
 *  - SW - Encoder Switch select pin
 */
#define CLK 26
#define DT 24
#define SW 22

/**
 * @brief Relay pin for the high voltage control
 * 
 */
#define RELAY_PIM 15

#define PWM_PIN 9

/**
 * @brief Relay pin for the high voltage control
 * 
 */
#define EN_HIGH_VOLTAGE 13

/**
 * @brief State machine for the display menu
 * 
 * @enum MenuState - Elements of the state machine
 * @var MAIN_MENU - Logo for the display menu
 * @var SELECT_DIRECTION - Select the direction of the motor 1
 * @var SELECT_MOTOR1_SPEED - Select the sering pump speed
 * @var SELECT_MOTOR2_SPEED - Select the colector speed
 * @var SENSOR_VALUES - Display the generan values after starting as temperature, humidity, pressure, voltage, state: start/stop.
 * @var TRAN_PWM_DUTY - Display the PWM duty cycle for the transformer.
 */
enum MenuState
{
    MAIN_MENU, 
    SELECT_DIRECTION,
    SELECT_MOTOR1_SPEED,
    SELECT_MOTOR2_SPEED,
    SENSOR_VALUES,
    TRAN_PWM_DUTY,
    TRAN_PWM_FREQ
};

MenuState currentState = MAIN_MENU;

/**
 * @brief Read the high voltage value usind a high voltage probe from the ADC pin
 * 
 * @param adcValue 
 * @return int 
 */
int voltageValue(uint16_t adcValue)
{
    // Calculate Vout1 from the first divider
    float Vout1 = (ADC_RESOLUTION * adcValue) * ((R1 + R2) / R2);

    // Calculate Vout2 from the second divider
    float Vout2 = Vout1 * ((R3 + R4) / R4);

    return Vout2; // Return the final scaled voltage
}
