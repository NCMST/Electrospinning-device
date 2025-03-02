/**
 * @file main.cpp
 * @author Creciunel Catalin
 * @brief entry point of the program
 * @version 0.1
 * @date 2025-03-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "main.h"

/**
 * @brief Declare the tasks
 *  - vTaskEncoder - Task for the encoder
 *  - vTaskMotor - Task for the motor
 *  - vTaskLCD - Task for the LCD
 *  - vTaskCommunication - Task for the communication
 *  - vTaskSensor - Task for the sensor
 *
 * @param pvParameters
 */
void vTaskEncoder(void *pvParameters);
void vTaskMotor(void *pvParameters);
void vTaskLCD(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vTaskSensor(void *pvParameters);

/**
 * @brief Motor variables
 * - motor1Speed - Speed of the first motor (float to allow fractional values)
 * - motor2Speed - Speed of the second motor (uint8_t)
 * - motorDirection - Direction of the motor (LOW - Forward, HIGH - Backward)
 */
float motor1Speed = 0.1;
uint8_t motor2Speed = 25;
uint8_t motorDirection = LOW;

/**
 * @brief Global variables for
 * - PWM value for the transformer
 * - Voltage value
 * - Temperature value
 * - Humidity value
 * - Pressure value
 *
 */
float pwmDutyCycle = 0.5;
float voltage = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;

/**
 * @brief Define the global object for the encoder
 *
 * @return Encoder
 */
Encoder enc(CLK, DT, SW, TYPE1);
/**
 * @brief Define the gobal LCD object
 *
 */
LiquidCrystal_I2C lcd(ADDRESS, DISPLAY_WIDTH, DISPLAY_HEIGHT);

/**
 * @brief Define the global object for the stepper motor driver
 *
 */
MWCSTEPPER tb6600_m1(ENA_PIN1, DIR_PIN1, PUL_PIN1);

/**
 * @brief Define the global object for the stepper motor driver
 *
 */
AF_DCMotor motor(1);

/**
 * @brief Define the global flag for the start/stop state
 *  - start - Start/Stop state
 *
 */
struct Flag
{
    uint8_t start = 0;
} flag;

/**
 * @brief Setup function
 * - Initialize the serial communication
 * - Enable the watchdog timer
 * - Create the tasks
 * - Start the FreeRTOS scheduler
 * - Initialize the PWM for the transformer
 * - Initialize the LCD
 * - Set the motor speed
 *
 */
void setup()
{
    Serial.begin(115200);

    wdt_enable(WDTO_2S);

    xTaskCreate(vTaskEncoder, "Encoder Task", 512, NULL, 1, NULL);
    xTaskCreate(vTaskMotor, "Motor Task", 512, NULL, 1, NULL);
    xTaskCreate(vTaskLCD, "LCD Task", 256, NULL, 1, NULL);
    xTaskCreate(vTaskCommunication, "Communication Task", 128, NULL, 1, NULL);
    xTaskCreate(vTaskSensor, "Sensor Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 249; // 14kHz

    OCR1B = (int)(pwmDutyCycle * OCR1A); // Set the PWM duty cycle 50%
    pinMode(9, OUTPUT);

    lcd.init();
    lcd.backlight();

    motor.setSpeed(motor2Speed);
    motor.run(RELEASE);
}

/**
 * @brief Loop function
 * is not used in FreeRTOS
 */
void loop() {}

/**
 * @brief Task for the LCD
 * - Displays the current state of the system
 * - Displays the current speed of the motor
 * - Displays the current direction of the motor
 * - Displays the current sensor values
 * - Displays the current PWM duty cycle
 * - Updates the LCD every 500 ms
 * - Reset the watchdog timer
 *
 * @param pvParameters
 */
void vTaskLCD(void *pvParameters)
{

    lcd.init();
    lcd.backlight();

    for (;;)
    {
        wdt_reset();

        lcd.clear();

        switch (currentState)
        {
        case MAIN_MENU:
            lcd.setCursor(0, 0);
            lcd.print("Electrospinning");
            lcd.setCursor(11, 1);
            lcd.print("setup");
            break;

        case SELECT_DIRECTION:
            lcd.setCursor(0, 0);
            lcd.print("Direction:");
            lcd.setCursor(0, 1);
            lcd.print(motorDirection == LOW ? "Forward" : "Backward");
            break;

        case SELECT_MOTOR1_SPEED:
            lcd.setCursor(0, 0);
            lcd.print("Motor 1 speed:");
            lcd.setCursor(0, 1);
            lcd.print(String(motor1Speed, 2)); // Show the current speed with two decimals
            break;

        case SELECT_MOTOR2_SPEED:
            lcd.setCursor(0, 0);
            lcd.print("Motor 2 speed:");
            lcd.setCursor(0, 1);
            lcd.print(int(motor2Speed)); // Show the current speed
            break;

        case SENSOR_VALUES:
            lcd.setCursor(0, 0);
            lcd.print("V_out:" + String(int(voltageValue(analogRead(ADC_PIN)) / 1000.0)) + "kV");
            lcd.setCursor(0, 1);
            lcd.print("T:" + String((int)(temperature)));
            lcd.setCursor(5, 1);
            lcd.print("H:" + String((int)(humidity)));
            lcd.setCursor(10, 1);
            lcd.print("P:" + String((int)(pressure)));
            lcd.setCursor(11, 0);
            lcd.print(flag.start ? "Start" : "Stop");
            break;
        case TRAN_PWM_DUTY:
            lcd.setCursor(0, 0);
            lcd.print("PWM Duty:");
            lcd.setCursor(0, 1);
            lcd.print(pwmDutyCycle);
            break;
        }

        vTaskDelay(TASK_DELAY_LCD / portTICK_PERIOD_MS); // Update the LCD every 500 ms
    }
}

/**
 * @brief Task for the motor
 * - Initialize the motor driver
 * - Set the motor direction
 * - Set the motor speed
 * - Start the motor
 * - Stop the motor
 * - Reset the watchdog timer
 *
 * @param pvParameters
 */
void vTaskMotor(void *pvParameters)
{
    tb6600_m1.init();
    pinMode(RELAY_PIM, OUTPUT);
    for (;;)
    {
        // motor2Speed = map(motor2Speed, 0, 200, 0, 255);

        tb6600_m1.set(motorDirection == HIGH ? CLOCKWISE : COUNTERCLOCKWISE, motor1Speed, THIRTYTWO_STEP);
        if (flag.start)
        {
            digitalWrite(RELAY_PIM, HIGH);

            tb6600_m1.run(); // Start motorul
            motor.setSpeed(motor2Speed);
            motor.run(motorDirection == HIGH ? FORWARD : BACKWARD);
        }
        else
        {
            digitalWrite(RELAY_PIM, LOW);

            tb6600_m1.active(false); // Stop motorul
            motor.run(RELEASE);
        }
        wdt_reset(); // Resetare watchdog

        vTaskDelay(TASK_DELAY_MOTOR / portTICK_PERIOD_MS); // Delay for fast response
    }
}

/**
 * @brief Task for the encoder
 * - Change the menu state on press
 * - Reset to the initial state on double press
 * - Change the motor direction
 * - Change the motor speed
 * - Change the PWM duty cycle
 * - Reset the watchdog timer
 *
 * @param pvParameters
 */
void vTaskEncoder(void *pvParameters)
{
    for (;;)
    {
        enc.tick();

        if (enc.isSingle())
        {
            switch (currentState)
            {
            case MAIN_MENU:
                currentState = SELECT_DIRECTION;
                Serial.println("Select Direction");
                break;
            case SELECT_DIRECTION:
                currentState = SELECT_MOTOR1_SPEED;
                Serial.println("Select motor 1 speed");
                break;
            case SELECT_MOTOR1_SPEED:
                currentState = SELECT_MOTOR2_SPEED;
                Serial.println("Select motor 2 speed");
                break;
            case SELECT_MOTOR2_SPEED:
                currentState = TRAN_PWM_DUTY;
                Serial.println("PWM Duty");
                break;
            case TRAN_PWM_DUTY:
                currentState = MAIN_MENU;
                Serial.println("Back to Main Menu");
                break;
            case SENSOR_VALUES:
                currentState = MAIN_MENU;
                Serial.println("Back to Main Menu");
                break;
            }
        }
        
        else if (enc.isDouble())
        {
            currentState = SENSOR_VALUES;
            Serial.println("Sensor Values");

            if (flag.start)
            {
                flag.start = false;
                Serial.println("Stop");
                digitalWrite(EN_HIGH_VOLTAGE, LOW);
            }
            else
            {
                flag.start = true;
                Serial.println("Start");
                digitalWrite(EN_HIGH_VOLTAGE, HIGH);
            }
        }

        switch (currentState)
        {
        case SELECT_DIRECTION:
            if (enc.isRight())
            {
                motorDirection = LOW;
                Serial.println("Direction: Forward");
            }
            else if (enc.isLeft())
            {
                motorDirection = HIGH;
                Serial.println("Direction: Backward");
            }
            break;
        case SELECT_MOTOR1_SPEED:
            if (enc.isLeft())
            {
                if (motor1Speed >= 1)
                {
                    motor1Speed++;
                    motor1Speed > 100 ? motor1Speed = 100 : motor1Speed = motor1Speed;
                }
                else if (motor1Speed >= 0.1 && motor1Speed < 1)
                {
                    motor1Speed += 0.1;
                }

                Serial.println("Speed: " + String(motor1Speed, 2));
            }
            else if (enc.isRight())
            {
                if (motor1Speed >= 1)
                {
                    motor1Speed--;
                    Serial.println("Speed: " + String(motor1Speed, 2));
                }
                else if (motor1Speed < 1 && motor1Speed >= 0.1)
                {
                    motor1Speed -= 0.1;
                    if (motor1Speed < 0.1)
                        motor1Speed = 0.1;

                    Serial.println("Speed: " + String(motor1Speed, 2));
                }
            }
            break;
        case SELECT_MOTOR2_SPEED:
            if (enc.isLeft())
            {
                motor2Speed++;
                motor2Speed > 200 ? motor2Speed = 200 : motor2Speed = motor2Speed;
                Serial.println("Speed: " + String(motor2Speed));
            }
            else if (enc.isRight())
            {
                motor2Speed--;
                motor2Speed < 1 ? motor2Speed = 1 : motor2Speed = motor2Speed;
                Serial.println("Speed: " + String(motor2Speed));
            }
            break;
        case TRAN_PWM_DUTY:
            if (enc.isLeft())
            {
                pwmDutyCycle += 0.1;
                pwmDutyCycle > 1 ? pwmDutyCycle = 255 : pwmDutyCycle = pwmDutyCycle;

                Serial.println("PWM Duty: " + String(pwmDutyCycle));
            }
            else if (enc.isRight())
            {
                pwmDutyCycle -= 0.1;
                pwmDutyCycle < 0 ? pwmDutyCycle = 0 : pwmDutyCycle = pwmDutyCycle;
                Serial.println("PWM Duty: " + String(pwmDutyCycle));
            }
            break;
        }

        wdt_reset();

        vTaskDelay(TASK_DELAY_ENCODER / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Communication task
 * - Read commands from the serial port
 * - Initialize variables based on commands
 *
 * - s - start, d - direction, D- duty cycle, m - motor 1 speed, M - motor 2 speed
 * Vlues: s - 0/1, d - 0/1, D - 0-1 (float), m - 0-100, M - 1-200
 *
 * @param pvParameters
 */
void vTaskCommunication(void *pvParameters)
{
    Serial.println("System Initialized\r\n s - start, d - direction, D- duty cycle, m - motor 1 speed, M - motor 2 speed\r\n");

    for (;;)
    {
        if (Serial.available() > 0)
        {
            char key = Serial.read();
            // int val = Serial.parseInt();
            float val = Serial.parseFloat();

            switch (key)
            {
            case 's':
                (int)val > 0 and (int) val < 1 ? flag.start = (int)val : Serial.println("Wrong value for start, val: " + String(val) + ", min: 0 - stop, max: 1 - start");
                flag.start = val;
                digitalWrite(ENA_PIN1, flag.start ? LOW : HIGH); // Enable/disable the driver
                digitalWrite(ENA_PIN2, flag.start ? LOW : HIGH);
                digitalWrite(EN_HIGH_VOLTAGE, flag.start ? HIGH : LOW);
                break;
            case 'd':
                (int)val > 0 and (int) val < 1 ? motorDirection = (int)val : Serial.println("Wrong value for motor direction, vauel: " + String(val) + ", min: 0 - clockwise, max: 1 - counterclockwise");
                break;
            case 'D':
                val > 0 and val < 1 ? pwmDutyCycle = val : Serial.println("Wrong value for pwm duty cycle, value: " + String(val) + ", min: 0 - 0%, max: 1 - 100%");
                break;
            case 'm':
                val > 0 and val < 100 ? motor1Speed = val : Serial.println("Wrong value for motor 1 speed, value: " + String(val) + ", min: 0 - 0%, max: 1 - 100%");
                break;
            case 'M':
                (int)val > 0 and (int) val < 200 ? motor2Speed = (int)val : Serial.println("Wrong value for motor 2 speed, value: " + String(val) + ", min: 0 - 0%, max: 1 - 100%");
                break;
            default:
                Serial.println("Wrong command! key: " + String(key) + ", value: " + String(val));
                break;
            }
        }

        wdt_reset();

        vTaskDelay(TASK_DELAY_COMMUNICATION / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Sensor task
 * - Read temperature, humidity and pressure from the BME280 sensor
 *
 * Temperature - °C
 * Humidity - %
 * Pressure - hPa
 *
 * @param pvParameters
 */
void vTaskSensor(void *pvParameters)
{
    Adafruit_BME280 bme;

    if (!bme.begin(0x76))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        vTaskDelete(NULL);
    }

    for (;;)
    {

        temperature = bme.readTemperature();
        humidity = bme.readHumidity();
        pressure = bme.readPressure() / 100.0F; // Convert in  hPa

        wdt_reset();

        vTaskDelay(TASK_DELAY_SENSOR / portTICK_PERIOD_MS);
    }
}
