#include "main.h"

void vTaskEncoder(void *pvParameters);
void vTaskMotor(void *pvParameters);
void vTaskLCD(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vTaskSensor(void *pvParameters);

// Motor constants
float motor1Speed = 0.1;      // Schimbat la float pentru a permite valori fracționare
uint8_t motor2Speed = 25;     // Viteza motorului
uint8_t motorDirection = LOW; // Direcția motorului

// Transformer variables
uint8_t pwmValue = 0;

float voltage = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;

// Encoder și LCD
// Create a global pointer for the encoder object
Encoder enc(CLK, DT, SW, TYPE1);
LiquidCrystal_I2C lcd(ADDRESS, 16, 2);
MWCSTEPPER tb6600_m1(ENA_PIN1, DIR_PIN1, PUL_PIN1); // Inițializează driverul motorului

AF_DCMotor motor(1);

struct Flag
{
    uint8_t start = 0; // Starea de start/stop
} flag;

void setup()
{
    Serial.begin(115200);

    wdt_enable(WDTO_2S);

    // Crearea sarcinilor
    xTaskCreate(vTaskEncoder, "Encoder Task", 512, NULL, 1, NULL);
    xTaskCreate(vTaskMotor, "Motor Task", 512, NULL, 1, NULL);
    xTaskCreate(vTaskLCD, "LCD Task", 256, NULL, 1, NULL);
    xTaskCreate(vTaskCommunication, "Communication Task", 128, NULL, 1, NULL);
    xTaskCreate(vTaskSensor, "Sensor Task", 256, NULL, 1, NULL);

    // Pornește scheduler-ul FreeRTOS
    vTaskStartScheduler();

    // Setare Timer0 pentru a obține o frecvență de aproximativ 7kHz
    pinMode(EN_HIGH_VOLTAGE, OUTPUT);

    // Initializarea LCD
    lcd.init();
    lcd.backlight();

    motor.setSpeed(motor2Speed);
    motor.run(RELEASE);
}

void loop() {}

void vTaskLCD(void *pvParameters)
{
    // Initializarea LCD
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
            lcd.print(String(motor1Speed, 2)); // Afișează cu două zecimale
            break;

        case SELECT_MOTOR2_SPEED:
            lcd.setCursor(0, 0);
            lcd.print("Motor 2 speed:");
            lcd.setCursor(0, 1);
            lcd.print(int(motor2Speed)); // Afișează viteza curentă
            break;

        case SENSOR_VALUES:
            lcd.setCursor(0, 0);
            lcd.print("V_out:" + String(int(voltageValue(analogRead(ADC_PIN)) / 1000.0))+"kV");
            lcd.setCursor(0, 1);
            lcd.print("T:" + String((int)(temperature)));
            lcd.setCursor(5, 1);
            lcd.print("H:" + String((int)(humidity)));
            lcd.setCursor(10, 1);
            lcd.print("P:" + String((int)(pressure)));
            lcd.setCursor(11, 0);
            lcd.print(flag.start ? "Start" : "Stop");
            break;
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // Actualizează LCD-ul la fiecare jumătate de secundă
    }
}

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

            tb6600_m1.run(); // Rotește motorul la viteza setată
            motor.setSpeed(motor2Speed);
            motor.run(motorDirection == HIGH ? FORWARD : BACKWARD);
        }
        else
        {
            digitalWrite(RELAY_PIM, LOW);

            tb6600_m1.active(false); // Oprește motorul
            motor.run(RELEASE);
        }
        wdt_reset(); // Resetare watchdog

        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay pentru reacții rapide
    }
}

void vTaskEncoder(void *pvParameters){
    for (;;)
    {
        enc.tick();

        if (enc.isSingle())
        {
            // Schimbă starea meniului la apăsare
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
            // Resetare la starea inițială
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
        else if (currentState == SELECT_DIRECTION)
        {
            if (enc.isRight())
            {
                motorDirection = LOW; // Forward
                Serial.println("Direction: Forward");
            }
            else if (enc.isLeft())
            {
                motorDirection = HIGH; // Backward
                Serial.println("Direction: Backward");
            }
        }
        else if (currentState == SELECT_MOTOR1_SPEED)
        {
            if (enc.isLeft())
            {
                // Crește viteza
                if (motor1Speed >= 1)
                {
                    motor1Speed++; // Crește cu 1 dacă este >= 1
                    if (motor1Speed > 100)
                        motor1Speed = 100; // Limitează viteza maximă
                }
                else if (motor1Speed >= 0.1 && motor1Speed < 1)
                {
                    motor1Speed += 0.1; // Crește cu 0.1 dacă este între 0.1 și 1
                }

                Serial.println("Speed: " + String(motor1Speed, 2)); // Afișează cu două zecimale
            }
            else if (enc.isRight())
            {
                // Scade viteza
                if (motor1Speed >= 1)
                {
                    motor1Speed--;                                      // Scade cu 1 dacă este >= 1
                    Serial.println("Speed: " + String(motor1Speed, 2)); // Afișează cu două zecimale
                }
                else if (motor1Speed < 1 && motor1Speed >= 0.1)
                {
                    motor1Speed -= 0.1; // Scade cu 0.1 dacă este între 0.1 și 1
                    if (motor1Speed < 0.1)
                        motor1Speed = 0.1; // Limitează viteza minimă la 0.1

                    Serial.println("Speed: " + String(motor1Speed, 2)); // Afișează cu două zecimale
                }
            }
        }
        else if (currentState == SELECT_MOTOR2_SPEED)
        {
            if (enc.isLeft())
            {
                motor2Speed++;
                if (motor2Speed > 200)
                    motor2Speed = 200; // Limitează viteza maximă
                Serial.println("Speed: " + String(motor2Speed));
            }
            else if (enc.isRight())
            {
                motor2Speed--;
                if (motor2Speed < 1)
                    motor2Speed = 1; // Limitează viteza minimă
                Serial.println("Speed: " + String(motor2Speed));
            }
        }

        wdt_reset(); // Resetare watchdog

        vTaskDelay(30 / portTICK_PERIOD_MS); // Întârziere pentru reacții rapide
    }
}

void vTaskCommunication(void *pvParameters)
{
    Serial.println("System Initialized\r\n s - start, d - direction, m - motor 1 speed, M - motor 2 speed\r\n");

    for (;;)
    {
        if (Serial.available() > 0)
        {
            char key = Serial.read();
            int val = Serial.parseInt();

            switch (key)
            {
            case 's':
                flag.start = val;
                digitalWrite(ENA_PIN1, flag.start ? LOW : HIGH); // Activează/dezactivează driverul
                digitalWrite(ENA_PIN2, flag.start ? LOW : HIGH);
                digitalWrite(EN_HIGH_VOLTAGE, flag.start ? HIGH : LOW);
                break;
            case 'd':
                motorDirection = val ? HIGH : LOW;
                break;
            case 'm':
                motor1Speed = val;
                break;
            case 'M':
                motor2Speed = val;
                break;
            default:
                Serial.println("Wrong command! key: " + String(key) + ", val: " + String(val));
                break;
            }
        }

        wdt_reset();

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void vTaskSensor(void *pvParameters)
{
    Adafruit_BME280 bme; // Crearea obiectului BME280

    if (!bme.begin(0x76))
    { // Verifică adresa I2C
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        vTaskDelete(NULL); // Oprește task-ul dacă senzorul nu este găsit
    }

    for (;;)
    {
        // Citirea valorilor de la BME280
        temperature = bme.readTemperature(); // Asigură-te că ai inițializat senzorul BME280
        humidity = bme.readHumidity();
        pressure = bme.readPressure() / 100.0F; // Convertim la hPa

        // Afișează valorile cu o singură zecimală
        // Serial.print("Temperature: ");
        // Serial.print((int)(temperature * 10) / 10.0); // Rotunjire la o zecimală
        // Serial.print(" °C, Pressure: ");
        // Serial.print((int)(pressure * 10) / 10.0); // Rotunjire la o zecimală
        // Serial.print(" hPa, Humidity: ");
        // Serial.print((int)(humidity * 10) / 10.0); // Rotunjire la o zecimală
        // Serial.println(" %");

        wdt_reset();

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Actualizează datele la fiecare 2 secunde
    }
}
