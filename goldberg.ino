#define IS_ALL_LOGGING_ENABLED true
#define IS_DARTBOARDSTEP_LOGGING true
#define IS_DETAILED_DARTBOARDSTEP_EXECUTION_LOGGING true
#define IS_DETAILED_LOGGING_WHILE_CALC_MEDIUM_LIGHT true
#define IS_CHEMICAL_LOGGING true
#define IS_SPRAING_PAINT_LOGGING true
#define IS_LADDER_LOGGING true
#define IS_SHIP_LOGGING true
#define IS_HAIRDRYER_LOGGING true
#define IS_PLASTIC_CUP_LOGGING true
#define IS_HAMMER_LOGGING true

#define IS_LASER_IMITATION_ENABLED false
#define IS_SPRAY_STEPMOTOR_CONTROL_BUTTONS_CONNECTED false
#define IS_DARTBOARD_SERVO_TESTING false
#define IS_DARTBOARD_PHOTORESISTOR_TESTING false

#define BASE_DARTBOARD_SERVO_DEGREE 0
#define FINAL_DARTBOARD_SERVO_DEGREE 160
#define BASE_STENCIL_BLOCKER_SERVO_DEGREE 90
#define FINAL_STENCIL_BLOCKER_SERVO_DEGREE 0
#define LIGHT_PERCENTAGE_BORDER 400
#define ALL_STEPMOTOR_DELAY 1800
#define SECONDS_TO_CALC_MEDIUM_LIGHT 5
#define MILLISECONDS_TO_TURNING_SPRAY_STEPMOTOR 2000
#define MILLISECONDS_TO_SPRAING_PAINT 3000
#define DELAY_TO_START_JACOBS_LADDER 4000
#define MILLISECONDS_TO_JACOBS_LADDER_WORKING 25000
#define MILLISECONDS_TO_SHIP_FAN_WORKING 10000
#define MILLISECONDS_TO_HAIRDRYER_WORKING 4000
#define MILLISECONDS_TO_LIT_THREADS 500
#define HEIGHT 2000
#define DELAYER 900
#define BASE 300

// Обычные пины
#define DARTBOARD_SERVO_PIN 9                 /* ШИМ выход       */
#define STENCIL_BLOCKER_SERVO_PIN 5           /* ШИМ выход       */
#define DARTBOARD_PHOTORESISTOR_PIN A0        /* аналоговый вход */
#define CHEMICAL_BEAKER_SIDE_PIN 7            /* цифровой вход   */
#define LADDER_RELAY_PIN 8                    /* цифровой выход  */
#define SPRAY_STEPMOTOR_0_PIN 10              /* цифровой выход  */
#define SPRAY_STEPMOTOR_1_PIN 11              /* цифровой выход  */
#define SPRAY_STEPMOTOR_2_PIN 12              /* цифровой выход  */
#define SPRAY_STEPMOTOR_3_PIN 13              /* цифровой выход  */
#define HAIRDRYER_RELAY_PIN 6                 /* цифровой выход  */

// Необязательные
#define LASER_BUZZER_PIN 3                    /* 3/11 ШИМ выход  */
#define LASER_IMITATION_PIN 6                 /* цифровой выход  */
#define CLOCKWISE_BUTTON_PIN 2                /* цифровой вход   */
#define ANTICLOCKWISE_BUTTON_PIN 4            /* цифровой вход   */

// Выходы для расширителя
#define PLASTIC_CUP_BUTTON_PIN 5              /* цифровой вход   */
#define PLASTIC_CUP_THREAD_RELAY_PIN 4        /* цифровой выход  */
#define SHIP_FAN_BUTTON_PIN 0                 /* цифровой вход   */
#define SHIP_FAN_RELAY_PIN 1                  /* цифровой выход  */
#define HAMMER_BUTTON_PIN 6                   /* цифровой вход   */
#define HAMMER_THREAD_RELAY_PIN 7             /* цифровой выход  */
#define HAIRDRYER_BUTTON_PIN 3                /* цифровой вход   */

// Хинты для проекта:
// Длинная нога светодиода к питанию
// звуковой пин должен быть либо 11 либо 3, потому что ШИМ блокируется при использовании пьезоэлемента
// выключатель замкнут, когда поджат к стороне с одним болтом
// analogWrite(pin(3, 5, 6, 9, 10, 11), 0-255)
// analogRead(A0 — A5) -> 0-1023
// digitalWrite((2 - 13, A0 — A5), (HIGH, LOW) = (1, 0))
// digitalRead((2 - 13, A0 — A5)) -> (HIGH, LOW) = (1, 0)
// tone((3, 11), frequency > 31)

#include <Servo.h>
#include <PCF8574.h>
#include <Wire.h>

Servo DartboardServo;
Servo StencilBlockerServo;
PCF8574 expander;
struct StepMotorPins {
    byte pin0;
    byte pin1;
    byte pin2;
    byte pin3;
};
StepMotorPins SprayStepMotor = {
    SPRAY_STEPMOTOR_0_PIN,
    SPRAY_STEPMOTOR_1_PIN,
    SPRAY_STEPMOTOR_2_PIN,
    SPRAY_STEPMOTOR_3_PIN
};

void setStepMotorPosition( byte pos, StepMotorPins motor ) {
    // Устанавливает активные магниты в моторе, тем самым приводя его в одно из 4х положений
    digitalWrite( motor.pin0, pos == 0 or pos == 1 );
    digitalWrite( motor.pin1, pos == 1 or pos == 2 );
    digitalWrite( motor.pin2, pos == 2 or pos == 3 );
    digitalWrite( motor.pin3, pos == 3 or pos == 0 );
    delayMicroseconds( ALL_STEPMOTOR_DELAY );
}

void turnStepMotorClockwise( StepMotorPins motor ) {
    // Шаговый двигатель делает один шаг по часовой стрелке
    for( char pos = 3; pos >= 0; pos-- ) {
        setStepMotorPosition( pos, motor );
    }
}

void turnStepMotorAntiClockwise( StepMotorPins motor ) {
    // Шаговый двигатель делает один шаг против часовой стрелки
    for( char pos = 0; pos <= 3; pos++ ) {
        setStepMotorPosition( pos, motor );
    }
}

void turnStepMotorInSeconds(void (*turnStepMotor)( StepMotorPins ), StepMotorPins motor, word milliseconds ) {
    Serial.println("turnStepMotorInSeconds");
    for( float elapsedTime = 0; elapsedTime <= milliseconds; elapsedTime = elapsedTime + ALL_STEPMOTOR_DELAY / 250 ) {
        turnStepMotor( motor );
        Serial.println(elapsedTime);
    }
}

void shutdownMotor( StepMotorPins motor ) {
    // Отключает все магниты, чтобы шаговый двигатель не потреблял энергию и не перегревался
    setStepMotorPosition( 255, motor );
}

int bezie4FindCord( float t, int c1, int c2, int c3, int c4 ) {
    return c1*(1-t)*(1-t)*(1-t) + 3*c2*t*(1-t)*(1-t) + 3*c3*t*t*(1-t) + c4*t*t*t;
}

void makeBlasterShootingSound() {
    noTone( LASER_BUZZER_PIN );
    for( float i = 0; i <= 1; i += 0.01 ) {
        // набор точек: (0, 0), (w, 0), (0, h), (w, h)
        tone( LASER_BUZZER_PIN, bezie4FindCord( i, BASE, BASE, HEIGHT, HEIGHT ) );
        delayMicroseconds(DELAYER);
    }
    for( float i = 0; i <= 1; i += 0.01 ) {
        // набор точек: (w, h), (2w, h), (w, 0), (2w, 0)
        tone( LASER_BUZZER_PIN, bezie4FindCord( i, HEIGHT, HEIGHT, BASE, BASE ) );
        delayMicroseconds(DELAYER);
    }
    noTone( LASER_BUZZER_PIN );
}

float getMediumDartboardLight( int seconds ) {
    #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING
        Serial.println( "Calculating medium light value started." );
    #endif
    Serial.println( "lightValueNow sumOfLightValues" );
    word sumOfLightValues = 0;
    int lightValueNow = 0;

    for( int i = 0; i <= seconds * 10; i++ ) {

        lightValueNow = analogRead( DARTBOARD_PHOTORESISTOR_PIN );
        sumOfLightValues += lightValueNow;

        #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING && IS_DETAILED_LOGGING_WHILE_CALC_MEDIUM_LIGHT
            //Serial.println( String( lightValueNow ) + " " + String( sumOfLightValues ));
            Serial.println( String( lightValueNow ));
        #endif

        delay( 100 );
    }

    float mediumLightValue = sumOfLightValues / seconds / 10;

    #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING
        Serial.println( "Medium light value from 5 seconds: " + String( mediumLightValue ) );
    #endif

    return mediumLightValue;
}

float getCriticalDartboardLight() {
    float criticalLight = getMediumDartboardLight( SECONDS_TO_CALC_MEDIUM_LIGHT ) / 100 * LIGHT_PERCENTAGE_BORDER;

    #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING
        Serial.println( "Critical light value:" + String( criticalLight ) );
    #endif

    return criticalLight;
}

void dartboardStep() {
    float criticalLight = getCriticalDartboardLight();
    int lightValueNow = analogRead( DARTBOARD_PHOTORESISTOR_PIN );
    #if IS_DARTBOARD_PHOTORESISTOR_TESTING
        while( true ) {
    #else
        while( lightValueNow <= criticalLight ) {
    #endif
        delay( 100 );
        lightValueNow = analogRead( DARTBOARD_PHOTORESISTOR_PIN );
        #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING && IS_DETAILED_DARTBOARDSTEP_EXECUTION_LOGGING
            Serial.println( "Current light: " + String( lightValueNow ) );
        #endif
    }
    #if IS_ALL_LOGGING_ENABLED && IS_DARTBOARDSTEP_LOGGING
        Serial.println( "HEADSHOT!!! Current light: " + String( lightValueNow ) );
    #endif
    delay(1000);
    DartboardServo.write( FINAL_DARTBOARD_SERVO_DEGREE - BASE_DARTBOARD_SERVO_DEGREE ); // количество градусов поворота
}

void plasticCupStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_PLASTIC_CUP_LOGGING
        Serial.println( "Waiting for pressing plastic cup button..." );
    #endif

    while ( expander.digitalRead( PLASTIC_CUP_BUTTON_PIN ) ) {
        delay( 100 );
    }
    #if IS_ALL_LOGGING_ENABLED && IS_PLASTIC_CUP_LOGGING
        Serial.println( "Plastic cup button pressed. Thread litted." );
    #endif
    expander.digitalWrite( PLASTIC_CUP_THREAD_RELAY_PIN, LOW );
    delay( MILLISECONDS_TO_LIT_THREADS );
    expander.digitalWrite( PLASTIC_CUP_THREAD_RELAY_PIN, HIGH );
    #if IS_ALL_LOGGING_ENABLED && IS_PLASTIC_CUP_LOGGING
        Serial.println( "Thread extinguished." );
    #endif
}

void shipStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_SHIP_LOGGING
        Serial.println( "Waiting for pressing ship fan button..." );
    #endif
    while ( expander.digitalRead( SHIP_FAN_BUTTON_PIN ) ) {
        delay( 100 );
    }
    #if IS_ALL_LOGGING_ENABLED && IS_SHIP_LOGGING
        Serial.println( "Ship fan button pressed. Ship fan enabled." );
    #endif
    expander.digitalWrite( SHIP_FAN_RELAY_PIN, LOW );
    delay( MILLISECONDS_TO_SHIP_FAN_WORKING );
    expander.digitalWrite( SHIP_FAN_RELAY_PIN, HIGH );
    #if IS_ALL_LOGGING_ENABLED && IS_SHIP_LOGGING
        Serial.println( "Ship fan disabled." );
    #endif
}

void hammerStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_HAMMER_LOGGING
        Serial.println( "Waiting for pressing hammer button..." );
    #endif
    while ( expander.digitalRead( HAMMER_BUTTON_PIN ) ) {
        delay( 100 );
    }
    #if IS_ALL_LOGGING_ENABLED && IS_HAMMER_LOGGING
        Serial.println( "Hammer button pressed. Thread litted." );
    #endif
    expander.digitalWrite( HAMMER_THREAD_RELAY_PIN, LOW );
    delay( MILLISECONDS_TO_LIT_THREADS );
    expander.digitalWrite( HAMMER_THREAD_RELAY_PIN, HIGH );
    #if IS_ALL_LOGGING_ENABLED && IS_HAMMER_LOGGING
        Serial.println( "Thread extinguished." );
    #endif
}

void hairdryerStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_HAIRDRYER_LOGGING
        Serial.println( "Waiting for pressing hairdryer button..." );
    #endif
    while ( expander.digitalRead( HAIRDRYER_BUTTON_PIN ) ) {
        delay( 100 );
    }
    #if IS_ALL_LOGGING_ENABLED && IS_HAIRDRYER_LOGGING
        Serial.println( "Hairdryer button pressed. Hairdryer enabled." );
    #endif
    digitalWrite( HAIRDRYER_RELAY_PIN, HIGH );
    delay( MILLISECONDS_TO_HAIRDRYER_WORKING );
    digitalWrite( HAIRDRYER_RELAY_PIN, LOW );
    #if IS_ALL_LOGGING_ENABLED && IS_HAIRDRYER_LOGGING
        Serial.println( "Hairdryer disabled." );
    #endif
}

void chemicalStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_CHEMICAL_LOGGING
        Serial.println( "Waiting for reaction..." );
    #endif
    while( digitalRead( CHEMICAL_BEAKER_SIDE_PIN ) ) {
        delay( 100 );
    }
    #if IS_ALL_LOGGING_ENABLED && IS_CHEMICAL_LOGGING
        Serial.println( "A reaction occured." );
    #endif
}

void sprayingStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_SPRAING_PAINT_LOGGING
        Serial.println( "Pulling the thread started." );
    #endif
    turnStepMotorInSeconds( turnStepMotorClockwise, SprayStepMotor, MILLISECONDS_TO_TURNING_SPRAY_STEPMOTOR );
    shutdownMotor( SprayStepMotor );
    #if IS_ALL_LOGGING_ENABLED && IS_SPRAING_PAINT_LOGGING
        Serial.println( "Pulling the thread finished." );
    #endif
    delay( MILLISECONDS_TO_SPRAING_PAINT );
    #if IS_ALL_LOGGING_ENABLED && IS_SPRAING_PAINT_LOGGING
        Serial.println( "Weakening of the thread started." );
    #endif
    turnStepMotorInSeconds( turnStepMotorAntiClockwise, SprayStepMotor, MILLISECONDS_TO_TURNING_SPRAY_STEPMOTOR );
    shutdownMotor( SprayStepMotor );
    #if IS_ALL_LOGGING_ENABLED && IS_SPRAING_PAINT_LOGGING
        Serial.println( "Weakening of the thread finished." );
    #endif
    StencilBlockerServo.write( FINAL_STENCIL_BLOCKER_SERVO_DEGREE - BASE_STENCIL_BLOCKER_SERVO_DEGREE );
}

void jacobsLadderStep() {
    #if IS_ALL_LOGGING_ENABLED && IS_LADDER_LOGGING
        Serial.println( "Jacob's ladder is lit." );
    #endif
    digitalWrite( LADDER_RELAY_PIN, LOW );
    delay( MILLISECONDS_TO_JACOBS_LADDER_WORKING );
    digitalWrite( LADDER_RELAY_PIN, HIGH );
    #if IS_ALL_LOGGING_ENABLED && IS_LADDER_LOGGING
        Serial.println( "Jacob's ladder extinguished." );
    #endif
}

void setup() {
    #if IS_ALL_LOGGING_ENABLED
        Serial.begin( 115200 );
    #endif
    expander.begin( 0x20 );

    DartboardServo.attach( DARTBOARD_SERVO_PIN );
    DartboardServo.write( BASE_DARTBOARD_SERVO_DEGREE );
    StencilBlockerServo.attach( STENCIL_BLOCKER_SERVO_PIN );
    StencilBlockerServo.write( BASE_STENCIL_BLOCKER_SERVO_DEGREE );

    pinMode( SPRAY_STEPMOTOR_0_PIN, OUTPUT );
    pinMode( SPRAY_STEPMOTOR_1_PIN, OUTPUT );
    pinMode( SPRAY_STEPMOTOR_2_PIN, OUTPUT );
    pinMode( SPRAY_STEPMOTOR_3_PIN, OUTPUT );
    shutdownMotor( SprayStepMotor );
    #if IS_LASER_IMITATION_ENABLED
        pinMode( LASER_IMITATION_PIN, OUTPUT );
        analogWrite( LASER_IMITATION_PIN, 130 );
    #endif
    #if IS_SPRAY_STEPMOTOR_CONTROL_BUTTONS_CONNECTED
        pinMode( CLOCKWISE_BUTTON_PIN, INPUT_PULLUP );
        pinMode( ANTICLOCKWISE_BUTTON_PIN, INPUT_PULLUP );
        while( true ) {
            if( !digitalRead( CLOCKWISE_BUTTON_PIN ) ) {
                turnStepMotorClockwise( SprayStepMotor );
            } else if( !digitalRead( ANTICLOCKWISE_BUTTON_PIN ) ) {
                turnStepMotorAntiClockwise( SprayStepMotor );
            } else {
                shutdownMotor( SprayStepMotor );
                delay( 100 );
            }
        }
    #endif
    #if IS_DARTBOARD_SERVO_TESTING
        while( true ) {
            StencilBlockerServo.write( 0 );
            Serial.println( "0" );
            delay(6000);
            StencilBlockerServo.write( 90 );
            Serial.println( "90" );
            delay(1000);
            StencilBlockerServo.write( 180 );
            Serial.println( "180" );
            delay(1000);
        }
    #endif

    pinMode( CHEMICAL_BEAKER_SIDE_PIN, INPUT_PULLUP );
    pinMode( DARTBOARD_PHOTORESISTOR_PIN, INPUT );

    pinMode( LADDER_RELAY_PIN, OUTPUT );
    pinMode( HAIRDRYER_RELAY_PIN, OUTPUT );
    digitalWrite( HAIRDRYER_RELAY_PIN, LOW );
    digitalWrite( LADDER_RELAY_PIN, HIGH );

    expander.pinMode( SHIP_FAN_BUTTON_PIN, INPUT );
    expander.pinMode( HAIRDRYER_BUTTON_PIN, INPUT );
    expander.pinMode( PLASTIC_CUP_BUTTON_PIN, INPUT );
    expander.pinMode( HAMMER_BUTTON_PIN, INPUT );

    expander.pinMode( SHIP_FAN_RELAY_PIN, OUTPUT );
    expander.pinMode( PLASTIC_CUP_THREAD_RELAY_PIN, OUTPUT );
    expander.pinMode( HAMMER_THREAD_RELAY_PIN, OUTPUT );
    expander.set();

    dartboardStep();
    plasticCupStep();
    shipStep();
    hammerStep();
    hairdryerStep();
    chemicalStep();
    sprayingStep();
    delay( DELAY_TO_START_JACOBS_LADDER );
    jacobsLadderStep();
}

void loop() {}
