#include <Arduino.h>
#include <motor_parameters.hpp>
#include <SimpleFOC.h>

// AS5600 magnetic encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// SimpleFOCMini driver
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN);

// GBM5208-75T motor
BLDCMotor motor = BLDCMotor(POLE_PAIRS);  // NB: for pure voltage control, do not pass the phase resistance!

// Starting time of the experiment
uint32_t t_start = 0;
uint32_t t_loop = 0;

float sinusoidal_frequencies[] = {1,2,3,10};
float amplitude = 1;
float computeInputSignal(uint32_t t_micros) {
    float t_seconds = ((float)t_micros)/1000000.0;
    float u = 0;
    for (float freq : sinusoidal_frequencies) {
        u += amplitude * sin(2*PI*freq*t_seconds);
    }
    return u;
}

void setup() {
    Serial.begin(115200);

    // Sensor setup 
    Wire.setClock(400000);
    sensor.init();

    // Driver setup
    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();

    // Motor setup
        // Set the current limit to avoid overheating
        motor.current_limit = CURRENT_LIMIT;
        
        // Initial position detection (I think) 
        motor.voltage_sensor_align = 3;

        // Set modulation to space vectors (why not? that's cool)
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

        // Low pass filters
        //motor.LPF_velocity = 0.01;
        //motor.LPF_angle = 0.01;

        // Control type
        motor.controller = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::voltage;
        motor.target = 1;

        // Link the sensor and the driver 
        motor.linkSensor(&sensor);
        motor.linkDriver(&driver);

        // Setup monitoring
        motor.useMonitoring(Serial);

        // Start the motor
        motor.init();
        motor.initFOC();

    _delay(1000);

    // Start the stopwatch
    t_start = micros();
    t_loop = t_start;
}

void loop() {
    // Compute the time from the start of the experiment
    uint32_t t_micros = micros() - t_start;

    // Compute the sum of sinusoidal signals
    float u = computeInputSignal(t_micros);

    // Control loop
    motor.loopFOC();
    motor.move(u);

    // Print data to serial
    if ( micros()-t_loop > 10000 ) {
        Serial.print(t_micros); Serial.print("\t"); Serial.print(u); Serial.print("\t"); Serial.print(motor.voltage.q); Serial.print("\t"); Serial.print(motor.shaft_velocity); Serial.print("\t"); Serial.print(motor.shaft_angle); Serial.print("\t"); Serial.println();
        //motor.monitor();
        //Serial.print("\t"); Serial.println( micros()-t );
        t_loop = micros();
    }
}
