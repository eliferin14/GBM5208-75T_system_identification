#include <SimpleFOC.h>

// Power supply
#define POWER_SUPPLY_VOLTAGE 15.1

// AS5600 magnetic encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
float sensor_offset = 3.75; // rad

// SimpleFOCMini driver
#define IN1 2
#define IN2 4
#define IN3 5
#define EN 15
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN);

// GBM5208-75T BLDC motor
#define POLE_PAIRS 11
#define PHASE_RESISTANCE 7.5
//#define KV
#define CURRENT_LIMIT 1.5
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);

// Commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

uint32_t t = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\n===============================================");

    // Encoder setup
    Wire.setClock(400000);
    sensor.init();
    motor.linkSensor(&sensor);

    // Driver setup
    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);

    // Motor setup
    motor.current_limit = CURRENT_LIMIT;
    motor.voltage_sensor_align = 3;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    //motor.motion_downsample = 5;
    //motor.PID_velocity.output_ramp = 100;
    
    //motor.LPF_angle = 0.01;
    motor.LPF_velocity = 0.01;

    motor.init();
 
    // Control mode configuration
    motor.controller = MotionControlType::velocity;
    motor.target = 0;
    //motor.target = PI/2 + sensor_offset;

    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    //motor.P_angle.P = 5;



    motor.init();
    motor.initFOC();

    // add target command T
    command.add('T', doTarget, "target angle");

    motor.useMonitoring(Serial);
    //motor.monitor_variables = MON_TARGET | MON_VEL | MON_ANGLE;

    Serial.println("Motor ready!");
    Serial.println("Set target velocity [rad/s]");
    _delay(1000);

    t = micros();
}

void loop() {
    //sensor.update(); Serial.println(sensor.getAngle());
    //motor.monitor();
    Serial.print( (micros()-t)/1000.0 ); Serial.print("\t"); Serial.print(motor.shaft_velocity); Serial.print("\n");

    motor.loopFOC();

    motor.move();

    // user communication
    //command.run();
}
