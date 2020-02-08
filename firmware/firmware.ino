
// Motor control pins, ie. outputs
#define PIN_MOTOR_IN_1_B 6
#define PIN_MOTOR_IN_1_A 3
#define PIN_MOTOR_IN_2_B 5
#define PIN_MOTOR_IN_2_A 9
#define PIN_MOTOR_EN_B 10
#define PIN_MOTOR_EN_A 11
// Hall Sensors from motor, ie. inputs
#define PIN_MOTOR_HALL_1_A 2
#define PIN_MOTOR_HALL_2_A 4
#define PIN_MOTOR_HALL_1_B 7
#define PIN_MOTOR_HALL_2_B 8

// The motor driver does not like full duty cycle, so clamp it at ~90%
#define MOTOR_MAX_VALUE 230
// To prevent accidents when the connection is lost, disable the motors 
// when we did not get a speed message after X ms
#define MAX_TIME_BETWEEN_MESSAGES_MS 1000
// How ofthen should we publish messages in ms
#define TIME_BETWEEN_PUBLISH 100

// ROS topic for left wheel speed
#define MC_OUTPUT_TOPIC "mco"
// ROS topic for left wheel encoder ticks
#define MC_STATUS_TOPIC "mcs"

#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))  

#include <ros.h>
#include <motor_pkg/motorcontroller_output.h>
#include <motor_pkg/motorcontroller_status.h>

ros::NodeHandle  nh;
motor_pkg::motorcontroller_status status_msg;

// Killswitch
unsigned long lastSpeedMessage;
int16_t lastLWSpeed = 0;
int16_t lastRWSpeed = 0;

// Time of the last publish of messages
unsigned long lastPublish;

template<uint8_t INPUT_PIN_1, uint8_t INPUT_PIN_2>
struct MotorTickCounter {

  uint8_t state;
  uint16_t tickCount;

  MotorTickCounter()
  : tickCount(0)
  , state(0)
  {}

  void init () {
    pinMode(INPUT_PIN_1, INPUT);
    pinMode(INPUT_PIN_2, INPUT);
    if(digitalRead(INPUT_PIN_1)) BIT_SET(state, 0); else BIT_CLEAR(state, 0);
    if(digitalRead(INPUT_PIN_2)) BIT_SET(state, 1); else BIT_CLEAR(state, 1);
  }

  void update() {
    uint8_t newState = 0;
    if(digitalRead(INPUT_PIN_1)) BIT_SET(newState, 0); else BIT_CLEAR(newState, 0);
    if(digitalRead(INPUT_PIN_2)) BIT_SET(newState, 1); else BIT_CLEAR(newState, 1);

    if(state != newState) {
      state = newState;
      tickCount++;
    }
  }

  uint16_t readTicks() {
    return this->tickCount;
  }
};

// Executed when a message is received in the LW_SPEED_TOPIC_NAME topic
void set_lwspeed(const int16_t & in)
{
  // Update timing so we now the connection still works
  lastSpeedMessage = millis();

  // When the speed did not change, do return. This is required
  // to have the motors running smooth 
  if (in == lastLWSpeed) return;
  lastLWSpeed = in;

  // Check the direction. When value < 0 reverse, otherwise forward
  if(in > 0) {
    // Forward direction
    analogWrite(PIN_MOTOR_IN_1_A, 0);
    analogWrite(PIN_MOTOR_IN_2_A, in > MOTOR_MAX_VALUE ? MOTOR_MAX_VALUE : in);
  } else {
    // Backward direction
    analogWrite(PIN_MOTOR_IN_1_A, in < -MOTOR_MAX_VALUE ? -MOTOR_MAX_VALUE : in);
    analogWrite(PIN_MOTOR_IN_2_A, 0);
  } 

  // Update timing so we now the connection still works
  lastSpeedMessage = millis();
}

// Executed when a message is received in the RW_SPEED_TOPIC_NAME topic
void set_rwspeed(const int16_t & in)
{

  // When the speed did not change, do return. This is required
  // to have the motors running smooth 
  if (in == lastRWSpeed) return;
  lastRWSpeed = in;

  // Check the direction. When value < 0 reverse, otherwise forward
  if(in > 0) {
    // Forward direction
    analogWrite(PIN_MOTOR_IN_1_B, in > MOTOR_MAX_VALUE ? MOTOR_MAX_VALUE : in);
    analogWrite(PIN_MOTOR_IN_2_B, 0);
  } else {
    // Backward direction
    analogWrite(PIN_MOTOR_IN_1_B, 0);
    analogWrite(PIN_MOTOR_IN_2_B, in < -MOTOR_MAX_VALUE ? -MOTOR_MAX_VALUE : in);
  } 
}

void ros_output_callback(const motor_pkg::motorcontroller_output & mco) {
  set_lwspeed(mco.lw_speed);
  set_rwspeed(mco.rw_speed);
  
  // Update timing so we now the connection still works
  lastSpeedMessage = millis();
}

ros::Subscriber<motor_pkg::motorcontroller_output> sub_output(MC_OUTPUT_TOPIC, &ros_output_callback);
ros::Publisher pub_status(MC_STATUS_TOPIC, &status_msg);

MotorTickCounter<PIN_MOTOR_HALL_1_A, PIN_MOTOR_HALL_2_B> lw_tick_counter;
MotorTickCounter<PIN_MOTOR_HALL_2_A, PIN_MOTOR_HALL_2_B> rw_tick_counter;

void setup() {
  // Setup pins
  pinMode(PIN_MOTOR_IN_1_B, OUTPUT);
  pinMode(PIN_MOTOR_IN_1_A, OUTPUT);
  pinMode(PIN_MOTOR_IN_2_B, OUTPUT);
  pinMode(PIN_MOTOR_IN_2_A, OUTPUT);
  pinMode(PIN_MOTOR_EN_B,   OUTPUT);
  pinMode(PIN_MOTOR_EN_A,   OUTPUT);

  // Setup initial values (i.e. enable motors but set
  // the speed to zero)
  analogWrite(PIN_MOTOR_IN_1_B, 0);
  analogWrite(PIN_MOTOR_IN_1_A, 0);
  analogWrite(PIN_MOTOR_IN_2_B, 0);
  analogWrite(PIN_MOTOR_IN_2_A, 0);
  digitalWrite(PIN_MOTOR_EN_B,  HIGH);
  digitalWrite(PIN_MOTOR_EN_A,  HIGH);

  // By default, rosserial uses 57600 baud, so configure the arduino
  // to use that
  nh.getHardware()->setBaud(57600);
  // Initialize the rosserial node
  nh.initNode();
  // Subscribe to the left wheel speed topic
  nh.subscribe(sub_output);
  // Advertise lw encoder ticks topic 
  nh.advertise(pub_status);

  // Reset the killswitch
  lastSpeedMessage = lastPublish = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read motor sensors
  lw_tick_counter.update();
  rw_tick_counter.update();
  
  // Check if there was at least one speed message in the last MAX_TIME_BETWEEN_MESSAGES_MS ms
  // if not, assume connection loss and stop motors to preven accidents (killswitch)
  if((currentMillis - lastSpeedMessage) > MAX_TIME_BETWEEN_MESSAGES_MS) {
    analogWrite(PIN_MOTOR_IN_1_B, 0);
    analogWrite(PIN_MOTOR_IN_1_A, 0);
    analogWrite(PIN_MOTOR_IN_2_B, 0);
    analogWrite(PIN_MOTOR_IN_2_A, 0);
    lastLWSpeed = 0;
    lastRWSpeed = 0;
  }

  // Check if it's time to publish our data
  if((currentMillis - lastPublish) > TIME_BETWEEN_PUBLISH) {
    status_msg.lw_ticks = lw_tick_counter.readTicks();
    status_msg.rw_ticks = rw_tick_counter.readTicks();
    status_msg.stamp = nh.now();
    pub_status.publish(&status_msg);

    lastPublish = currentMillis;
  }

  // Required for the connection to rosserial_client to work
  nh.spinOnce();
}
