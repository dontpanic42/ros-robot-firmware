
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
#define LW_SPEED_TOPIC_NAME "lw_speed"
// ROS topic for right wheel speed
#define RW_SPEED_TOPIC_NAME "rw_speed"
// ROS topic for left wheel encoder ticks
#define LW_ENCODER_TICKS_TOPIC_NAME "lwheel_ticks"
// ROS topic for left wheel encoder ticks
#define RW_ENCODER_TICKS_TOPIC_NAME "rwheel_ticks"


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 lw_encoder_ticks_msg;
std_msgs::Int32 rw_encoder_ticks_msg;

// Killswitch
unsigned long lastSpeedMessage;
int16_t lastLWSpeed = 0;
int16_t lastRWSpeed = 0;

// Time of the last publish of messages
unsigned long lastPublish;

struct MotorTickCounter {
  uint8_t pin1, pin2; 
  bool lastState1, lastState2;
  uint16_t tickCount;

  MotorTickCounter(int pin1, int pin2)
  : pin1(pin1)
  , pin2(pin2)
  , tickCount(0)
  {}

  void init () {
    pinMode(this->pin1, INPUT);
    pinMode(this->pin2, INPUT);
    this->lastState1 = digitalRead(this->pin1);
    this->lastState2 = digitalRead(this->pin2);
  }

  void update() {
    int state1 = digitalRead(this->pin1);
    int state2 = digitalRead(this->pin2);

    if(this->lastState1 != state1 || this->lastState2 != state2) {
      this->lastState1 = state1;
      this->lastState2 = state2;
      tickCount++;
    }
  }

  int readTicks() {
    return this->tickCount;
  }
};

// Executed when a message is received in the LW_SPEED_TOPIC_NAME topic
void ros_callback_lwspeed(const std_msgs::Int16 &msg)
{
  // Update timing so we now the connection still works
  lastSpeedMessage = millis();

  // Read speed data from message
  int16_t in = msg.data;

  // When the speed did not change, do return. This is required
  // to have the motors running smooth 
  if (in == lastLWSpeed) return;
  lastLWSpeed = in;

  // Check the direction. When value < 0 reverse, otherwise forward
  if(in > 0) {
    // Forward direction
    in = in > MOTOR_MAX_VALUE ? MOTOR_MAX_VALUE : in;
    analogWrite(PIN_MOTOR_IN_1_A, 0);
    analogWrite(PIN_MOTOR_IN_2_A, in);
  } else {
    // Backward direction
    in = in < -MOTOR_MAX_VALUE ? -MOTOR_MAX_VALUE : in;
    analogWrite(PIN_MOTOR_IN_1_A, in);
    analogWrite(PIN_MOTOR_IN_2_A, 0);
  } 

  // Update timing so we now the connection still works
  lastSpeedMessage = millis();
}

// Executed when a message is received in the RW_SPEED_TOPIC_NAME topic
void ros_callback_rwspeed(const std_msgs::Int16 &msg)
{
  // Update timing so we now the connection still works
  lastSpeedMessage = millis();

  // Read speed data from message
  int16_t in = msg.data;

  // When the speed did not change, do return. This is required
  // to have the motors running smooth 
  if (in == lastRWSpeed) return;
  lastRWSpeed = in;

  // Check the direction. When value < 0 reverse, otherwise forward
  if(in > 0) {
    // Forward direction
    in = in > MOTOR_MAX_VALUE ? MOTOR_MAX_VALUE : in;
    analogWrite(PIN_MOTOR_IN_1_B, in);
    analogWrite(PIN_MOTOR_IN_2_B, 0);
  } else {
    // Backward direction
    in = in < -MOTOR_MAX_VALUE ? -MOTOR_MAX_VALUE : in;
    analogWrite(PIN_MOTOR_IN_1_B, 0);
    analogWrite(PIN_MOTOR_IN_2_B, in);
  } 

  // Update timing so we now the connection still works
  lastSpeedMessage = millis();
}

ros::Subscriber<std_msgs::Int16> sub_lw_speed(LW_SPEED_TOPIC_NAME, &ros_callback_lwspeed);
ros::Subscriber<std_msgs::Int16> sub_rw_speed(RW_SPEED_TOPIC_NAME, &ros_callback_rwspeed);

ros::Publisher lw_encoder_ticks(LW_ENCODER_TICKS_TOPIC_NAME, &lw_encoder_ticks_msg);
ros::Publisher rw_encoder_ticks(RW_ENCODER_TICKS_TOPIC_NAME, &rw_encoder_ticks_msg);

MotorTickCounter lw_tick_counter(PIN_MOTOR_HALL_1_A, PIN_MOTOR_HALL_2_B);
MotorTickCounter rw_tick_counter(PIN_MOTOR_HALL_2_A, PIN_MOTOR_HALL_2_B);

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
  nh.subscribe(sub_lw_speed);
  // Subscribe to the right wheel speed topic
  nh.subscribe(sub_rw_speed);
  
  // Advertise lw encoder ticks topic 
  nh.advertise(lw_encoder_ticks);
  // Advertise rw encoder ticks topic 
  nh.advertise(rw_encoder_ticks);

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
    lw_encoder_ticks_msg.data = lw_tick_counter.readTicks();
    rw_encoder_ticks_msg.data = rw_tick_counter.readTicks();
     
    lw_encoder_ticks.publish(&lw_encoder_ticks_msg);
    rw_encoder_ticks.publish(&rw_encoder_ticks_msg);

    lastPublish = currentMillis;
  }

  // Required for the connection to rosserial_client to work
  nh.spinOnce();
}
