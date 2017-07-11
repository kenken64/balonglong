#define DEBUG         // uncomment this statement if serial debug output needed

/* ============================= Line Follower ============================= */
// jolliBot: Motor 1 for right motor and Motor 2 for left motor

#define MOTOR1_F 10     // Motor 1 forward
#define MOTOR1_R 11     // Motor 1 reverse
#define MOTOR2_F 3      // Motor 2 forward
#define MOTOR2_R 9      // Motor 2 reverse

#define RIGHT_HEADLIGHT 4
#define LEFT_HEADLIGHT 5

#define NUM_SENSORS  5

#define M1_DEFAULT_SPEED 160   // Right default motor speed
#define M2_DEFAULT_SPEED 160   // Left default motor speed

#define M1_MAX_SPEED 220       // Right max motor speed
#define M2_MAX_SPEED 220       // Left max motor speed

boolean b_sensor[NUM_SENSORS];     // variable for holding current sensor readings  //if using digital method
double prev_position;

int thresholdValue = 400;

long sensors[NUM_SENSORS];  // Array used to store readings for sensors.

long sensors_average;
double sensors_sum;
double position;
int last_proportional;

int calmax[NUM_SENSORS];   // Array used to store max readings for sensors.
int calmin[NUM_SENSORS];   // Array used to store min readings for sensors.


/* ============================= Audio ============================= */
#define SPEAKER_OUT 8

#define mC 1911
#define mC1 1804
#define mD 1703
#define mEb 1607
#define mE 1517
#define mF 1432
#define mF1 1352
#define mG 1276
#define mAb 1204
#define mA 1136
#define mBb 1073
#define mB 1012
#define mc 955
#define mc1 902
#define md 851
#define meb 803
#define me 758
#define mf 716
#define mf1 676
#define mg 638
#define mab 602
#define ma 568
#define mbb 536
#define mb 506

#define mp 0  //pause


// MELODY and TIMING  =======================================
//  melody[] is an array of notes, accompanied by beats[],
//  which sets each note's relative length (higher #, longer note)
int melody[] = {mg, mab , mE};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int beats[]  = {4, 8 , 8};

int MAX_COUNT = sizeof(melody) / sizeof(int); // Melody length, for looping.

// Set overall tempo
long tempo = 10000;
// Set length of pause between notes
int pause = 1000;
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES

// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;


void TriggerSound();
void set_motorsLF(int motor1speed, int motor2speed);
void set_motors(int motor1speed, int motor2speed);

//**********************************************************************************************************************************************************
void setup()
{
  delay(1000);  // Short delay so motors will not be activated if Arduino is in the process of program code download.

  Serial.begin(115200);

  // initialize the motor pins as output:
  pinMode(MOTOR1_F, OUTPUT);
  pinMode(MOTOR1_R, OUTPUT);
  pinMode(MOTOR2_F, OUTPUT);
  pinMode(MOTOR2_R, OUTPUT);

  // Initialize the LED headlight pin as an output:
  pinMode(RIGHT_HEADLIGHT, OUTPUT);
  pinMode(LEFT_HEADLIGHT, OUTPUT);

  // Turn off headlights
  digitalWrite(RIGHT_HEADLIGHT, HIGH);
  digitalWrite(LEFT_HEADLIGHT, HIGH);

  // Initialize the speaker pin as an output:
  pinMode(SPEAKER_OUT, OUTPUT);

  // Warning sound before motors activated
  for (int i = 0; i < 5; i++)
  {
    TriggerSound();
    delay(500);
  }

  // Rotate right and left on the spot
  for(int counter=0;counter<100;counter++)
  {
    if(counter < 25 || counter >= 75)
      set_motors(80,-80);
    else
      set_motors(-80,80);
 
    // This function records a set of sensor readings and keeps track of the minimum and maximum values encountered.
    line_sensors_scan();

    delay(20);
  }

  set_motors(0, 0);
  #ifdef DEBUG  
    Serial.println("Sensor Maximum Values");
    Serial.print(calmax[0]);
    Serial.print("\t");
    Serial.print(calmax[1]);
    Serial.print("\t");
    Serial.print(calmax[2]);
    Serial.print("\t");
    Serial.print(calmax[3]);
    Serial.print("\t");
    Serial.println(calmax[4]);
    Serial.println(" ");

    Serial.println("Sensor Minimum Values");
    Serial.print(calmin[0]);
    Serial.print("\t");
    Serial.print(calmin[1]);
    Serial.print("\t");
    Serial.print(calmin[2]);
    Serial.print("\t");
    Serial.print(calmin[3]);
    Serial.print("\t");
    Serial.println(calmin[4]);
    Serial.println(" ");
  #endif
}


void PID_program()
{
  checkSensors();
  pid_calc();
}

void lineFollow(void) {
  PID_program();
}

//**********************************************************************************************************************************************************
void loop()
{
  // Turn on headlights
  digitalWrite(RIGHT_HEADLIGHT, LOW);
  digitalWrite(LEFT_HEADLIGHT, LOW);
  // follow line using PID algorithm
  lineFollow();
}

void pid_calc() {
  const int max = 160;
  int integral;
  position = int(sensors_average / sensors_sum);
  // hack if the robot is confused refer to the previous calculated position.
  if((b_sensor[0] == 0 && b_sensor[1] == 0 && b_sensor[2] == 0 && b_sensor[3] == 0 && b_sensor[4] == 0)){
    position = prev_position;
  }
  prev_position = position;
  int proportional = ((int)position) - 2000;
  int derivative = proportional - last_proportional;
  integral += proportional;
  last_proportional = proportional;
  int power_difference = proportional/10 + integral/10000 + derivative*3/2;
  //if((b_sensor[0] == 1 && b_sensor[1] == 1 && b_sensor[2] == 1 && b_sensor[3] == 1 && b_sensor[4] == 1)){
    //power_difference = max;
  //}

  // determines if the robot turns left or right.
  int m1Speed = max + power_difference;
  int m2Speed = max - power_difference;

  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > max)
    m1Speed = max;
  if (m2Speed > max)
    m2Speed = max;
  set_motorsLF(m2Speed,m1Speed);
}

//**********************************************************************************************************************************************************
void checkSensors()
{
  sensors_average = 0;
  sensors_sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensors[i] = analogRead(i);    // Reading nearer to 900 => light background, Reading nearer to 0 => dark background <400
    // Apply the calibration to the sensor reading
    sensors[i] = map(sensors[i], calmin[i], calmax[i], 0, 1000);

    // In case the sensor value is outside the range seen during calibration
    sensors[i] = constrain(sensors[i], 0, 1000);
    //Serial.println(sensors[i]);
    if (sensors[i] < thresholdValue){  // Dark background
      b_sensor[i] = 1;
    }else{  
      b_sensor[i] = 0;
    }
    sensors_average = sensors_average + b_sensor[i] * i * 1000;   //Calculating the weighted mean
    sensors_sum += int(b_sensor[i]);
  }            //Calculating sum of sensor readings

  Serial.println("Current Sensor Values");
  Serial.print(sensors[0]);
  Serial.print("\t");
  Serial.print(sensors[1]);
  Serial.print("\t");
  Serial.print(sensors[2]);
  Serial.print("\t");
  Serial.print(sensors[3]);
  Serial.print("\t");
  Serial.println(sensors[4]);
  Serial.println(" ");

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(b_sensor[i]);
    Serial.print(" ");
    if(i == (NUM_SENSORS -1)) Serial.println("");
  }
  position = int(sensors_average / sensors_sum);
  Serial.print(" sensors_average: ");
  Serial.print(sensors_average);
  Serial.print(" sensors_sum : ");
  Serial.print(sensors_sum);
  Serial.print(" position: ");
  Serial.print(position);
  Serial.println(' ');
}


//**********************************************************************************************************************************************************
void set_motorsLF(int motor1speed, int motor2speed)     //set_motorsLF(int rightMotorSpeed, int leftmotorSpeed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor speed above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0

  set_motors(motor1speed, motor2speed);
}



//**********************************************************************************************************************************************************
void set_motors(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    analogWrite(MOTOR1_R, abs(rightMotorSpeed)); digitalWrite(MOTOR1_F, LOW);
  }
  else
  {
    analogWrite(MOTOR1_F, rightMotorSpeed);   digitalWrite(MOTOR1_R, LOW);
  }



  if (leftMotorSpeed < 0)
  {
    analogWrite(MOTOR2_R, abs(leftMotorSpeed)); digitalWrite(MOTOR2_F, LOW);
  }
  else
  {
    analogWrite(MOTOR2_F, leftMotorSpeed);   digitalWrite(MOTOR2_R, LOW);
  }
}

//**********************************************************************************************************************************************************  
void line_sensors_scan()
{ 
  for (int i = 0; i < NUM_SENSORS; i++)
  { 
    sensors[i] = analogRead(i);

    if (sensors[i] > calmax[i])
      calmax[i] = sensors[i];

    if (sensors[i] < calmin[i])
      calmin[i] = sensors[i];
  }    
}


//**********************************************************************************************************************************************************
void TriggerSound()
{
  // Set up a counter to pull from melody[] and beats[]
  for (int i = 0; i < MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    duration = beat * tempo;

    playTone();
    delayMicroseconds(pause);
  }
}



//**********************************************************************************************************************************************************
// Pulse the speaker to play a tone for a particular duration
void playTone()
{
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {


      digitalWrite(SPEAKER_OUT, HIGH);
      delayMicroseconds(tone_ / 2);

      // DOWN
      digitalWrite(SPEAKER_OUT, LOW);
      delayMicroseconds(tone_ / 2);

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    }
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delayMicroseconds(duration);
    }
  }
}
