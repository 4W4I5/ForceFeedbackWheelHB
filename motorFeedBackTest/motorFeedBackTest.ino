//Software serial is slowing the arduino down
//Need ways to improve processing speed and only send motor commands when needed to.
#define CLK 3
#define DT 2
#include <JoystickFFB.h>
#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(0,1);        // RX, TX

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      30         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;



Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_MULTI_AXIS, 4, 0,
                   false, false, false, false, false, false,
                   false, false, true, true, true);

// Set to true to test "Auto Send" mode or false to test "Manual Send" mode.
// const bool testAutoSendMode = true;
const bool testAutoSendMode = false;
unsigned long gNextTime = 0;
unsigned int gCurrentStep = 0;
int steeringRangeMax = 300;
volatile int steeringCounter = steeringRangeMax / 2;
int homeEncoder = steeringRangeMax / 2;
int encoderUpperLim = steeringRangeMax;
int encoderLowerLim = -encoderUpperLim;
int currentStateCLK;
int lastStateCLK;
int acceleratePin = A0;
int brakePin = A1;
//--------Smoothing LED
const int numReadings = 25;
int readingsLED1[numReadings]; // the readings from the analog input
int readingsLED2[numReadings]; // the readings from the analog input
int readIndex1 = 0;            // the index of the current reading
int readIndex2 = 0;            // the index of the current reading
int total1 = 0;                // the running total
int total2 = 0;                // the running total
float average1 = 0;            // the average
float average2 = 0;            // the average
float smoothBrightness = 0;
int accelerateLED = 10;
int brakeLED = 9;
int brightness1 = 0; // how bright the LED is
int brightness2 = 0; // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by
//-------------------FEEDBACK MOTOR-------------
unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;
int var = 0;

//-------------------FEEDBACK MOTOR-------------

//----------------------------------------------
// ########################## SEND ##########################
void Send(int16_t uSpeed, int16_t uSteer =0)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte    = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("CMD1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" TorqueRequested: ");  Serial.print(Feedback.cmd2);
           
            Serial.print(" Speed: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" Volt: ");  Serial.print(Feedback.batVoltage / 100);
            Serial.print(" Temp: ");  Serial.print(Feedback.boardTemp);
            Serial.println();
            } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}
void testSteering(int value)
{
    // int newVal = value * 1.9;
    int newVal = value;
    Joystick.setSteering(newVal);
}
void accelerateUpdate(int value)
{
    Joystick.setAccelerator(value);
}
void brakeUpdate(int value)
{
    Joystick.setBrake(value);
}
void updateLed()
{
    total1 = total1 - readingsLED1[readIndex1];
    total2 = total2 - readingsLED2[readIndex2];
    readingsLED1[readIndex1] = analogRead(acceleratePin);
    readingsLED2[readIndex2] = analogRead(brakePin);
    total1 = total1 + readingsLED1[readIndex1];
    total2 = total2 + readingsLED2[readIndex2];
    readIndex1 = readIndex1 + 1;
    readIndex2 = readIndex2 + 1;

    if (readIndex1 >= numReadings)
    {
        readIndex1 = 0;
    }
    else if (readIndex2 >= numReadings)
    {
        readIndex2 = 0;
    }
    average1 = total1 / numReadings;
    average2 = total2 / numReadings;

    int brightness1 = map(average1, 120, 400, 0, 255);
    int brightness2 = map(average2, 120, 450, 0, 255);
    if (brightness1 >= 255)
    {
        brightness1 = 255;
    }
    if (brightness1 <= 0)
    {
        brightness1 = 0;
    }
    if (brightness2 >= 255)
    {
        brightness2 = 255;
    }
    if (brightness2 <= 0)
    {
        brightness2 = 0;
    }
    analogWrite(accelerateLED, brightness1);
    analogWrite(brakeLED, brightness2);
    delay(1); // delay in between reads for stability
}
// void updateMotor()  //We might not need PID since DirectInput already has the values
// {
//     Input = analogRead(PIN_INPUT);

//     double gap = abs(Setpoint - Input); // distance away from setpoint
//     if (gap < 10)
//     { // we're close to setpoint, use conservative tuning parameters
//         myPID.SetTunings(consKp, consKi, consKd);
//     }
//     else
//     {
//         // we're far from setpoint, use aggressive tuning parameters
//         myPID.SetTunings(aggKp, aggKi, aggKd);
//     }

//     myPID.Compute();
// }
void setup()
{
    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);

    // Call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);

    Joystick.setSteeringRange(0, steeringRangeMax);
    Joystick.setAcceleratorRange(120, 400);
    Joystick.setBrakeRange(100, 450);
    (testAutoSendMode) ? Joystick.begin() : Joystick.begin(false); // if joystick is bugging remove this and uncoment the bottom one
    // if (testAutoSendMode)
    // {
    //     Joystick.begin();
    // }
    // else
    // {
    //     Joystick.begin(false);
    // }
    pinMode(CLK, INPUT_PULLUP);
    pinMode(DT, INPUT_PULLUP);
    pinMode(acceleratePin, INPUT_PULLUP);
    pinMode(brakePin, INPUT_PULLUP);
    pinMode(accelerateLED, OUTPUT);
    pinMode(brakeLED, OUTPUT);



    for (int thisReading = 0; thisReading < numReadings; thisReading++)
    {
        readingsLED1[thisReading] = 0;
        readingsLED2[thisReading] = 0;
    }

    
    

    
    Serial.begin(SERIAL_BAUD);
    HoverSerial.begin(HOVER_SERIAL_BAUD);
}

void motorUpdate(int accelerationVal =0){
    unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(accelerationVal);
}
int accel = 0;
int brake = 0;
void loop()
{
    accel = analogRead(acceleratePin);
    brake = analogRead(brakePin);
    if(accel > 425){accel = 425;}
    if(accel < 100){accel = 100;}
    Serial.println(accel);
    accelerateUpdate(accel);
    brakeUpdate(brake);
    testSteering(steeringCounter);
    updateLed();
    motorUpdate(map(accel, 100, 425, 0, 1000));
    Joystick.sendState(); // sends final values over USB
    // updateMotor();
}

void updateEncoder() // Interuptt service routine
{
    // Read the current state of CLK
    currentStateCLK = digitalRead(CLK);
    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK && currentStateCLK == 1)
    {

        // If the DT state is different than the CLK state then
        // the encoder is rotating CCW so decrement
        if (digitalRead(DT) != currentStateCLK)
        {
            steeringCounter--;
            // if(steeringCounter < encoderLowerLim){ //remove these when done since it destroys the calibration
            //     steeringCounter = encoderLowerLim;
            // }
        }
        else
        {
            // Encoder is rotating CW so increment
            steeringCounter++;
            // if(steeringCounter > encoderUpperLim){
            //     steeringCounter = encoderUpperLim;
            // }
        }
    }

    // Remember last CLK state
    lastStateCLK = currentStateCLK;
}
