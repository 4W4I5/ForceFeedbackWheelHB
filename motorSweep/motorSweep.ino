
#define CLK 3
#define DT 2
//GOAL 
//Make the motor rotate opposite to the encoder and have it try to return to home

unsigned long gNextTime = 0;
unsigned int gCurrentStep = 0;
// volatile int counter = 1200;     //good range
int steeringRangeMax = 300;
volatile int steeringCounter = steeringRangeMax / 2;
int encoderUpperLim = steeringRangeMax;
int encoderLowerLim = -encoderUpperLim;
int currentStateCLK;
int lastStateCLK;
int acceleratePin = A0;
int brakePin = A1;

//-------------------FEEDBACK MOTOR-------------
int motorPWMPin = 6; // yellow
int rightDir = 8;    // purple
int leftDir = 7;     // grey

//-------------------FEEDBACK MOTOR-------------
void hardBrake(int delayTime)
{
    digitalWrite(rightDir, 0);
    digitalWrite(leftDir, 0);
    digitalWrite(motorPWMPin, 0);
    delay(delayTime);
}
void turnLeft(int pwmSignal =0 )
{
    digitalWrite(leftDir, 1);
    digitalWrite(rightDir, 0);
    analogWrite(motorPWMPin, pwmSignal);
}
void turnRight(int pwmSignal =0)
{
    digitalWrite(leftDir, 0);
    digitalWrite(rightDir, 1);
    analogWrite(motorPWMPin, pwmSignal);
}
//----------------------------------------------



void setup()
{
    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);

    // Call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);

    pinMode(CLK, INPUT_PULLUP);
    pinMode(DT, INPUT_PULLUP);
    pinMode(acceleratePin, INPUT_PULLUP);
    pinMode(brakePin, INPUT_PULLUP);


    pinMode(motorPWMPin, OUTPUT);
    pinMode(rightDir, OUTPUT);
    pinMode(leftDir, OUTPUT);

}
void loop()
{
    // if(steeringCounter > 0){
    //   turnRight(255);
    // }
    // if(steeringCounter < 0){
    //   turnLeft(255);
    // }
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
