volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
bool isCalib = false;
int mapped = 0;
void setup() {
  Serial.begin (9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  pinMode(19, INPUT_PULLUP); // internalเป็น pullup input pin 3
  
  attachInterrupt(digitalPinToInterrupt(2), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ai1, RISING);

  

  
  
  }
   
  void loop() {
  // Send the value of counter
  mapped = map(counter, -1000,1000,-360,360);
  if( counter != temp ){
  Serial.println (mapped);
  temp = counter;
  }
  }
 
 
  
   
  void ai0() {
  
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {

  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }

 
    
    
  

  
