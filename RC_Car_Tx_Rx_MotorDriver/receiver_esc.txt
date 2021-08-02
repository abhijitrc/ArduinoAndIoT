 //Receiver code
 //Abhijit Chaudhari
 //This receiver code is a combination of nrf24l01 receiver and L298N dc motor driver
 //used in my project to operate two dc motors.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//ESC driver
//M1
int enA = 3;
int in1 = 4;
int in2 = 5;
//M2
int enB = 6;
int in3 = 9;
int in4 = 10;

//Receiver data structure
struct Signal {
uint8_t fandr;//forward and reverse, throttle     
uint8_t steer;//steering, left and right, throttle
uint8_t rotaV;//clockwise and anti-clockwise movement for forward and reverse
uint8_t rotaH;//clockwise and anti-clockwise movement for left and right
};

struct Signal trans_data;
const byte pipeIn[6] = "00001";

//Set CE and CSN PIN From Receiver
RF24 radio(7, 8); 

void ResetData()
{

trans_data.fandr = 0;
trans_data.steer = 0; 
trans_data.rotaV = 0;
trans_data.rotaH = 0;
}

void ResetMotor()
{
  // Motor 1
  
  digitalWrite(enA, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor 2
  
  digitalWrite(enB, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void setup()
{
  //motor driver pins
  pinMode(enA, OUTPUT);
  pinMode(enB,  OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  ResetData();
  ResetMotor();
  
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); 
}
unsigned long lastRecvTime = 0;

void recvData()
{
  while ( radio.available() ) {
  radio.read(&trans_data, sizeof(Signal));
  lastRecvTime = millis();
  Serial.println("vert");
  Serial.println(trans_data.fandr);
  Serial.println("rotationV");
  Serial.println(trans_data.rotaV);
  Serial.println("horiz");
  Serial.println(trans_data.steer);
  Serial.println("rotationH");
  Serial.println(trans_data.rotaH);
  Serial.println("");    
  
  }
}

void loop()
{
  unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
ResetData(); 
}
  recvData(); 
  int m1=trans_data.steer;
  int m2=trans_data.fandr;
  int rotV=trans_data.rotaV;
  int rotH=trans_data.rotaH;
  mpower(1,  m1,rotH);
  mpower(2,  m2,rotV);
}


void mpower(int motor,  int spd , int rotation )
{
  int pwm;
  int pA;
  int pB;
  if (motor == 1)
  {
    pwm = enA;
    pA = in1;
    pB = in2;
  } else if (motor == 2) 
  {
    pwm = enB;
    pA = in3;
    pB = in4;
  } else 
  {
    return;
  }

  if (rotation == -1) 
  {
    digitalWrite(pA, LOW);
    digitalWrite(pB, LOW);
  } 
  else if (rotation == 0) 
  {
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
  } 
  else if (rotation == 1) 
  {
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
  }
  analogWrite(pwm, spd);
}
