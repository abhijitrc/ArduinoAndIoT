 //Transmitter code
 //Abhijit Chaudhari
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte pipeOut[6] = "00001";

RF24 radio(7, 8);

//signal data structure. 
struct Signal {
uint8_t fandr;//forward and reverse, throttle
uint8_t steer;//steering, left and right, throttle
uint8_t rotaV;//clockwise and anti-clockwise movement for forward and reverse
uint8_t rotaH;//clockwise and anti-clockwise movement for left and right
};
struct Signal trans_data;
 
void ResetData() 
{
trans_data.fandr = 0;// actual center position for forward and reverse joystick is 517
trans_data.steer = 0; // actual center position for stering joystick is 528
trans_data.rotaV=0;
trans_data.rotaH=0;
}

void setup()
{
//Initialize everything.
Serial.begin(115200);
radio.begin();
radio.openWritingPipe(pipeOut);
radio.setAutoAck(false);
radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.stopListening(); 
ResetData();
}

void processSignal()
{
  // Forward and Reverse
  int posVert=analogRead(A0);
  // Steering
  int posHorz= analogRead(A1);

  // Forward and Reverse
  if (posVert < 460)
  {
    // Set motor in reverse.
    trans_data.rotaV = 1;
    //Determine motor speed.
    //Since we are reversing we need to reverse readings.
    trans_data.fandr = map(posVert, 460, 0, 0, 255);
  }
  else if (posVert > 564)
  {
    // Set motor forward.
    trans_data.rotaV = 0;
    //Determine motor speed.
    trans_data.fandr = map(posVert, 564, 1023, 0, 255);
  }
  else
  {
    // Motors are stopped
    trans_data.rotaV = 0;
    trans_data.fandr= 0;
  }
  
  // Steering section
  if (posHorz < 460)
  {
    // Left
    // Since we are going left we need to reverse readings
    trans_data.steer = map(posHorz, 460, 0, 0, 255);
    trans_data.rotaH = 0;
  }
  else if (posHorz > 564)
  {
    // Move Right
    trans_data.steer = map(posHorz, 564, 1023, 0, 255);
    trans_data.rotaH = 1;

  }
  else
  {
    // Motors are stopped
      trans_data.rotaH = 0;
      trans_data.steer= 0;
  }
}

void loop()
{
  processSignal();
  Serial.println("vert");
  Serial.println(trans_data.fandr);
  Serial.println("rotationV");
  Serial.println(trans_data.rotaV);
  Serial.println("horiz");
  Serial.println(trans_data.steer);
  Serial.println("rotationH");
  Serial.println(trans_data.rotaH);
  Serial.println("");
//Referencing is one of the features specifically for use with pointers. 
//The ampersand operator & is used for this purpose. 
//If x is a variable, then &x represents the address of the variable x.      
  radio.write(&trans_data, sizeof(Signal));
  delay(12); //Calibrate the delay based on your transmitter and receiver.
}
