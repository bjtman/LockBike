// Brian Tice
// Lock Bike v. 1.5
// 12-18-13

// 12-27-13 -look into using serialevent() to trigger hardware serial data
//          -look into just using the sparkcore or electric imp..

// Notes on this version: State machine implemented, harware ISR routine implemented for RFID reader
// Switched to Arduino Mega for a few reasons. One is that serial lines were running thin
// on Uno. Two is to anticipate further peripherals, ie SparkCore, etc.
    
#include <Adafruit_VC0706.h>
#include <SD.h>
#include <SPI.h>

// comment out this line if using Arduino V23 or earlier
#include <SoftwareSerial.h>      

#define chipSelect 4
#define STATE_IDLE 0
#define STATE_READRFID 1
#define STATE_MOTIONSENSED 2

// Using SoftwareSerial (Arduino 1.0+) or NewSoftSerial (Arduino 0023 & prior):
#if ARDUINO >= 100
// On Uno: camera TX connected to pin 2, camera RX to pin 3:
// SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
// On Mega: camera TX connected to pin 69 (A15), camera RX to pin 3:
  SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
#else
  NewSoftSerial cameraconnection = NewSoftSerial(2, 3);
#endif

Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);
volatile int state = 0;

void setup() {
  
  
// When using hardware SPI, the SS pin MUST be set to an
// output (even if not connected or used).  If left as a
// floating input w/SPI on, this can cause lockuppage.

#if !defined(SOFTWARE_SPI)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if(chipSelect != 53) pinMode(53, OUTPUT); // SS on Mega
#else
  if(chipSelect != 10) pinMode(10, OUTPUT); // SS on Uno, etc.
#endif
#endif
 
  attachInterrupt(4,RfidReadDetected,CHANGE); // setup an interrupt triggered by 
                                              // a change on pin 19 for Arduino Mega
                                              // when pin changes value.
  
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("VC0706 Camera test");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }  
  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  //cam.setImageSize(VC0706_640x480);        // biggest
  cam.setImageSize(VC0706_320x240);          // medium
  //cam.setImageSize(VC0706_160x120);        // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");


  //  Motion detection system can alert you when the camera 'sees' motion!
  cam.setMotionDetect(true);           // turn it on
  //cam.setMotionDetect(false);        // turn it off   (default)

  // You can also verify whether motion detection is active!
  Serial.print("Motion detection is ");
  if (cam.getMotionDetect()) 
    Serial.println("ON");
  else 
    Serial.println("OFF");
  
}



void loop() 
{
  
  
  checkCamera();
  
  switch(state)
  {
    
     case STATE_IDLE:
     { 
       // This is the state when no one is swiping and there is no motion
       Serial.println("Being a Good Idle Machine");
       break;
     }
     
     case STATE_READRFID:
     {  
        byte i = 0;
        byte val = 0;
        byte code[6];
        byte checksum = 0;
        byte bytesread = 0;
        byte tempbyte = 0;
        
        if(Serial1.available() > 0) 
        {
          if((val = Serial1.read()) == 2) 
          {                  // check for header 
             bytesread = 0; 
             while (bytesread < 12) 
             {                        // read 10 digit code + 2 digit checksum
               if( Serial1.available() > 0) 
               { 
                  val = Serial1.read();
                  if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) 
                  { // if header or stop bytes before the 10 digit reading 
                      break;                                    // stop reading
                  }

                  // Do Ascii/Hex conversion:
                  if ((val >= '0') && (val <= '9')) 
                  {
                    val = val - '0';
                  } 
                  else if ((val >= 'A') && (val <= 'F')) 
                  {
                    val = 10 + val - 'A';
                  }

                  // Every two hex-digits, add byte to code:
                  if (bytesread & 1 == 1) 
                  {
                    // make some space for this hex-digit by
                    // shifting the previous hex-digit with 4 bits to the left:
                    code[bytesread >> 1] = (val | (tempbyte << 4));

                    if (bytesread >> 1 != 5) 
                    {                // If we're at the checksum byte,
                       checksum ^= code[bytesread >> 1];       // Calculate the checksum... (XOR)
                    };
                 } 
                 else 
                 {
                    tempbyte = val;                           // Store the first hex digit first...
                 };

                 bytesread++;                                // ready to read next digit
             } 
          } 

          // Output to Serial:

          if (bytesread == 12) 
          {                          // if 12 digit read is complete
            Serial.print("5-byte code: ");
            for (i=0; i<5; i++) 
            {
              if (code[i] < 16) Serial.print("0");
              Serial.print(code[i], HEX);
              Serial.print(" ");
            }
            Serial.println();

            Serial.print("Checksum: ");
            Serial.print(code[5], HEX);
            Serial.println(code[5] == checksum ? " -- passed." : " -- error.");
            Serial.println();
          }

          bytesread = 0;
          state = STATE_IDLE;
          break;
     }
    
   
   }  
   
  }
  }
}


// Hardware ISR routine that check for a pin change on the Serial1 line, where the RFID chip is connected
// Keep Code Minimal here
void RfidReadDetected()
{
  state = STATE_READRFID;
  
}


void checkCamera()
{
  if (cam.motionDetected()) {
   Serial.println("Motion!");   
   cam.setMotionDetect(false);
   
  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  File imgFile = SD.open(filename, FILE_WRITE);
  
  uint16_t jpglen = cam.frameLength();
  Serial.print(jpglen, DEC);
  Serial.println(" byte image");
 
  Serial.print("Writing image to "); Serial.print(filename);
  
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);

    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");

    jpglen -= bytesToRead;
  }
  imgFile.close();
  Serial.println("...Done!");
  cam.resumeVideo();
  cam.setMotionDetect(true);
 }
  
 return;
}
