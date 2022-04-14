// Source code for a puzzle box using the Arduino MEGA 2560
// written by Jan Baert
// last update: 2/11/2021

// load libraries
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// stepper motor
const int A = 40;  // phase 1
const int B = 42;  // phase 2
const int C = 44 ; // phase 3
const int D = 46;  // phase 4
const int NUMBER_OF_STEPS_PER_REV = 512;
const int stepper_relay = 3;
  
// SD reader (MISO=50, MOSI=51, SCK=52)
#define SD_CS 48 // chip select SD reader  
File sdfile; 

//RTC (SCL and SDA on respective pins on MEGA 2560)
RTC_DS3231 rtc;
char buf1[20];
char buflcd[20];

// RFID reader (no RX pin)
#define RFID_TXPIN 10
SoftwareSerial ssrfid(RFID_TXPIN , 255);


// RGB LEDs
const int red_pins[3] =  {36,30,24};
const int green_pins[3] =  {38,32,26};
const int blue_pins[3] =  {34,28,22};


// IR reflectance sensors
const int IR_pins[] = {5,6,7};     // define pins distance sensors for experiemnt
const int nsensors = 3;            // numnber of distance sensors

// beam break sensor
const int LDR = A0;
const int BB_led = 2;

// LCD screen (SDA = 20, SDL=21)
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

////////////////////////////////////////////////// setup system ///////////////////////////////////////////////////

void setup() {
      
      // Init Serial port
      Serial.begin(9600);   
      while(!Serial);
      
                                          
      // lcd
      Wire.begin();
      lcd.init(); 


      // init SPI bus
      SPI.begin();

      // setup SD card reader
      lcd.setCursor(0,0);
      lcd.print("Initializing");
      lcd.setCursor(0,1);
      lcd.print("SD card");
      if (!SD.begin(SD_CS)) {
        lcd.setCursor(0,0);
        lcd.print("SD card failed,");
        lcd.setCursor(0,1);
        lcd.print("reset system");
        
 
        while (1);
      }
      lcd.setCursor(0,0);
      lcd.print("Initialization");
      lcd.setCursor(0,1);
      lcd.print("SD succesful");
      SD.mkdir("temp");  

      //Setup for the RTC
      lcd.setCursor(0,0);
      lcd.print("Initializing RTC"); 

      
      if(!rtc.begin()) {
        lcd.setCursor(0,0);
        lcd.print("RTC not found");
        Serial.println("RTC not found");
        lcd.setCursor(0,1);
        lcd.print("reset system");
        Serial.flush();
        abort();
      }
      if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      }
      
      lcd.setCursor(0,0);
      lcd.print("Initialization");
      lcd.setCursor(0,1);
      lcd.print("RTC succesful");       

      // rfid
      ssrfid.begin(9600); 
          
      // stepper motor
      pinMode(A,OUTPUT);
      pinMode(B,OUTPUT);
      pinMode(C,OUTPUT);
      pinMode(D,OUTPUT);
      pinMode(stepper_relay,OUTPUT);

      // LEDs
      for (uint8_t i = 0; i< nsensors; i++) {
          pinMode(red_pins[1], OUTPUT);
          pinMode(blue_pins[1], OUTPUT);
          pinMode(green_pins[1], OUTPUT);
      }

      // IR sensors
      for (uint8_t i = 0; i< nsensors; i++) {
        pinMode(IR_pins[i],INPUT_PULLUP);
      }

      // beam break sensor
      pinMode(LDR, INPUT);
      pinMode(BB_led, OUTPUT); 

      // feedback to user
      lcd.setCursor(0,0);
      lcd.print("setup succesful");
      lcd.setCursor(0,1);
      lcd.print("system running");
      delay(3000);
      lcd.clear();


}

/////////////////////////////////////////////////// main code ///////////////////////////////////////////////////

void loop() {

   
     // define variable for experimental setup
     String tagID;                                 // string to store RFID 
     int nchoices = 3;                             // number of choices
     String possible_colours[3] = {"R","G","B"};   // colour cues 
     int  choice = -1;                             // integer to store individual's choice
     int  maxtrialdur = 10000;                     // maximum time (ms) a trial can last
     int  trialdur = 10000;                        // time the individual needs to solve the puzzle
     int  trialcount;                              // number of trials within current paradigm
     String trialcue;                              // resource cue for current paradigm
     int  trialsolution;                           // position of correct solution
     int maxtrialnumber = 10;                      // maximum number of trails within a paradigm 
     int error = 0;                                // boolean to indicate major issues
     unsigned long starttime = millis();
     unsigned long endtime = starttime;
     File sdfile;
     ssrfid.listen();

     if (ssrfid.available() > 0){

            // turn on circuits
            digitalWrite(BB_led,HIGH);
            digitalWrite(stepper_relay,HIGH);

            // get timestamp
            DateTime now = rtc.now();
            sprintf(buf1, "%02d:%02d:%02d %02d/%02d/%02d",  now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
            sprintf(buflcd, "%02d:%02d %02d/%02d/%02d",  now.hour(), now.minute(), now.day(), now.month(), now.year());
            Serial.println(buflcd);
            
            // read tag
            tagID = ssrfid.readString().substring(3,11);   //only read characters 3 to 11, as 1-2 are version and last 2 are check sum
            lcd.setCursor(0,0);
            lcd.print("tag: "+tagID);
            lcd.setCursor(0,1);
            lcd.print(buflcd);
            Serial.println(tagID);
            
            // read settings of the last puzzle or create a new one if new individual
            sdfile = SD.open("temp/"+tagID+".txt");
            if (sdfile) {
                // read setting from last game
                sdfile.seek(0);
                trialcue = String(sdfile.readStringUntil(','));
                sdfile.seek(2);
                trialcount = sdfile.parseInt();
                SD.remove("temp/"+tagID+".txt");

                // change paradigm if maximum trial number is reached
                if(trialcount == maxtrialnumber){
                    trialcount = 0;
                    if(trialcue=="R"){
                        String pos_col[2]={"G","B"};
                        trialcue = pos_col[random(2)];                   
                    }
                    if(trialcue=="G"){
                        String pos_col[2]={"R","B"};
                        trialcue = pos_col[random(2)];  
                    }
                    if(trialcue=="B"){
                      String pos_col[2]={"R","G"};
                      trialcue = pos_col[random(2)];                       
                    }
                }else{
                      trialcount++;
                }
                
            }else{  
                 // random generate a first trial
                 trialcount = 0;
                 trialcue = possible_colours[random(3)];                 
            }

            // setup puzzle
            int pos_posit[3] = {0,1,2};
            for (int i=0; i<3; i++)
            {
              int r = random(i,3); 
              int temp = pos_posit[i];
              pos_posit[i] = pos_posit[r];
              pos_posit[r] = temp;
            }
          
            RGB_color(255,0,0,red_pins[pos_posit[0]],green_pins[pos_posit[0]],blue_pins[pos_posit[0]]);
            RGB_color(0,255,0,red_pins[pos_posit[1]],green_pins[pos_posit[1]],blue_pins[pos_posit[1]]);
            RGB_color(0,0,255,red_pins[pos_posit[2]],green_pins[pos_posit[2]],blue_pins[pos_posit[2]]);     
           
            
            // record response
            while ((endtime - starttime) <= maxtrialdur and choice == -1) 
            {
              for (int i = 0; i< nchoices; i++) {
                int S=digitalRead(IR_pins[i]);// read the sensor       
                if(S == 0){
                  choice = i;
                  trialdur = endtime - starttime;
                  break;
                }
              }
              
              endtime = millis();
            }
          

            // turn of lights
            RGB_color(0,0,0,red_pins[0],green_pins[0],blue_pins[0]);
            RGB_color(0,0,0,red_pins[1],green_pins[1],blue_pins[1]);
            RGB_color(0,0,0,red_pins[2],green_pins[2],blue_pins[2]);

        
            // give reward if correct
            if(trialcue=="R"){
              trialsolution = pos_posit[0];
            }
            if(trialcue=="G"){
              trialsolution = pos_posit[1];
            }
            if(trialcue=="B"){
              trialsolution = pos_posit[2];
            }

            if(choice == trialsolution){
            
                  int rellevel = analogRead(LDR);
                  int minlevel = rellevel;
                  
                  // spin stepper until something drops or after one and a half spin of the wheel
                  int nrot = 0;
                  while(minlevel > 0.99*rellevel && nrot <15){
                    nrot ++;
                    int i=0;
                    while(i< 52){
                        onestep();
                        i++;
                        int LDRvalue = analogRead(LDR);
                        if(LDRvalue < minlevel){
                          minlevel = LDRvalue;
                        }
                    }
                  
                  }
            }

            // write data from test to SD card 
            sdfile = SD.open(tagID+".txt", FILE_WRITE);

            if (sdfile) {
              sdfile.print(tagID);
              sdfile.print(",");
              sdfile.print(buf1);
              sdfile.print(",");
              sdfile.print(trialcue);
              sdfile.print(",");
              sdfile.print(trialcount);
              sdfile.print(",");
              sdfile.print(possible_colours[choice]);
              sdfile.print(",");
              sdfile.print(trialdur);
              sdfile.println(",");
              sdfile.close();  
            } else {
              Serial.println("error opening "+tagID+".txt");
              lcd.setCursor(0,0);
              lcd.print("error opening");
              lcd.setCursor(0,1);
              lcd.print("log file");
              error = 1;
            }

            // write last setup to temporary file
            sdfile = SD.open("temp/"+tagID+".txt", FILE_WRITE);
            if(sdfile){
              sdfile.print(trialcue);
              sdfile.print(",");
              sdfile.print(trialcount);
              sdfile.close();  
             } else {
              Serial.println("error opening temp/"+tagID+".txt");
              lcd.setCursor(0,0);
              lcd.print("error opening");
              lcd.setCursor(0,1);
              lcd.print("temporary file");
              
              error = 1;
            }
            

            // shut down system and wait for 1second
            digitalWrite(BB_led,LOW);
            digitalWrite(stepper_relay,LOW);
            if(error==0){
              lcd.clear();
            }
            delay(1000);

     }
    }

/////////////////////////////////////////////////// functions ///////////////////////////////////////////////////

// stepper motor
void write(int a,int b,int c,int d){
    digitalWrite(A,a);
    digitalWrite(B,b);
    digitalWrite(C,c);
    digitalWrite(D,d);
}

void onestep(){
    write(1,0,0,0);
    delay(1);
    write(1,1,0,0);
    delay(1);
    write(0,1,0,0);
    delay(1);
    write(0,1,1,0);
    delay(1);
    write(0,0,1,0);
    delay(1);
    write(0,0,1,1);
    delay(1);
    write(0,0,0,1);
    delay(1);
    write(1,0,0,1);
    delay(1);
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value,int red_light_pin, int green_light_pin, int blue_light_pin)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
