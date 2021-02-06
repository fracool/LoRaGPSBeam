#include <Arduino.h>
#include <RadioLib.h>
#include<Wire.h>
#include <axp20x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TinyGPS++.h>
#define HAS_SPI
#include <TBeam.h>

#define DELAY 1000  //screen refresh delay
#define TRANSMISSION_DELAY 30000  //LoRa Transmission delay


byte cipher[8] = {0xAA, 0x33, 0x5F, 0x3B, 0x47, 0x00, 0x01, 0xFE};
byte buff[8];

double lat;
double lon;
uint8_t tme[3];
uint16_t date[3];


SX1276 radio = new Module(RADIO_CS, RADIO_DI0, RADIO_RST, RADIO_BUSY);

TinyGPSPlus gps;


void updateDisplay(){
    display.clearDisplay();

    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(tme[2]);
    display.print(':');
    display.print(tme[1]);
    display.print(':');
    display.print(tme[0]);


    display.setTextSize(1);
    display.setCursor(0,18);
    display.print("LAT:");
    display.print(lat, 6);

    display.setCursor(0,34);
    display.print("LNG:");
    display.print(lon, 6);

    display.print(" S:");
    display.print(gps.satellites.value());

    display.setTextSize(2);
    display.setCursor(0,48);
    display.print("TINYYY GPS ");
    display.display();
}
void sleep(){
    Serial.println("Going to sleep...");
    delay(3000);
    esp_sleep_enable_timer_wakeup(60*1000*1000);
    radio.sleep();
    disablePeripherals();
    turnOffRTC();
    delay(1000);
    esp_deep_sleep_start();
}
void getInfo() {
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
        lat = gps.location.lat();
        lon = gps.location.lng();

        Serial.print(lat, 6);
        Serial.print(F(","));
        Serial.print(lon, 6);

    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()) {
        date[0] = gps.date.day();
        date[1] = gps.date.month();
        date[2] = gps.date.year();

        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()) {

        tme[0] = gps.time.second();
        tme[1] = gps.time.minute();
        tme[2] = gps.time.hour();

        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
void doubleToBytes(double input){

    byte dataAr[8] = {
            ((uint8_t*)&input)[0],
            ((uint8_t*)&input)[1],
            ((uint8_t*)&input)[2],
            ((uint8_t*)&input)[3],
            ((uint8_t*)&input)[4],
            ((uint8_t*)&input)[5],
            ((uint8_t*)&input)[6],
            ((uint8_t*)&input)[7]
    };
    Serial.print("Double to Byte: ");
    for(int i=0; i<8; i++){
        Serial.print(dataAr[i]);
        buff[i] = dataAr[i];
        Serial.print(" ");
    }
    Serial.println();


}
void cipherBytes(byte *raw){

    for(uint8_t i =0; i<8; i++){
        buff[i] = raw[i]+cipher[i];
    }
    Serial.print("Ciphered: ");
    for(int i=0; i<8; i++){
        Serial.print(buff[i]);
        Serial.print(" ");
    }
    Serial.println();

}
void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    //pinMode(2, OUTPUT);
    //digitalWrite(2, HIGH);
    initDisplay();

    // initialize SX1276 with default settings
    Serial.print(F("[SX1276] Initializing ... "));

    int state = radio.begin(868.0, 125.0, 9, 7, 0xAA);

    if (state == ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        delay(10000);
        sleep();
    }

    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.readData();
    // radio.scanChannel();
}
void transmitGPS(double lat, double lng){
    byte transmission[16];

    doubleToBytes(lat); // stored in the buffer 'buff'
    cipherBytes(buff);
    for(uint16_t i =0; i<8; i++ ){
        transmission[i]= buff[i]; //write ciphered lat into transmission first
    }

    doubleToBytes(lng);
    cipherBytes(buff);
    for(uint16_t i =8; i<16; i++ ){
        transmission[i]= buff[i-8]; // then write lng
    }


    Serial.print("Attempting Transmission... ");
    for(int i=0; i<16; i++){
        Serial.print(transmission[i]);
        Serial.print(" ");
    }
    Serial.println();

    int status = radio.transmit(transmission, 16);
    if (status == ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(status);
        delay(10000);
        sleep();
    }

}
long timer=millis();
long timer2=millis();
void loop()
{

    // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial1.available() > 0) {

            if (gps.encode(Serial1.read())) {
                getInfo();
                if(millis()>timer+DELAY){updateDisplay();timer=millis();}
                if(millis()>timer2+TRANSMISSION_DELAY){transmitGPS(lat, lon); timer2=millis(); batteryStatus();sleep();}
                if(axpIrq){sortIQR();}
            }



    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS detected: check wiring."));
        delay(10000);
        sleep();
    }
}

