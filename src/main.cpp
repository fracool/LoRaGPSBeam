#include <Arduino.h>
#include <RadioLib.h>
#include<Wire.h>
#include <axp20x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TinyGPS++.h>

#define DELAY 1000

#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define GPS_BAND_RATE      9600
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK             GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK              5
#define RADIO_MISO              19
#define RADIO_MOSI             27
#define RADIO_CS             18
#define RADIO_DI0           26
#define RADIO_RST              23
#define RADIO_DIO1         33
#define RADIO_BUSY           32
#define LoRa_frequency 868.0

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

#define GPS_BAUD_RATE               9600
#define HAS_GPS
#define HAS_SPI // HAS_I2C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI   14
#define OLED_CLK   13
#define OLED_DC    33
#define OLED_CS    32
#define OLED_RESET 25
#define OLED_PWR 2
double lat;
double lon;
uint8_t tme[3];
uint16_t date[3];

#ifdef HAS_SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);////
#else
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 4);
#endif

SX1276 radio = new Module(RADIO_CS, RADIO_DI0, RADIO_RST, RADIO_BUSY);
AXP20X_Class PMU;

bool initPMU()
{
    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("OH NO, Power INIT FAIL!");
        return false;
    }
    /*
     * The charging indicator can be turned on or off
     * * * */
     //PMU.setChgLEDMode(LED_BLINK_4HZ);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC3Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC3, AXP202_ON);
    *
    * * * */

    /*
     *   Turn off unused power sources to save power
     * **/

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);//gps off

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                   AXP202_VBUS_CUR_ADC1 |
                   AXP202_BATT_CUR_ADC1 |
                   AXP202_BATT_VOL_ADC1,
                   AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                  AXP202_VBUS_CONNECT_IRQ |
                  AXP202_BATT_REMOVED_IRQ |
                  AXP202_BATT_CONNECT_IRQ,
                  AXP202_ON);
    PMU.clearIRQ();

    return true;
}
void disablePeripherals()
{
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    radio.sleep();
#ifdef HAS_SPI
    digitalWrite(OLED_PWR, LOW);
#endif
}
void initBoard()
{
#ifdef HAS_SPI
    pinMode(OLED_PWR, OUTPUT);
    digitalWrite(OLED_PWR, HIGH);
#endif
    Serial.println("initBoard");
    initPMU();
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, RADIO_CS);
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("Radio Started!");

    Serial.println("Display Connected.");

    Serial.println("GPS INIT");


    /*
    * T-BeamV1.0, V1.1 LED defaults to low level as trun on,
    * so it needs to be forced to pull up
    * * * * */

    gpio_hold_dis(GPIO_NUM_4);
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);

}
void initDisplay(){
    display.begin(SSD1306_SWITCHCAPVCC,  0x3C);//i2c addr = 0x3c // while commented out, it's using SPI mode
#ifdef HAS_SPI
    display.setRotation(2);
#endif
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.clearDisplay();
    display.print("Tiny GPS, BIIITCH");
    display.display();
    delay(300);
}
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
    disablePeripherals();
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
long timer=millis();
long timer2=millis();
void loop()
{

    // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial1.available() > 0) {

            if (gps.encode(Serial1.read())) {
                getInfo();
                if(millis()>timer+DELAY){updateDisplay();timer=millis();}


            }
            //if (millis()>timer2+10000){sleep();}

    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS detected: check wiring."));
        delay(10000);
        sleep();
    }
}

