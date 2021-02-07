//
// Created by fraser on 05/02/2021.
//
#include<axp20x.h>
#ifndef TINYGPS_TBEAM_H
#define TINYGPS_TBEAM_H

#endif //TINYGPS_TBEAM_H
#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define GPS_BAND_RATE               9600
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK             GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK                  5
#define RADIO_MISO                  19
#define RADIO_MOSI                  27
#define RADIO_CS                    18
#define RADIO_DI0                   26
#define RADIO_RST                   23
#define RADIO_DIO1                  33
#define RADIO_BUSY                  32
#define LoRa_frequency 868.0

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

#define GPS_BAUD_RATE               9600
#define HAS_GPS

//#define HAS_SPI // HAS_I2C THIS IS DEFINED IN MAIN

#define SCREEN_WIDTH                128 // OLED display width, in pixels
#define SCREEN_HEIGHT               64 // OLED display height, in pixels
// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI                   14
#define OLED_CLK                    13
#define OLED_DC                     33
#define OLED_CS                     32
#define OLED_RESET                  25
#define OLED_PWR                    2

#ifdef HAS_SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);////
#else
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 4);
#endif

AXP20X_Class PMU;
bool  axpIrq = 0;
void turnOffRTC(){
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
}

void setFlag(void)
{
    axpIrq = true;
}
void sortIQR(){
    if (axpIrq) {
        axpIrq = 0;
        PMU.readIRQ();
        Serial.println("axp20x irq enter!");
        if (PMU.isAcinOverVoltageIRQ()) {
            Serial.printf("isAcinOverVoltageIRQ\n");
        }
        if (PMU.isAcinPlugInIRQ()) {
            Serial.printf("isAcinPlugInIRQ\n");
        }
        if (PMU.isAcinRemoveIRQ()) {
            Serial.printf("isAcinRemoveIRQ\n");
        }
        if (PMU.isVbusOverVoltageIRQ()) {
            Serial.printf("isVbusOverVoltageIRQ\n");
        }
        if (PMU.isVbusPlugInIRQ()) {
            Serial.printf("isVbusPlugInIRQ\n");
        }
        if (PMU.isVbusRemoveIRQ()) {
            Serial.printf("isVbusRemoveIRQ\n");
        }
        if (PMU.isVbusLowVHOLDIRQ()) {
            Serial.printf("isVbusLowVHOLDIRQ\n");
        }
        if (PMU.isBattPlugInIRQ()) {
            Serial.printf("isBattPlugInIRQ\n");
        }
        if (PMU.isBattRemoveIRQ()) {
            Serial.printf("isBattRemoveIRQ\n");
        }
        if (PMU.isBattEnterActivateIRQ()) {
            Serial.printf("isBattEnterActivateIRQ\n");
        }
        if (PMU.isBattExitActivateIRQ()) {
            Serial.printf("isBattExitActivateIRQ\n");
        }
        if (PMU.isChargingIRQ()) {
            Serial.printf("isChargingIRQ\n");
        }
        if (PMU.isChargingDoneIRQ()) {
            Serial.printf("isChargingDoneIRQ\n");
        }
        if (PMU.isBattTempLowIRQ()) {
            Serial.printf("isBattTempLowIRQ\n");
        }
        if (PMU.isBattTempHighIRQ()) {
            Serial.printf("isBattTempHighIRQ\n");
        }
        if (PMU.isPEKShortPressIRQ()) {
            Serial.printf("isPEKShortPressIRQ\n");
        }
        if (PMU.isPEKLongtPressIRQ()) {
            Serial.printf("isPEKLongtPressIRQ\n");
        }

        if (PMU.isTimerTimeoutIRQ()) {
            Serial.printf("isTimerTimeoutIRQ\n");
            PMU.offTimer();
            PMU.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
        }
        PMU.clearIRQ();
    }
}
bool initPMU()
{
    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("OH NO, Power INIT FAIL!");
        return false;
    }

    PMU.setChargeControlCur(500);
    PMU.setChargingTargetVoltage(AXP202_TARGET_VOL_4_2V);

    PMU.setVWarningLevel1(3450);
    PMU.setVWarningLevel2(3400);
    PMU.setPowerDownVoltage(2700);

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
    attachInterrupt(PMU_IRQ, setFlag, FALLING);

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

    //PMU.setPowerOutPut(AXP192_DCDC3, AXP202_OFF);
    //PMU.setDCDC3Voltage(1800);  //Set to lower volts, save power? doing this causes a crash
    PMU.setPowerOutPut(AXP192_DCDC3, AXP202_ON);  //Power for the esp32 it's on anyways, this isn't needed

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

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
    Wire.begin(I2C_SDA, I2C_SCL);
    initPMU();
    Serial.begin(115200);

    SPI.begin(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, RADIO_CS);
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("Radio Started!");

    Serial.println("Display Connected.");

    Serial.println("GPS INIT");


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
    display.print("Tiny GPS, YO");
    display.display();
    delay(300);
}
void batteryStatus(){
    if(PMU.isBatteryConnect()){
        Serial.print("Battery Voltage: "); Serial.print(PMU.getBattVoltage(), 2); Serial.println(" mV");
        if(PMU.isChargeing()){Serial.print("Battery Charge Current: "); Serial.print(PMU.getBattChargeCurrent(), 2); Serial.println(" mA");}
        else { Serial.print("Battery Discharge: "); Serial.print(PMU.getBattDischargeCurrent(), 2); Serial.print(" mA");}
    }
}