#define TINY_GSM_MODEM_SIM800

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// or Software Serial on Uno, Nano
//#include <SoftwareSerial.h>
//SoftwareSerial SerialAT(2, 3); // RX, TX

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

/*
 * Tests enabled
 */
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_USSD false
#define TINY_GSM_TEST_BATTERY false
#define TINY_GSM_TEST_GPS false
// powerdown modem after tests
#define TINY_GSM_POWERDOWN true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "YourAPN";
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include <bigiot.h>
#include "esp_adc_cal.h"
#include <Wire.h>
#include <axp20x.h>

TinyGsm modem(SerialAT);

#define PIN_TX                  27
#define PIN_RX                  26
#define UART_BAUD               115200
#define PWR_PIN                 4
#define RST_PIN                 5
#define MODEM_POWER_ON          23
#define DEFAULT_VREF            1100
#define BUZZER_PIN_NUM          18
#define I2C_SDA                 21
#define I2C_SCL                 22
#define USER_GPIO               13


#include "privateKey.h"     //Comment out this line and fill in the following data

#ifndef VBAT_STREAM_ID
#define BIGIOT_DEV_ID               ""
#define BIGIOT_API_KEY              ""
#define VBAT_STREAM_ID              ""
#define BOOTCOUNT_STREAM_ID         ""
#define CHARGE_CURRENT_STREAM_ID    ""
#define ACIN_STREAM_ID              ""
#define ACIN_CURRENT_STREAM_ID      ""
#define SERVER_WECHAT_KEY           ""            //Third-party WeChat interface apikey，See http://sc.ftqq.com/3.version
#endif

const char *id =                    BIGIOT_DEV_ID ;        //platform device id
const char *apikey =                BIGIOT_API_KEY;        //platform device api key
const char *usrkey =                "";                    //platform user key , if you are not using encrypted login,you can leave it blank


#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  180        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR uint32_t bootCount = 0;
uint32_t startstamp = 0;

TinyGsmClient gsmClient;
BIGIOT bigiot(gsmClient);
ServerChan cat(gsmClient, SERVER_WECHAT_KEY);
AXP20X_Class axp;
RTC_DATA_ATTR uint32_t vref = DEFAULT_VREF;

/***********************************
 *  !!! ADC FUNCTION
 * *********************************/
float getVoltage(uint8_t gpio)
{
    uint16_t v = analogRead(gpio);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return (battery_voltage);
}

void setupADC()
{
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_7, (adc_bits_width_t)ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        // Serial.printf("eFuse Vref:%u mV\n", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        // Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        // Serial.println("Default Vref: 1100mV");
    }
}
#include <WiFi.h>


void play(int c)
{
    for (int i = 0; i < c; i++) {
        ledcWrite(0, 1000);
        delay(200);
        ledcWrite(0, 0);
        delay(200);
    }
}

void setup()
{
    startstamp = millis();

    WiFi.mode(WIFI_OFF);

    // SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    // if (!SD.begin(SD_CS)) {
    //     Serial.println("SDCard MOUNT FAIL");
    // } else {
    //     uint32_t cardSize = SD.cardSize() / (1024 * 1024);
    //     String str = "SDCard Size: " + String(cardSize) + "MB";
    //     Serial.println(str);
    // }

    // DBG("Wait...");
    // delay(200);


    esp_sleep_wakeup_cause_t r = esp_sleep_get_wakeup_cause();
    if (r == ESP_SLEEP_WAKEUP_TIMER) {
        ++bootCount;
    } else {
        // setupADC(); //No use
    }

    Wire.begin(I2C_SDA, I2C_SCL);

    int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);

    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }

    //! Turn off unused power
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    //! Do not turn off DC3, it is powered by esp32
    // axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON);

    //Turn off the charge indicator
    axp.setChgLEDMode(AXP20X_LED_OFF);

    //Turn on AXP192 data acquisition ADC
    axp.adc1Enable(AXP202_BATT_CUR_ADC1 |
                   AXP202_BATT_VOL_ADC1 |
                   AXP202_ACIN_VOL_ADC1 |
                   /*AXP202_ACIN_CUR_ADC1 | AXP202_VBUS_VOL_ADC1 |*/
                   AXP202_VBUS_VOL_ADC1 |
                   AXP202_VBUS_CUR_ADC1,
                   AXP202_ON);
    axp.clearIRQ();


    // vbat_voltage = getVoltage(VBAT_PIN_NUM);
    // vbus_voltage = getVoltage(VBUS_PIN_NUM);
    // if (vbat_voltage < 3.0 && vbus_voltage < 4) {
    //     esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * 300);
    //     esp_deep_sleep_start();
    // }

    // Set console baud rate
    SerialMon.begin(115200);
    delay(10);

    // Set-up modem reset, enable, power pins
    pinMode(PWR_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    pinMode(USER_GPIO, OUTPUT);


    digitalWrite(USER_GPIO, LOW);
    digitalWrite(PWR_PIN, LOW);
    digitalWrite(RST_PIN, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);


    // pinMode(BUZZER_PIN_NUM, OUTPUT);
    ledcAttachPin(BUZZER_PIN_NUM, 0);
    ledcSetup(0, 1000, 16);
    play(2);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);


}

void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void enterSleep(uint32_t time_in_us)
{
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
        DBG("GPRS disconnected");
    } else {
        DBG("GPRS disconnect: Failed.");
    }
    // modem.sleepEnable(false);

    modem.poweroff();
    DBG("Poweroff.");

    esp_sleep_enable_timer_wakeup(time_in_us);

    Serial.print("Use -> ");
    Serial.print(millis() - startstamp);
    Serial.println("Ms");

    esp_deep_sleep_start();
}



void loop()
{
    DBG("Initializing modem...");
    gsmClient.init(&modem);
    if (!modem.begin()) {
        DBG("Cannot init modem");
        espDelay(5000);
        modem.restart();
        return;
    }

    DBG("Waiting for network...");
    if (!modem.waitForNetwork()) {
        espDelay(5000);
        return;
    }

    if (modem.isNetworkConnected()) {
        DBG("Network connected");
    }

    DBG("Connecting to", apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        // delay(10000);
        return;
    }

    bool res = modem.isGprsConnected();
    DBG("GPRS status:", res ? "connected" : "not connected");
    play(4);


#if TINY_GSM_TEST_BATTERY
    uint8_t chargeState = -99;
    int8_t percent = -99;
    uint16_t milliVolts = -9999;
    modem.getBattStats(chargeState, percent, milliVolts);
    DBG("Battery charge state:", chargeState);
    DBG("Battery charge 'percent':", percent);
    DBG("Battery voltage:", milliVolts / 1000.0F);

    float temp = modem.getTemperature();
    DBG("Chip temperature:", temp);
#endif

    //Regist platform command event hander
    // bigiot.eventAttach([](const int devid, const int comid, const char *comstr, const char *slave) {
    //     Serial.printf("Received[%d] - [%d]:%s \n", devid, comid, comstr);
    // });

    //Regist device disconnect hander
    // bigiot.disconnectAttack([](BIGIOT & obj) {
    // When the device is connected to the platform, you can preprocess your peripherals here
    // Serial.print(obj.deviceName());
    // Serial.println("  disconnect");
    // });

    //Regist device connect hander
    bigiot.connectAttack([](BIGIOT & obj) {
        // When the device is connected to the platform, you can preprocess your peripherals here
        Serial.print(obj.deviceName());
        Serial.println("  connect");

        // String title = String(obj.deviceName()) + "[电量过低]";
        // if (! cat.sendWechat(title.c_str(), NULL)) {
        //     Serial.println("Send fail");
        // }

    });

    // Login to bigiot.net
    if (!bigiot.login(id, apikey, usrkey)) {
        Serial.println("Login fail");
        enterSleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    }

    static char vbat_buffer[64];
    static char current_buffer[64];
    static char boot_buffer[64];
    static char acin_buffer[64];
    static char acin_current_buffer[64];

    // snprintf(volbuffer, sizeof(volbuffer), "%.2fV/%.2fmA", axp.getBattVoltage() / 1000.0, axp.isChargeing() ? axp.getBattChargeCurrent() : axp.getBattDischargeCurrent());

    snprintf(vbat_buffer, sizeof(vbat_buffer), "%f",  axp.getBattVoltage() / 1000.0);
    snprintf(acin_buffer, sizeof(acin_buffer), "%f",  axp.getVbusVoltage() / 1000.0);
    snprintf(current_buffer, sizeof(current_buffer), "%f", axp.isChargeing() ? axp.getBattChargeCurrent() : 0.0);
    snprintf(acin_current_buffer, sizeof(acin_current_buffer), "%f",  axp.getVbusCurrent());
    snprintf(boot_buffer, sizeof(boot_buffer), "%u", bootCount);

    const char *id[] = {VBAT_STREAM_ID, CHARGE_CURRENT_STREAM_ID, ACIN_STREAM_ID, ACIN_CURRENT_STREAM_ID, BOOTCOUNT_STREAM_ID};
    const char *data[] = {vbat_buffer, current_buffer, acin_buffer, acin_current_buffer, boot_buffer};
    bigiot.upload(id, data, 5);

    uint32_t stamp = millis();
    while (1) {
        if (millis() - stamp > 2000) {
            enterSleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        }
        bigiot.handle();
        delay(5);
    }
}
