#include <Arduino.h>
#include <SoftwareSerial.h>
// #include <TinyGsmClient.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Preferences.h"
#include "driver/timer.h"

// تعریف تایمر
#define TIMER_DIVIDER         80      // تقسیم کلاک برای رسیدن به 1 MHz
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // مقیاس تایمر به 1 MHz (1µs per tick)
#define TIMER_INTERVAL_SEC    (600)   // فاصله تایمر (600 ثانیه = 10 دقیقه)

hw_timer_t *timer = NULL;
volatile bool timerEnabled = false; // حالت تایمر


#define DOOR_IS_OPEN 1
#define DOOR_IS_CLOSE 0

#define PERSON_DETECTED 1
#define PERSON_NOT_DETECTED 0

#define STATUS_OFF 0
#define STATUS_ON 1

#define SMS_TARGET  "09364884505"
#define CALL_TARGET "09364884505"

#define NUMBER_OF_DOOR 4
#define NUMBER_OF_PERSON 

#define DOOR_1 0
#define DOOR_2 1
#define DOOR_3 2
#define DOOR_4 3
#define PERSON_1 4
#define PERSON_2 5
#define PERSON_3 6
#define PERSON_4 7

#define M08_POWER_PIN 26
#define BUGLAR_ALARM_PIN 2

#define ALARM_PWM_RES 8
#define ALARM_PWM_CHA 0
#define ALARM_PWM_FRQ 2000


Preferences NVS;

typedef struct struct_message {
  bool value;
  uint8_t battery;
} struct_message;

struct_message receivedData;

uint8_t sensorMacAddr[10][6] = {{0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} , 
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF} ,
                                {0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF}};

typedef struct sensor {
    bool status;
    uint8_t battery;
}sensor;

sensor sensorData[10];

bool update[10] = {0};
bool AlarmStatus = STATUS_OFF;
bool turnOffAlarm = false;
bool smsStatus = STATUS_OFF;
bool callStatus = STATUS_OFF;

uint64_t measureTime = 0;
uint64_t readMassageTime = 0;

String receivedMassage = "";
String activeNumber = "";
String problem = "";

const char*  phoneNumberStoredAdd = "softwareAdd";

SoftwareSerial SerialAT(33, 25);  // RX, TX

String sendATCommand(String command, unsigned long timeout) {
    String response = "";
    SerialAT.flush();
    SerialAT.println(command); // ارسال دستور AT

    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        while (SerialAT.available()) {
            char c = SerialAT.read();
            response += c;
        }
        if (response.indexOf("OK") != -1) { // بررسی پاسخ موفق
            return response;
        }
    }

    // اگر تا پایان زمان مشخص پاسخی دریافت نشد
    return "FAIL";
}

String sendSMS(String phoneNumber, String message) {
    // Set SMS to text mode
    String response = sendATCommand("AT+CMGF=1", 5000);
    // Serial.println("Response: " + response);
    if (response.indexOf("FAIL") != -1) {
        return "Failed to set text mode";
    }

    response = sendATCommand("AT+CSCS=\"GSM\"", 5000);
    // Serial.println("Response: " + response);
    if (response.indexOf("FAIL") != -1) {
        return "Failed to set GSM";
    }

    // Set the recipient phone number
    
    SerialAT.println("AT+CMGS=\"" + phoneNumber + "\""); // ارسال دستور AT
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        while (SerialAT.available()) {
            char c = SerialAT.read();
            response += c;
        }
        // if (response.indexOf(">") != -1) { // بررسی پاسخ موفق
        //     Serial.println("Response: " + response);
        //     return "Failed to set phone number";
        // }
    }
    // Serial.println("Response: " + response);


    // Send the SMS content followed by the Ctrl+Z character
    SerialAT.print(message);
    SerialAT.write(26); // Ctrl+Z character

    // Wait for the response from the modem
    response = sendATCommand("", 10000); // Wait for up to 10 seconds for the SMS to send
    // Serial.println("Response: " + response);
    if (response.indexOf("OK") != -1) {
        return "SMS sent successfully";
    } else {
        return "Failed to send SMS";
    }
}

String readAndDeleteLastSMS() {
    // Set SMS to text mode
    String response = sendATCommand("AT+CMGF=1", 5000);
    if (response.indexOf("FAIL") != -1) {
        return "Failed to set text mode";
    }

    // List all SMS messages to find the last one
    response = sendATCommand("AT+CMGL=\"REC UNREAD\"", 5000);
    if (response.indexOf("OK") == -1) {
        return "Failed to list SMS messages";
    }

    // Extract the last SMS index
    int lastMsgIndex = response.lastIndexOf("+CMGL:");
    if (lastMsgIndex == -1) {
        return "No unread SMS messages found";
    }

    // Find the start of the index
    int indexStart = lastMsgIndex + 7; // "+CMGL: " is 7 characters
    int indexEnd = response.indexOf(',', indexStart);
    if (indexEnd == -1) {
        return "Failed to parse SMS index";
    }

    String indexStr = response.substring(indexStart, indexEnd);
    int msgIndex = indexStr.toInt();

    // Read the last SMS message
    response = sendATCommand("AT+CMGR=" + String(msgIndex), 5000);
    if (response.indexOf("OK") == -1) {
        return "Failed to read SMS";
    }

    // Delete the SMS message
    String deleteResponse = sendATCommand("AT+CMGD=" + String(msgIndex), 5000);
    if (deleteResponse.indexOf("OK") == -1) {
        return "Failed to delete SMS";
    }

    // Extract and return the SMS content
    int msgContentStart = response.indexOf("\r\n", response.indexOf("+CMGR:")) + 2;
    int msgContentEnd = response.indexOf("\r\n", msgContentStart);
    String msgContent = response.substring(msgContentStart, msgContentEnd);

    return "SMS: " + msgContent;
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Bytes received: ");
    Serial.println(len);

    for (size_t i = 0; i < 10; i++) {
        if ((mac[0] == sensorMacAddr[i][0]) && (mac[1] == sensorMacAddr[i][1]) && (mac[2] == sensorMacAddr[i][2]) && (mac[3] == sensorMacAddr[i][3]) && (mac[4] == sensorMacAddr[i][4]) && (mac[5] == sensorMacAddr[i][5])) {
            sensorData[i].status = receivedData.value;
            sensorData[i].battery = receivedData.battery;
            update[i] = true;

            Serial.print("sensorData " + String(i) + "value-> " + String(receivedData.value));
            Serial.print("sensorData " + String(i) + "battery->"  + String(receivedData.battery));
        }
    }
}

void espnowInit(void) {
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void checkDoorStatus(void) {
    String massage = "";
    for (size_t i = 0; i < NUMBER_OF_DOOR; i++) {
        if ((update[i])) {
            // check status of door to turn on/off buglar alarm
            if ((sensorData[DOOR_1].status || sensorData[DOOR_2].status || sensorData[DOOR_3].status || sensorData[DOOR_4].status) == DOOR_IS_OPEN) {
                if (AlarmStatus) {
                    timerStart(timer);
                    ledcWrite(ALARM_PWM_CHA, 125);
                }
            }

            // check status of door and send SMS and call to target
            if (sensorData[i].status == DOOR_IS_OPEN) {
                if (smsStatus) {
                    massage += "Door " + String(i + 1) + "is open";
                    sendSMS("09364884505" ,"");
                }
                
                if (callStatus) {
                    /* code */
                }
            } else if (sensorData[i].status == DOOR_IS_CLOSE) {
                if (smsStatus) {
                    massage += "Door " + String(i + 1) + "is close";
                    sendSMS("09364884505" ,"");
                }
                
                if (callStatus) {
                    /* code */
                }
            }

            update[i] = false;
        }
    }
}

void checkPersonStatus(void) {
    String massage = "";
    for (size_t i = NUMBER_OF_DOOR; i < 8 ; i++) {
        if ((update[i])) {
            // check status of door to turn on/off buglar alarm
            if ((sensorData[PERSON_1].status || sensorData[PERSON_2].status || sensorData[PERSON_3].status || sensorData[PERSON_4].status) == PERSON_DETECTED) {
                if (AlarmStatus) {
                    timerStart(timer);
                    ledcWrite(ALARM_PWM_CHA, 125);
                }
            }

            // check status of door and send SMS and call to target
            if (sensorData[i].status == PERSON_DETECTED) {
                if (smsStatus) {
                    massage += "Person Detected: " + String(i + 1);
                    sendSMS("09364884505" ,"");
                }
                
                if (callStatus) {
                    /* code */
                }
            }

            update[i] = false;
        }
    }
}

void IRAM_ATTR onTimer() {
    ledcWrite(ALARM_PWM_CHA, 0);
}


void setup() {
    // Set console baud rate
    Serial.begin(115200);
    Serial.flush();
    
    // Set M08 baud rate
    SerialAT.begin(57600);
    SerialAT.flush();

    delay(3000);

    //ُSetup Timer for 10m
    timer = timerBegin(0, TIMER_DIVIDER, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, TIMER_INTERVAL_SEC * TIMER_SCALE, true);
    timerStop(timer); 

    //Restore phone Number
    NVS.begin("SM1001", false);
    activeNumber = NVS.getString(phoneNumberStoredAdd);

    // Setup ESP-NOW
    espnowInit();

    // Setup Buglar Alarm
    pinMode(BUGLAR_ALARM_PIN, OUTPUT);
    ledcSetup(ALARM_PWM_CHA, ALARM_PWM_FRQ, ALARM_PWM_RES);
    ledcAttachPin(BUGLAR_ALARM_PIN, ALARM_PWM_CHA);
    ledcWrite(ALARM_PWM_CHA, 0);

    // Check M08 Power on/off
    pinMode(M08_POWER_PIN, OUTPUT);

    String response = sendATCommand("AT", 10000);
    Serial.println("Response: " + response);

    if (response.indexOf("FAIL") != -1) {
        // m08 is off (turn the module on)
        digitalWrite(M08_POWER_PIN ,HIGH);
        delay(1500);
        digitalWrite(M08_POWER_PIN ,LOW);
        Serial.println("The Module turnd on");
        delay(10000);
    }


    measureTime = millis();
    readMassageTime = millis();
}

void loop() {
    if ((millis() - readMassageTime) > 10000) {
        readMassageTime = millis();

        receivedMassage = readAndDeleteLastSMS();
        Serial.println("Received Massage: " + receivedMassage);

        if (receivedMassage.indexOf("Alarm off") != -1) {
            AlarmStatus = STATUS_OFF;

            Serial.println("SMART HOME : Alarm off");
            sendSMS("09364884505" ,"Alarm off");
            // sendSMS(activeNumber ,"SMART HOME : Alarm off");
            receivedMassage = "";
        }
        
        if (receivedMassage.indexOf("Alarm on") != -1) {
            AlarmStatus = STATUS_ON;
            Serial.println("SMART HOME : Alarm on");
            sendSMS("09364884505" ,"SMART HOME : Alarm on");
            // sendSMS(activeNumber ,"SMART HOME : Alarm on");
            receivedMassage = "";
        }

        if (receivedMassage.indexOf("Sms off") != -1) {
            smsStatus = STATUS_OFF;

            Serial.println("SMART HOME : Sms off");
            sendSMS("09364884505" ,"SMART HOME : Sms off");
            // sendSMS(activeNumber ,"SMART HOME : Sms off");
            receivedMassage = "";
        }

        if (receivedMassage.indexOf("Sms on") != -1) {
            smsStatus = STATUS_ON;

            Serial.println("SMART HOME : Sms on");
            sendSMS("09364884505" ,"SMART HOME : Sms on");
            // sendSMS(activeNumber ,"SMART HOME : Sms on");
            receivedMassage = "";
        }

        if (receivedMassage.indexOf("Call off") != -1) {
            callStatus = STATUS_OFF;

            Serial.println("SMART HOME : Call off");
            sendSMS("09364884505" ,"SMART HOME : Call off");
            // sendSMS(activeNumber ,"SMART HOME : Call off");
            receivedMassage = "";
        }
    
        if (receivedMassage.indexOf("Call on") != -1) {
            callStatus = STATUS_ON;

            Serial.println("SMART HOME : Call on");
            sendSMS("09364884505" ,"SMART HOME : Call on");
            // sendSMS(activeNumber ,"SMART HOME : Call on");
            receivedMassage = "";
        }

        if (receivedMassage.indexOf("All off") != -1) {
            callStatus = STATUS_OFF;
            smsStatus = STATUS_OFF;
            AlarmStatus = STATUS_OFF;

            Serial.println("SMART HOME : All off");
            sendSMS("09364884505" ,"SMART HOME : All off");
            // sendSMS(activeNumber ,"SMART HOME : All off");
            receivedMassage = "";
        }
    
        if (receivedMassage.indexOf("All on") != -1) {
            callStatus = STATUS_ON;
            smsStatus = STATUS_ON;
            AlarmStatus = STATUS_ON;

            Serial.println("SMART HOME : All on");
            sendSMS("09364884505" ,"SMART HOME : All on");
            // sendSMS(activeNumber ,"SMART HOME : All on");
            receivedMassage = "";
        }
    
        if (receivedMassage.indexOf("Mute") != -1) {
            timerStop(timer);
            ledcWrite(ALARM_PWM_CHA, 0);

            Serial.println("SMART HOME : Mute");
            sendSMS("09364884505" ,"SMART HOME : Mute");
            // sendSMS(activeNumber ,"SMART HOME : Mute");
            receivedMassage = "";
        }
    
        if (receivedMassage.indexOf("Add phone number") != -1) {
            activeNumber = receivedMassage.substring(receivedMassage.lastIndexOf("number") + 7 ,(receivedMassage.length()));
            NVS.putString(phoneNumberStoredAdd ,activeNumber);

            Serial.println("SMART HOME : Add phone number");
            problem = sendSMS("09364884505" ,"SMART HOME : New number is " + activeNumber);
            Serial.println("sendSMS problem: " + problem);
            // sendSMS(activeNumber ,"SMART HOME : Mute");
            receivedMassage = "";
        }
    
    }

    if ((millis() - measureTime) > 1000) {
        measureTime = millis();

        checkDoorStatus();
        checkPersonStatus();
    }
    
    
    
    // while (Serial.available()) {
    //     SerialAT.write(Serial.read());
    // }
    // while (SerialAT.available()) {
    //     Serial.write(SerialAT.read());
    // }
}