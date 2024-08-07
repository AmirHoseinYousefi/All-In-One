#include <Arduino.h>
#include <SoftwareSerial.h>
// #include <TinyGsmClient.h>

#define SMS_TARGET  "09364884505"
#define CALL_TARGET "09364884505"

#define M08_POWER_PIN 26
#define BUGLAR_ALARM_PIN 2

#define ALARM_PWM_RES 8
#define ALARM_PWM_CHA 0
#define ALARM_PWM_FRQ 2

SoftwareSerial SerialAT(33, 25);  // RX, TX

String sendATCommand(String command, unsigned long timeout) {
    String response = "";
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
  if (response.indexOf("FAIL") != -1) {
      return "Failed to set text mode";
  }

  // Set the recipient phone number
  response = sendATCommand("AT+CMGS=\"" + phoneNumber + "\"", 5000);
  if (response.indexOf(">") == -1) {
      return "Failed to set phone number";
  }

  // Send the SMS content followed by the Ctrl+Z character
  SerialAT.print(message);
  SerialAT.write(26); // Ctrl+Z character

  // Wait for the response from the modem
  response = sendATCommand("", 10000); // Wait for up to 10 seconds for the SMS to send
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


void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  // Set M08 baud rate
  SerialAT.begin(57600);

  // Setup Buglar Alarm
  pinMode(BUGLAR_ALARM_PIN, OUTPUT);
  ledcSetup(ALARM_PWM_CHA, ALARM_PWM_FRQ, ALARM_PWM_RES);
  ledcAttachPin(BUGLAR_ALARM_PIN, ALARM_PWM_CHA);
  ledcWrite(ALARM_PWM_CHA, 0);

  // Check M08 Power on/off
  pinMode(M08_POWER_PIN, OUTPUT);

  String response = sendATCommand("AT", 5000);
  if (response.indexOf("FAIL") != -1) {
    // m08 is off (turn the module on)
    digitalWrite(M08_POWER_PIN ,HIGH);
    delay(1500);
    digitalWrite(M08_POWER_PIN ,LOW);
    Serial.println("The Module turnd on");
  }
}

void loop() {
  
}