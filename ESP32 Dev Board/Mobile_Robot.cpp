#include <WiFi.h>
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 3000
#define HOST "192.168.235.144"

#define SSID "WLAN"
#define PASS ""



//------------------------------------------------------------- Motor A Pins / Left Motor
const int ENA = 5;
const int IN1 = 18;
const int IN2 = 19;

//------------------------------------------------------------- Motor B Pins / Right Motor
const int ENB = 25;
const int IN3 = 33;
const int IN4 = 32;



struct PHI {
    float phi_R;
    float phi_L;
};


struct PWM {
    short pwm_R;
    short pwm_L;
};

int client = -1;
PWM* pwm = nullptr;



void stream(void* pvParameters) {

    while (true) {

        recv(client, (void*)pwm, sizeof(PWM), MSG_WAITALL);


        if (pwm->pwm_L > 0) {

            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, +pwm->pwm_L);
        }
        else {

            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, -pwm->pwm_L);
        }


        if (pwm->pwm_R > 0) {

            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENB, +pwm->pwm_R);
        }
        else {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, -pwm->pwm_R);
        }

    }

}




void setup() {

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);


    delay(2000);

    Serial.begin(9600);

    pwm = new PWM;
    pwm->pwm_R = 0;
    pwm->pwm_L = 0;

    // ------------------------------------------ Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(SSID, PASS);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.println(".");
        delay(1000);
    }

    Serial.println("\nLAN connection: OK");

    Serial.println(WiFi.localIP());

    // ------------------------------------------ socket()
    while (client == -1) {
        client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        delay(1000);
    }

    Serial.println("Socket creation: OK");

    // ------------------------------------------ connect()
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(HOST);

    int stat = -1;
    while (stat != 0) {
        stat = connect(client, (struct sockaddr*)&server_addr, sizeof(server_addr));
        delay(1000);
    }

    Serial.println("Connection to the control panel: OK");

    delay(1000);

    xTaskCreate(stream, "stream", 10000, NULL, 1, NULL);


}


void loop() {

}