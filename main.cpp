#include <Arduino.h>

#include <SoftwareSerial.h>
/**
 * Created by K. Suwatchai (Mobizt)
 *
 * Email: k_suwatchai@hotmail.com
 *
 * Github: https://github.com/mobizt/Firebase-ESP8266
 *
 * Copyright (c) 2022 mobizt
 *
 */

/** This example will show how to authenticate as a user with Email and password.
 *
 * You need to enable Email/Password provider.
 * In Firebase console, select Authentication, select Sign-in method tab,
 * under the Sign-in providers list, enable Email/Password provider.
 *
 * From this example, the user will be granted to access the specific location that matches
 * the user uid.
 *
 * This example will modify the database rules to set up the security rule which need to
 * guard the unauthorized access with the user Email.
 */

#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "LIVE TIM_19F0_2G"
#define WIFI_PASSWORD "pnarnhedtu"

/** 2. Define the API key
 *
 * The API key (required) can be obtained since you created the project and set up
 * the Authentication in Firebase console. Then you will get the API key from
 * Firebase project Web API key in Project settings, on General tab should show the
 * Web API Key.
 *
 * You may need to enable the Identity provider at https://console.cloud.google.com/customer-identity/providers
 * Select your project, click at ENABLE IDENTITY PLATFORM button.
 * The API key also available by click at the link APPLICATION SETUP DETAILS.
 *
 */
//#define API_KEY "AIzaSyBYRuTLUNr3J4KDndx9IIrLUpPKFBe4o1Q"  //O MEUUUUUUU
#define API_KEY "AIzaSyAVR2RcYkYspjp3zbPKv-NgXULbJHfTsGg"   // O DE PEEPEEEEEEEEE

/* 3. Define the user Email and password that already registerd or added in your project */
/* O MEUUUUUUU
#define USER_EMAIL "jrmarcelo2011@gmail.com" 
#define USER_PASSWORD "marcelo"
*/
#define USER_EMAIL "vrgf@cesar.school" // O DE PEEPEEEEEEEEE
#define USER_PASSWORD "testando"     // O DE PEEPEEEEEEEEE

/* 4. If work with RTDB, define the RTDB URL */
//#define DATABASE_URL "https://fir-test-4477b-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_URL "https://wsrr-sistemasembarcados-default-rtdb.firebaseio.com/" //O DE PEEEPEEEEEEEEEE

/** 5. Define the database secret (optional)
 *
 * This database secret needed only for this example to modify the database rules
 *
 * If you edit the database rules yourself, this is not required.
 */
//#define DATABASE_SECRET "kEkfTIGH7ohrd94bddFy8XTyc02s6gR9izmz2jzn" //O MEUUUUU
#define DATABASE_SECRET "RJYzuIpwWsskzLrPCA6X0dJLA12AJVwU6eSqxmG4"   //O DE PEPEEEEEEEEEEEE

/* 6. Define the Firebase Data object */
FirebaseData fbdo;

/* 7. Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* 8. Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;

#define Pin_ST_NUCLEO_RX 5
#define Pin_ST_NUCLEO_TX 4

SoftwareSerial SSerial(Pin_ST_NUCLEO_RX, Pin_ST_NUCLEO_TX);

void setup()
{

    pinMode(BUILTIN_LED,OUTPUT);
    Serial.begin(115200);

    SSerial.begin(115200);

    //SSerial.println("Serial by software!");
    Serial.println("Serial by hardware!");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    //Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        //Serial.print(".");
        delay(300);
    }
    //Serial.println();
    //Serial.print("Connected with IP: ");
    //Serial.println(WiFi.localIP());
    //Serial.println();

    //Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    /* Assign the RTDB URL */
    config.database_url = DATABASE_URL;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    String base_path = "/UsersData/MyData";

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);

    /** Now modify the database rules (if not yet modified)
     *
     * The user, <user uid> in this case will be granted to read and write
     * at the certain location i.e. "/UsersData/<user uid>".
     *
     * If you database rules has been modified, please comment this code out.
     *
     * The character $ is to make a wildcard variable (can be any name) represents any node key
     * which located at some level in the rule structure and use as reference variable
     * in .read, .write and .validate rules
     *
     * For this case $userId represents any <user uid> node that places under UsersData node i.e.
     * /UsersData/<user uid> which <user uid> is user UID.
     *
     * Please check your the database rules to see the changes after run the below code.
     */
    String var = "$userId";
    String val = "($userId === auth.uid && auth.token.premium_account === true && auth.token.admin === true)";
    Firebase.setReadWriteRules(fbdo, base_path, var, val, val, DATABASE_SECRET);

    /** path for user data is now "/UsersData/<user uid>"
     * The user UID can be taken from auth.token.uid
     */
}

void loop()
{
    // Firebase.ready() should be called repeatedly to handle authentication tasks.

    if (millis() - dataMillis > 1000 && Firebase.ready())
    {
        dataMillis = millis();
        String path = "/UsersData/MyData";
        path += auth.token.uid.c_str(); //<- user uid of current user that sign in with Emal/Password
        path += "/test/int";
        //Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, path, analogRead(A0)) ? "ok" : fbdo.errorReason().c_str());
        
        bool led;
        Firebase.getInt(fbdo,"LED", &led);
        digitalWrite(BUILTIN_LED,led);

        //Ler autenticação
        int aut = 0;
        //Firebase.getInt(fbdo,"AUT", &aut);
        Firebase.getInt(fbdo,"Open", &aut);
        Serial.printf("%d", aut);

        //Enviar para o Núcleo
        /*
        if (Serial.available()){
            SSerial.write((char)aut);
        }
        */
       /*
        if (SSerial.available()){
            Serial.printf("Writing on Serial...\n");
            Serial.write(SSerial.read());
        }
        */
        if (SSerial.available()){
            Serial.printf("Writing on SS...\n");
            if (aut == 1){
                SSerial.write('S');
                Serial.read();
                SSerial.read();
            }
            else{
                SSerial.write('N');
                Serial.read();
                SSerial.read();
            }
        }

    }
}
