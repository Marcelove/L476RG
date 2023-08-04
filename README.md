# L476RG
Embbed Systems final project - nucleo L476RG

Marcelo Amorim & Vinicius Rodrigues.

As our proposal, we will create a functional remote gate system using a nodeMCU board and a core-476G.
The gate system will have a Java-based interface, designed as an Android application, with the purpose of authenticating users.

___________________________________________________________________________________________________________________________________
Functionalities of the Remote Gate System:

- Siren/Buzzer: Activated when an unauthorized user attempts to open the gate.
- Presence Sensor: Utilizes a PushButton to simulate the presence sensor. If pressed, it indicates someone is at the gate.
- Luminosity Sensor: Used to observe the ambient light level, and if it's dark, the floodlight will be activated.
- Door Opening: Uses a separate LED to simulate the door opening. If the LED is on, the door is open.
- Floodlight: Utilizes the built-in LED of the core board to simulate the floodlight. If the area is dark, the LED will be lit.
- Login and Password Input Interface: Implements an Android application for entering login and password. Firebase Realtime will handle authentications.

____________________________________________________________________________________________________________________________________
Project Resources:

- STM32cubeIDE: Development interface used for building, executing, and debugging core board commands.
- Firebase: Real-time database used to control authentications.
- AndroidStudio: Development environment for building the Android application acting as an authenticator.
- VsCode + PlatformIO: Development environment for building, executing, and debugging nodeMCU board commands.

____________________________________________________________________________________________________________________________________
General Operation:

- The core board constantly checks for any presence in the area. If someone is detected, it sends a request for authentication.
Resident authentication is done through the Android application: successful login updates the Firebase authentication state,
and if a request to open the door is made, the door will open for 10 seconds. In case of authentication failure,
a 2-second siren will be triggered, indicating an authentication error.

- The nodeMCU board enables real-time verification of Firebase through Wi-Fi connection. A light sensor operates in parallel to the entire system;
if it's dark, the floodlight will be activated.

____________________________________________________________________________________________________________________________________
Authentication Process:
After creating our project in Firebase, we built our database using the platform. Using an object called "Pessoa1," 
we created both "Name" and "Password" records. The Android application connects to Firebase and retrieves the "Name" and "Password" 
fields entered by the user in the application interface. These fields will be read as strings only after the login button is clicked.
We then compare the information in the database with the entered data. If there is a match, the "Open" section will be set to "1" (i.e., granted)
for 10 seconds; if there is no match, an error message will be displayed to the user.
