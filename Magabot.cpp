#include "Magabot.h"
#include <inttypes.h>
#include <Math.h>

Magabot::Magabot() {
    // initialize the digital pin as an output.
    // Pin 13 has an LED connected on most Arduino boards:


    //RGB led pins
    pinMode(11, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);

    //Bumpers
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);



    Wire.begin();

    _clicksPerTurn = 3900;
    sonarId = 0;
    _ping = false;
    _sonarTimer = millis();
}

//**************************//
//******Actuate motors******//
//**************************//

void Magabot::actuateMotors(int vel1, int vel2) {

    vel2 = vel2 * -1;
    byte v1b1 = vel1 >> 8;
    byte v1b2 = vel1 & 0xFF;
    byte v2b1 = vel2 >> 8;
    byte v2b2 = vel2 & 0xFF;

    Wire.beginTransmission(0x15);
    Wire.write((byte) 0);
    Wire.write(v1b1);
    Wire.write(v1b2);
    Wire.write(1); //  high byte
    Wire.endTransmission();

    Wire.beginTransmission(0x16);
    Wire.write((byte) 0);
    Wire.write(v2b1);
    Wire.write(v2b2);
    Wire.write(1); //  high byte

    Wire.endTransmission();



}



//************************//
//******Actuate LEDs******//
//************************//

void Magabot::actuateLEDs(int Red, int Green, int Blue) {
    Red = 255 - Red;
    Green = 255 - Green;
    Blue = 255 - Blue;

    analogWrite(11, (unsigned char) Green);
    analogWrite(10, (unsigned char) Red);
    analogWrite(9, (unsigned char) Blue);
}


//***************************************//
//*****SONARS READ **********************//
//***************************************//

uint8_t Magabot::getSonarReadings(float* sonarReadings) {
	uint8_t returnedID = -1;

    if (_ping) {
        while (millis() < _sonarTimer) {

        }
        // step 3: instruct sensor to return a particular echo _sonarReading
        Wire.beginTransmission(0x71 + _sonarId); // transmit to device #112
        Wire.write(0x02); // sets register pointer to echo #1 register (0x02)
        Wire.endTransmission(); // stop transmitting
        int value;
        // step 4: request _sonarReading from sensor
        Wire.requestFrom(0x71 + _sonarId, 2); // request 2 chars from slave device #112

        // step 5: receive _sonarReading from sensor
        if (2 <= Wire.available()) // if two chars were received
        {
            value = Wire.read(); // receive high char (overwrites previous _sonarReading)
            value = value << 8; // shift high char to be high 8 bits
            value |= Wire.read(); // receive low char as lower 8 bits
            sonarReadings [_sonarId] = value / 100.0;
        }
		returnedID = _sonarId;
        _sonarId = (_sonarId + 1) % 5;
    }
    Wire.beginTransmission(0x71 + sonarId);
    Wire.write((byte) 0);
    Wire.write(0x51);
    Wire.endTransmission();
    _sonarTimer = millis() + 60;
    _ping = true;
	return returnedID;
}





//***************************************//
//****Front bumpers read function********//
//***************************************//

bool Magabot::readBumpers() {
    return digitalRead(2) != 1 || digitalRead(3) != 1 || digitalRead(4) != 1 || digitalRead(5) != 1;
}


//***************************//
//****** Odometer ***********//
//***************************//

void Magabot::readClicks(float * wheelsRotation) {
    short leftClicks = 0;
    short rightClicks = 0;
    Wire.beginTransmission(0x15);
    Wire.write(0x19);
    Wire.write(1);
    Wire.endTransmission();

    delay(1);

    Wire.beginTransmission(0x16);
    Wire.write(0x19);
    Wire.write(1);
    Wire.endTransmission();

    delay(1);

    Wire.beginTransmission(0x15); // transmit to device 0x15
    Wire.write(0x15); // sets register pointer to echo #1 register (0x15)
    Wire.endTransmission();

    Wire.requestFrom(0x15, 2);

    if (2 <= Wire.available()) // if two chars were received
    {
        leftClicks = (short) ((Wire.read() << 8) + Wire.read());
    }

    Wire.beginTransmission(0x16); // transmit to device 0x16
    Wire.write(0x15); // sets register pointer to echo #1 register (0x15)
    Wire.endTransmission();

    Wire.requestFrom(0x16, 2);
    if (2 <= Wire.available()) // if two chars were received
    {
        rightClicks = (short) ((Wire.read() << 8) + Wire.read());
    }
    Wire.beginTransmission(0x15);
    Wire.write(0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);

    Wire.beginTransmission(0x16);
    Wire.write(0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);
    wheelsRotation[0] = leftClicks * -2 * PI / _clicksPerTurn;
    wheelsRotation[1] = rightClicks * 2 * PI / _clicksPerTurn;
}