/*
  Magabot.h - Library for controlling Magabot robotics platform.
  Created by Francisco Dias, December 23, 2011.
  Released into the public domain.
  http://magabot.cc/
*/

#ifndef Magabot_h
#define Magabot_h


#include <Arduino.h>
#include <Wire.h>

#define REGISTER_CONFIG (16)
#define REGISTER_OUTPUT (16)


class Magabot
{
	public:
		Magabot();
		void actuateMotors(int vel1, int vel2);
		void actuateLEDs(int Red, int Green, int Blue);
		void getSonarReadings(float* sonarReadings);
		bool readBumpers();
		void readClicks(float * wheelsRotation);
		

	private:
				int _sonarId;
                bool _ping;
                unsigned long _sonarTimer;
                int _clicksPerTurn;

};


#endif