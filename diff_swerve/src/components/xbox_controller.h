#ifndef XBOX_CONTROLLER_
#define XBOX_CONTROLLER_

#include <string.h>

class XboxController {
	private:
		std::string eventName;
		bool ready;
		int buttonPressed;
		void controllerThread(); 
	public:
		volatile float leftY, leftX, rightY, rightX;
		XboxController(std::string eventName);
		bool isConnected();
		bool isReady() { return ready; };
		libevdev* initializeLibevdev();
		float getLeftY() { return leftY; };
		float getLeftX() { return leftX; };
		float getRightY() { return rightY; };
		float getRightX() { return rightX; };
		int getButtonPressed() { return buttonPressed; };
};

#endif
