#ifndef SensorArray_h
#define SensorArray_h

#include <Arduino.h>
#include "MotorController.h"


class SensorArray {
    public:
        void begin();
        void calibrate(MotorController& motorL, MotorController& motorR);

        void readIrRaw(int* irVals);
		void readIrCalibrated(int* irVals);
        bool isOut();	// returns the status of robot if it is outside the track
 		int getPos();
        int getContrast();

    private:
		int irVal_[9];

		void selectMUXChannel_(int channel);


		int SCAN_TIME = 800;
		int highest_[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
		int lowest_[9] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
		int contrast_;
		void saveIrCalibration_(int* lowest, int *highest, int contrast);
		void restoreIrCalibration(int* lowest, int *highest, int* contrast);

		bool outside = false;
};

#endif
