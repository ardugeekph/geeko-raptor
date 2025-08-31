#include "SensorArray.h"
#include <EEPROM.h>
#include "MotorController.h"



void SensorArray::begin() {
	// Initialize MUX A-C
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);

	// Initialize MUX OUTPUT
	pinMode(IR_MUX_OUTPUT, INPUT);

	// Initialize LEDs
	DDRD |= (1 << PD6) | (1 << PD5);

    restoreIrCalibration(lowest_, highest_, &contrast_);
}


void SensorArray::calibrate(MotorController& motorL, MotorController& motorR) {
    // reset values
    contrast_ = 0;
    for(int i = 0; i < 9; i++){
        highest_[i] = 0;
        lowest_[i] = 1023;
    }

    int counter = 0;
	int high = 0, low = 1023;
	bool isCentered = false;

	while(counter < 4){
		motorL.encoder.update();
		motorR.encoder.update();
		motorL.setRpmSpeed(300, 1, false);
		motorR.setRpmSpeed(300, 1, true);

		int mid_threshold = ((highest_[4] - lowest_[4]) / 2) + lowest_[4];
		int mid = irVal_[4];
		
		// Serial.print("Threshold: ");
		// Serial.print(mid_threshold);
		// Serial.print("\tMid: ");
		// Serial.print(mid);
		// Serial.print("\tCounter: ");
		// Serial.println(counter);
		
		if (mid > mid_threshold && !isCentered) {
			isCentered = true;
			counter++;
		} else if (mid < mid_threshold && isCentered) {
			isCentered = false;
		}

		readIrRaw(irVal_);
		for(int i = 0; i < 9; i++){
			if(irVal_[i] > high) high = irVal_[i];
			if(irVal_[i] < low) low = irVal_[i];
			if(irVal_[i] > highest_[i]) highest_[i] = irVal_[i];
			if(irVal_[i] < lowest_[i]) lowest_[i] = irVal_[i];
		}
	}

	contrast_ = high - low;

	motorL.stop();
	motorR.stop();
	
	// Save to EEPROM
	saveIrCalibration_(lowest_, highest_, contrast_);
}


void SensorArray::readIrRaw(int* irVals){
	for (int i = 0; i < 8; i++) {
		selectMUXChannel_(i);
		irVals[i] = analogRead(IR_MUX_OUTPUT);
		// Serial.print(IR_val[i]);
		// Serial.print("  ");
	}
	irVals[8] = analogRead(IR_BACK_SENSOR);
	// Serial.print(IR_val[8]);
	// Serial.println("  ");
}


void SensorArray::selectMUXChannel_(int channel) {
	// Clear PC3, PC4, PC5 (MUX select bits)
	PORTC &= ~((1 << PC3) | (1 << PC4) | (1 << PC5));

	// Set them according to bits of channel
	if (channel & 0b001) PORTC |= (1 << PC3); // A3
	if (channel & 0b010) PORTC |= (1 << PC4); // A4
	if (channel & 0b100) PORTC |= (1 << PC5); // A5
}


void SensorArray::saveIrCalibration_(int* lowest, int *highest, int contrast) {
    // Save lowest[] - saved as two separate 8-bit values per sensor
    for (int i = 0; i < 9; i++) {
        byte val = lowest[i] & 0xFF;
        EEPROM.write(((i + 1) * 2) - 1, val);
        val = lowest[i] >> 8;
        EEPROM.write((i + 1) * 2, val);
    }

    // Save highest[] - also saved as two 8-bit values per sensor, offset by 18
    for (int i = 0; i < 9; i++) {
        byte val = highest[i] & 0xFF;
        EEPROM.write(((i + 1) * 2) - 1 + 18, val);
        val = highest[i] >> 8;
        EEPROM.write(((i + 1) * 2) + 18, val);
    }

    // Save contrast
    byte contrast_byte = contrast & 0xFF;
    EEPROM.write(37, contrast_byte);
    contrast_byte = contrast >> 8;
    EEPROM.write(38, contrast_byte);
}


/**
 * @brief Restores the IR sensor calibration values from EEPROM.
 *
 * This function reads previously stored calibration data for a 9-channel IR sensor array.
 * The calibration data includes:
 * - `lowest[9]`: The lowest recorded values for each IR sensor during calibration.
 * - `highest[9]`: The highest recorded values for each IR sensor during calibration.
 * - `contrast`: A global contrast threshold used for sensor evaluation.
 *
 * Data is stored in EEPROM in the following layout:
 * - Bytes 0–17:   9 pairs (low byte, high byte) for `lowest[9]`
 * - Bytes 18–35:  9 pairs for `highest[9]`
 * - Bytes 36–37:  contrast (low byte at 36, high byte at 37)
 *
 * This function reconstructs the 16-bit values from two 8-bit reads (low and high bytes).
 * The restored values are printed via Serial for debugging.
 */
void SensorArray::restoreIrCalibration(int* lowest, int *highest, int* contrast){
	for (int i = 0; i < 9; i++) {
        lowest[i] = EEPROM.read((i + 1) * 2) << 8 | EEPROM.read(((i + 1) * 2) - 1);
    }
    for (int i = 0; i < 9; i++) {
        highest[i] = EEPROM.read((i + 1) * 2 + 18) << 8 | EEPROM.read(((i + 1) * 2) - 1 + 18);
    }
    *contrast = EEPROM.read(38) << 8 | EEPROM.read(37);

	Serial.println("Lowest");
    // Debug print
    for (int i = 0; i < 9; i++) {
        Serial.print(lowest[i]);
        Serial.print("\t");
    }
    Serial.println();

	Serial.println("Highest");
    for (int i = 0; i < 9; i++) {
        Serial.print(highest[i]);
        Serial.print("\t");
    }
    Serial.println();


	Serial.print("Contrast: ");
    Serial.println(*contrast);
}

void SensorArray::readIrCalibrated(int* irVals){
	int val;
	int readOrder[8] = {0, 7, 1, 6, 2, 5, 3, 4};
	for (int i = 0; i < 8; i++) {
		int idx = readOrder[i];
		selectMUXChannel_(idx);
		val = analogRead(IR_MUX_OUTPUT);
		irVals[idx] = (val - lowest_[idx]) * float(1024.00 / (highest_[idx] - lowest_[idx]));
	}
	val = analogRead(IR_BACK_SENSOR);
	irVals[8] = (val - lowest_[8]) * float(1024.00/(highest_[8] - lowest_[8]));
}


bool SensorArray::isOut() {
	return outside;
}

int SensorArray::getContrast() {
	return contrast_;
}


int SensorArray::getPos() {

	readIrCalibrated(irVal_);

  	float total = (irVal_[0]*0) + (irVal_[1]*1000.00) + (irVal_[2]*2000.00) + (irVal_[3]*3000.00) + (irVal_[4]*4000.00) + (irVal_[5]*5000.00) + (irVal_[6]*6000.00) + (irVal_[7]*7000.00);
  	float pos = total/(irVal_[0] + irVal_[1] + irVal_[2] + irVal_[3] + irVal_[4] + irVal_[5] + irVal_[6] + irVal_[7]);

    // for (int i = 0; i < 8; i++) {
    //     Serial.print(irVal_[i]);
    //     Serial.print("\t");
    // }
    // Serial.println();

	//  get lowest and highest reading to know the current contrast
  	int low=7000, high=0;
  	for(int i = 0; i < 8; i++){
    		if(irVal_[i] > high) high = irVal_[i];
    		if(irVal_[i] < low) low = irVal_[i];
  	}

	
	// set outside_ status
  	if(high-low < contrast_/2) outside = true;
  	else outside = false;

	int highest_val = 0;
  	if(!outside){
		for(int i = 0; i < 8; i++){
			if(irVal_[i] > highest_val){
				highest_val = irVal_[i];
				lastChannelSeen = i;
			}
		}
  	}

	if(outside) {
		if(lastChannelSeen == 0) pos = 0;
		else if(lastChannelSeen == 7) pos = 7000;
		else pos = 0;
  	}

    return pos - 3500;
}