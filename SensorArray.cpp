#include "SensorArray.h"
#include <EEPROM.h>
#include "MotorController.h"



void SensorArray::begin() {
	// Initialize MUX A-C
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);

	// Initialize MUX OUTPUT
	pinMode(IR_MUX_OUTPUT, INPUT);

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
	int high, low;
	unsigned int timer = millis();

	while(counter < 4){
		if (counter % 2 == 0){
			motorL.setSpeed(100);
			motorR.setSpeed(100);
		} else {
			motorL.setSpeed(-100);
			motorR.setSpeed(-100);
		}

		readIrRaw(irVal_);
		for(int i = 0; i < 9; i++){
			if(irVal_[i] > high) high = irVal_[i];
			if(irVal_[i] < low) low = irVal_[i];
			if(irVal_[i] > highest_[i]) highest_[i] = irVal_[i];
			if(irVal_[i] < lowest_[i]) lowest_[i] = irVal_[i];
		}

		if (millis() - timer > SCAN_TIME) {
			counter += 1;
			timer = millis();
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
	for (int i = 0; i < 8; i++) {
		selectMUXChannel_(i);
		val = analogRead(IR_MUX_OUTPUT);
		irVals[i] = (val - lowest_[i]) * float(1024.00/(highest_[i] - lowest_[i]));
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
    return 0;
}