#ifndef LineTriggers_h
#define LineTriggers_h

#include <Arduino.h>

// Front MUX: F0 (outer left) .. F7 (outer right). BACK = rear center (irVals[8]).
enum class IrChannel : uint8_t {
	F0 = 0,
	F1,
	F2,
	F3,
	F4,
	F5,
	F6,
	F7,
	BACK = 8
};

// Named thresholds on the calibrated 0–1023 scale (after robot.calibrateSensors()).
enum class LineLevel : uint16_t {
	Low = 256,
	Mid = 512,
	High = 768
};

enum class TriggerLogic : uint8_t { All, Any };

struct IrChannelRule {
	uint8_t channel;
	uint16_t min;
	uint16_t max;
};

constexpr IrChannelRule irRule(IrChannel ch, uint16_t min, uint16_t max) {
	return {static_cast<uint8_t>(ch), min, max};
}

constexpr uint16_t irMask(IrChannel ch) {
	return 1u << static_cast<uint8_t>(ch);
}

constexpr uint16_t irMask(IrChannel a, IrChannel b) {
	return irMask(a) | irMask(b);
}

constexpr uint16_t irMask(IrChannel a, IrChannel b, IrChannel c) {
	return irMask(a, b) | irMask(c);
}

constexpr uint16_t irMask(IrChannel a, IrChannel b, IrChannel c, IrChannel d) {
	return irMask(a, b, c) | irMask(d);
}

constexpr uint16_t irMask(
	IrChannel a, IrChannel b, IrChannel c, IrChannel d, IrChannel e
) {
	return irMask(a, b, c, d) | irMask(e);
}

constexpr uint16_t irMask(
	IrChannel a, IrChannel b, IrChannel c, IrChannel d, IrChannel e, IrChannel f
) {
	return irMask(a, b, c, d, e) | irMask(f);
}

constexpr uint16_t irMask(
	IrChannel a,
	IrChannel b,
	IrChannel c,
	IrChannel d,
	IrChannel e,
	IrChannel f,
	IrChannel g
) {
	return irMask(a, b, c, d, e, f) | irMask(g);
}

// T-junction: front sensors 1–6 (excludes outers 0/7).
constexpr uint16_t IR_MASK_T_JUNCTION = 0x007E;
// T-junction with rear center sensor.
constexpr uint16_t IR_MASK_T_JUNCTION_BACK = 0x017E;
constexpr uint16_t IR_MASK_OUTER_LEFT = 0x0001;   // sensor 0
constexpr uint16_t IR_MASK_INNER_LEFT = 0x000E;   // sensors 1–3
constexpr uint16_t IR_MASK_INNER_RIGHT = 0x0070;  // sensors 4–6
constexpr uint16_t IR_MASK_OUTER_RIGHT = 0x0080;  // sensor 7
constexpr uint16_t IR_MASK_OUTERS = 0x0081;       // sensors 0 and 7

#endif
