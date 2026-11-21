/*
 * FFBWheel.h
 *
 *  Created on: 29.03.2022
 *      Author: Yannick
 */

#ifndef USEREXTENSIONS_SRC_FFBWHEEL_H_
#define USEREXTENSIONS_SRC_FFBWHEEL_H_
#include "constants.h"
#ifdef FFBWHEEL

#include "FFBHIDMain.h"

#ifdef HIDAXISRES_USE_32B_DESC
#define FFBWHEEL_32B_MODE true
#else
#define FFBWHEEL_32B_MODE false
#endif

class FFBWheel : public FFBHIDMain {
public:
	FFBWheel();
	virtual ~FFBWheel();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	void usbInit() override;

protected:
	FFBWheel(uint8_t axisCount); // Wheel on the first axis plus further axes without game effects

private:
	std::shared_ptr<EffectsCalculator> effects_calc = std::make_shared<EffectsCalculator>();
	std::shared_ptr<EffectsControlItf> ffb = std::make_shared<HidFFB>(effects_calc,1);
};

#ifdef FFBWHEEL_MULTI
/**
 * FFB wheel on the first axis plus FFBWHEEL_MULTI-1 command axes.
 * Games only drive the first axis (same USB descriptor as the wheel).
 * The other axes use their independent effects and torque sent with axis.<n>.hosttorque.
 * Positions of all axes are reported as HID axes (X, Y, Z, Rx).
 */
class FFBWheelMulti : public FFBWheel {
public:
	FFBWheelMulti();
	static ClassIdentifier info;
	const ClassIdentifier getInfo();
};
#endif

#endif

#endif /* USEREXTENSIONS_SRC_FFBWHEEL_H_ */
