/*
 * Protect.cpp
 *
 *  Created on: 2017年7月10日
 *      Author: Romeli
 */

#include "Protect.h"

void Protect::SM() {
	if (U_StepMotor1.IsBusy()) {
		U_StepMotor1.SafetyProtect(Limit::Data);
	}
	if (U_StepMotor2.IsBusy()) {
		U_StepMotor2.SafetyProtect(Limit::Data);
	}
}
