/*
 * ExADC.h
 *
 *  Created on: 2017年7月3日
 *      Author: Romeli
 */

#ifndef EXADC_H_
#define EXADC_H_

#include "U_Misc.h"
#include "cmsis_device.h"

class ExADC {
public:
	static WordtoByte_Typedef Data[8];
	static void Init();
	static void RefreshData();
private:
	static void GPIOInit();
};

#endif /* EXADC_H_ */
