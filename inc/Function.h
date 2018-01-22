/*
 * Function.h
 *
 *  Created on: 2017年7月5日
 *      Author: Romeli
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <Tool/U_PID.h>
#include "U_Misc.h"
#include "cmsis_device.h"
#include "Protocol.h"
#include "Setting.h"


class Function {
public:
	static TwoWordtoByteSigned_Typedef ASBSWAAE_Pos[512];
	static WordtoByteSigned_Typedef ASBSWAAE_ADC[512];
	static WordtoByteSigned_Typedef ADCDATA[128];
	static WordtoByteSigned_Typedef ADCDATA2[128];

	static PIDParam_Typedef PIDParam;
	static U_PID PID;
	static bool PIDEnable;
	static bool AutoControl_SpecialADCWithTime_Busy;
	static bool AutoControl_SpecialADCWithTrigger_Busy;
	static bool AutoControl_SpecialDoubleADCWithTrigger_Busy;
	static bool AutoControl_SpecialMotorPosition_Busy;

	static void Enter(P_Buf_Typedef* p_buf);

	static void Inquire(P_Buf_Typedef* p_buf);
	static void Control(P_Buf_Typedef* p_buf);
	static void AutoControl(P_Buf_Typedef* p_buf);
	static void Setting(P_Buf_Typedef* p_buf);
	static void Special(P_Buf_Typedef* p_buf);

	static void Inquire_Limit();
	static void Inquire_ExLimit();
	static void Inquire_Valve();
	static void Inquire_Encoder(uint8_t no);
	static void Inquire_ADC(uint8_t no);
	static void Inquire_ExADC(uint8_t no);
	static void Inquire_Tigger();
	static void Inquire_Motor();
	static void Inquire_Special(uint16_t num);
	static void Inquire_DAC(uint8_t no);
	static void Inquire_SpecialADCWithTime(uint16_t num);
	static void Inquire_SpecialADCWithTrigger(uint8_t num);
	static void Inquire_SpecialDoubleADCWithTrigger(uint8_t num);
	static void Inquire_SpecialStatus(PC_Typedef pc);
	static void Inquire_Status(uint8_t no);

	static void Control_Valve(uint32_t status);
	static void Control_Motor(int16_t speed);
	static void Control_SM(uint8_t no, uint8_t status);
	static void Control_DAC(uint8_t no, uint16_t data);
	static void Control_ValveOpen(uint32_t status);
	static void Control_ValveClose(uint32_t status);

	static void AutoControl_SM_By_Step(uint8_t no, int32_t step);
	static void AutoControl_SM_By_Limit(uint8_t no, uint8_t status,
			uint8_t limitNo);
	static bool AutoControl_SM_By_Limit_Judge(uint8_t limitNo);
	static void AutoControl_SM_By_Step_With_ADC_And_Encoder(uint8_t no,
			int32_t step, uint8_t encoderNo, uint8_t adcNo, uint16_t num);
	static void AutoControl_SpecialADCWithTime(uint8_t ms, uint8_t adcNo,
			uint16_t num);
	static void AutoControl_SpecialADCWithTrigger(uint8_t sensorNo,
			uint8_t moment, uint8_t adcNo, uint8_t num);
	static void AutoControl_SpecialDoubleADCWithTrigger(uint8_t sensorNo,
			uint8_t moment, uint8_t adcNo, uint8_t adcNo2, uint8_t num);
	static void AutoControl_SpecialMotorPosition(uint8_t sensorNo, uint16_t ms);

	static void Setting_SM_Speed(uint8_t no, uint16_t speed, uint32_t tgtAcc);
	static void Setting_Valve_Default(uint32_t status);
	static void Setting_Encoder_Zero(uint8_t no);
	static void Setting_Protect_Limit(uint8_t no, uint8_t status,
			uint8_t limitNo);
	static void Setting_PIDParam(uint8_t no, DoubletoByte_Typedef p,
			DoubletoByte_Typedef i, DoubletoByte_Typedef d,
			DoubletoByte_Typedef set);
	static void Setting_PIDInput(uint8_t no, DoubletoByte_Typedef now);
	static void Setting_PIDEnable(uint8_t no, uint8_t state);
	static void Setting_SM_RelDir(uint8_t no, uint8_t status);
	static void Setting_USART(uint8_t com);
	static void Setting_Address(uint8_t add);

	static void Special_Reset();
	static void Special_Stop();
	static void Special_Continue();
	static void Special_Cacel();
private:
};

#endif
