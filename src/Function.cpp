/*
 * Function.cpp
 *
 *  Created on: 2017骞�7鏈�5鏃�
 *      Author: Romeli
 */

#include <Tool/U_SystemTick.h>
#include "Function.h"

#include "LED.h"

#include "Module.h"

#include "U_ADC1.h"
#include "ExADC.h"
#include "U_DAC.h"

#include "Limit.h"
#include "ExLimit.h"

#include "PowerDev.h"

#include "TimeTick.h"

TwoWordtoByteSigned_Typedef Function::ASBSWAAE_Pos[512];
WordtoByteSigned_Typedef Function::ASBSWAAE_ADC[512];
WordtoByteSigned_Typedef Function::ADCDATA[128];
WordtoByteSigned_Typedef Function::ADCDATA2[128];

PIDParam_Typedef Function::PIDParam = { 0, 0, 0 };
U_PID Function::PID = U_PID(0, 0, 0, 0.001, PIDDir_Negtive, &PIDParam,
		PIDMode_Diff);
bool Function::PIDEnable = false;
bool Function::AutoControl_SpecialADCWithTime_Busy = false;
bool Function::AutoControl_SpecialADCWithTrigger_Busy = false;
bool Function::AutoControl_SpecialDoubleADCWithTrigger_Busy = false;
bool Function::AutoControl_SpecialMotorPosition_Busy = false;

void Function::Enter(P_Buf_Typedef* p_buf) {
	uint8_t mask = p_buf->pc & PC_Mask;
	switch (mask) {
	case PC_Inquire_Mask:
		Inquire(p_buf);
		break;
	case PC_Control_Mask:
		Control(p_buf);
		break;
	case PC_AutoControl_Mask:
		TimeTick::ThreadStart = true;
		AutoControl(p_buf);
		TimeTick::ThreadStart = false;
		break;
	case PC_Setting_Mask:
		Setting(p_buf);
		break;
	case PC_Special_Mask:
		Special(p_buf);
		break;
	default:
		break;
	}
	p_buf->flag = false;
}

void Function::Inquire(P_Buf_Typedef* p_buf) {
	switch (p_buf->pc) {
	case PC_Inquire_Limit:
		Inquire_Limit();
		break;
	case PC_Inquire_ExLimit:
		Inquire_ExLimit();
		break;
	case PC_Inquire_Valve:
		Inquire_Valve();
		break;
	case PC_Inquire_Encoder:
		Inquire_Encoder(p_buf->data[0]);
		break;
	case PC_Inquire_ADC:
		Inquire_ADC(p_buf->data[0]);
		break;
	case PC_Inquire_ExADC:
		Inquire_ExADC(p_buf->data[0]);
		break;
	case PC_Inquire_Tigger:
		Inquire_Tigger();
		break;
	case PC_Inquire_Motor:
		Inquire_Motor();
		break;
	case PC_Inquire_Special: {
		WordtoByte_Typedef num;
		num.byte[0] = p_buf->data[0];
		num.byte[1] = p_buf->data[1];
		num.word = num.word > 1024 ? 1024 : num.word;
		Inquire_Special(num.word);
	}
		break;
	case PC_Inquire_DAC:
		Inquire_DAC(p_buf->data[0]);
		break;
	case PC_Inquire_SpecialADCWithTime: {
		WordtoByte_Typedef num;
		num.byte[0] = p_buf->data[0];
		num.byte[1] = p_buf->data[1];
		Inquire_SpecialADCWithTime(num.word);
	}
		break;
	case PC_Inquire_SpecialADCTrigger:
		Inquire_SpecialADCWithTrigger(p_buf->data[0]);
		break;
	case PC_Inquire_SpecialStatus:
		Inquire_SpecialStatus((PC_Typedef) p_buf->data[0]);
		break;
	case PC_Inquire_SpecialDoubleADCTrigger:
		Inquire_SpecialDoubleADCWithTrigger(p_buf->data[0]);
		break;
	case PC_Inquire_Status:
		Inquire_Status(p_buf->data[0]);
		break;
	default:
		break;
	}
}

void Function::Control(P_Buf_Typedef* p_buf) {
	switch (p_buf->pc) {
	case PC_Control_Valve: {
		TwoWordtoByte_Typedef tmp;
		tmp.byte[0] = p_buf->data[0];
		tmp.byte[1] = p_buf->data[1];
		tmp.byte[2] = p_buf->data[2];
		Control_Valve(tmp.twoword);
		break;
	}
	case PC_Control_Motor: {
		WordtoByteSigned_Typedef tmp;
		tmp.byte[0] = p_buf->data[0];
		tmp.byte[1] = p_buf->data[1];
		Control_Motor(tmp.word);
		break;
	}
	case PC_Control_SM:
		Control_SM(p_buf->data[0], p_buf->data[1]);
		break;
	case PC_Control_DAC: {
		WordtoByte_Typedef data;
		data.byte[0] = p_buf->data[1];
		data.byte[1] = p_buf->data[2];
		Control_DAC(p_buf->data[0], data.word);
	}
		break;
	case PC_Control_ValveOpen: {
		TwoWordtoByte_Typedef tmp;
		tmp.byte[0] = p_buf->data[0];
		tmp.byte[1] = p_buf->data[1];
		tmp.byte[2] = p_buf->data[2];
		Control_ValveOpen(tmp.twoword);
	}
		break;
	case PC_Control_ValveClose: {
		TwoWordtoByte_Typedef tmp;
		tmp.byte[0] = p_buf->data[0];
		tmp.byte[1] = p_buf->data[1];
		tmp.byte[2] = p_buf->data[2];
		Control_ValveClose(tmp.twoword);
	}
		break;
	default:
		break;
	}
	uint8_t ttt = 0;
	Protocol::Send(PC_Post_Complete, 0, p_buf->pc, &ttt);
}

void Function::AutoControl(P_Buf_Typedef* p_buf) {
	uint8_t ttt = 0;
	Protocol::Send(PC_Post_Complete, 0, p_buf->pc, &ttt);

	LED::Turn(Color_Blue);
	switch (p_buf->pc) {
	case PC_AutoControl_SM_By_Step: {
		TwoWordtoByteSigned_Typedef step;
		step.byte[0] = p_buf->data[1];
		step.byte[1] = p_buf->data[2];
		step.byte[2] = p_buf->data[3];
		step.byte[3] = p_buf->data[4];
		AutoControl_SM_By_Step(p_buf->data[0], step.twoword);
		break;
	}
	case PC_AutoControl_SM_By_Limit: {
		AutoControl_SM_By_Limit(p_buf->data[0], p_buf->data[1], p_buf->data[2]);
		break;
	}
	case PC_AutoControl_SM_By_Step_With_ADC_And_Encoder: {
		TwoWordtoByteSigned_Typedef step;
		WordtoByte_Typedef num;
		TimeTick::ThreadStart = true;
		step.byte[0] = p_buf->data[1];
		step.byte[1] = p_buf->data[2];
		step.byte[2] = p_buf->data[3];
		step.byte[3] = p_buf->data[4];

		num.byte[0] = p_buf->data[7];
		num.byte[1] = p_buf->data[8];
		num.word = num.word > 1024 ? 1024 : num.word;
		AutoControl_SM_By_Step_With_ADC_And_Encoder(p_buf->data[0],
				step.twoword, p_buf->data[5], p_buf->data[6], num.word);
		break;
	}
	case PC_AutoControl_SpecialADCWithTime: {
		WordtoByte_Typedef num;
		num.byte[0] = p_buf->data[2];
		num.byte[1] = p_buf->data[3];
		AutoControl_SpecialADCWithTime(p_buf->data[0], p_buf->data[1],
				num.word);
	}
		break;
	case PC_AutoControl_SpecialADCWithTrigger:
		AutoControl_SpecialADCWithTrigger(p_buf->data[0], p_buf->data[1],
				p_buf->data[2], p_buf->data[3]);
		break;
	case PC_AutoControl_SpecialMotorPosition: {
		WordtoByte_Typedef ms;
		ms.byte[0] = p_buf->data[1];
		ms.byte[1] = p_buf->data[2];
		AutoControl_SpecialMotorPosition(p_buf->data[0], ms.word);
	}
		break;
	case PC_AutoControl_SpecialADCDoubleWithTrigger: {
		AutoControl_SpecialDoubleADCWithTrigger(p_buf->data[0], p_buf->data[1],
				p_buf->data[2], p_buf->data[3], p_buf->data[4]);
	}
		break;
	default:
		break;
	}
}

void Function::Setting(P_Buf_Typedef* p_buf) {
	switch (p_buf->pc) {
	case PC_Setting_SM_Speed: {
		WordtoByte_Typedef speed;
		TwoWordtoByte_Typedef acc;
		speed.byte[0] = p_buf->data[1];
		speed.byte[1] = p_buf->data[2];

		acc.byte[0] = p_buf->data[3];
		acc.byte[1] = p_buf->data[4];
		acc.byte[2] = p_buf->data[5];
		acc.byte[3] = p_buf->data[6];
		Setting_SM_Speed(p_buf->data[0], speed.word, acc.twoword);

		break;
	}
	case PC_Setting_Valve_Default: {
		TwoWordtoByte_Typedef status;
		status.byte[0] = p_buf->data[0];
		status.byte[1] = p_buf->data[1];
		status.byte[2] = p_buf->data[2];
		Setting_Valve_Default(status.twoword);

		break;
	}
	case PC_Setting_Encoder_Zero:
		Setting_Encoder_Zero(p_buf->data[0]);
		break;
	case PC_Setting_Protect_Limit:
		Setting_Protect_Limit(p_buf->data[0], p_buf->data[1], p_buf->data[2]);
		break;
	case PC_Setting_PIDParam: {
		DoubletoByte_Typedef p, i, d, set;
		uint8_t index = 1;
		for (uint8_t j = 0; j < 8; ++j) {
			p.byte[j] = p_buf->data[index++];
		}
		for (uint8_t j = 0; j < 8; ++j) {
			i.byte[j] = p_buf->data[index++];
		}
		for (uint8_t j = 0; j < 8; ++j) {
			d.byte[j] = p_buf->data[index++];
		}
		for (uint8_t j = 0; j < 8; ++j) {
			set.byte[j] = p_buf->data[index++];
		}
		Setting_PIDParam(p_buf->data[0], p, i, d, set);

		break;
	}
	case PC_Setting_PIDInput: {
		DoubletoByte_Typedef now;
		uint8_t index = 1;
		for (uint8_t j = 0; j < 8; ++j) {
			now.byte[j] = p_buf->data[index++];
		}

		Setting_PIDInput(p_buf->data[0], now);

		break;
	}
	case PC_Setting_PIDEnable:
		Setting_PIDEnable(p_buf->data[0], p_buf->data[1]);
		break;
	case PC_Setting_SM_RelDir:
		Setting_SM_RelDir(p_buf->data[0], p_buf->data[1]);
		break;
	case PC_Setting_USART:
		Setting_USART(p_buf->data[0]);
		break;
	case PC_Setting_Address:
		Setting_Address(p_buf->data[0]);
		break;
	default:
		break;
	}
	Protocol::Send(PC_Post_Complete, p_buf->pc);
}

void Function::Special(P_Buf_Typedef* p_buf) {
	switch (p_buf->pc) {
	case PC_Special_Reset:
		Special_Reset();
		break;
	case PC_Special_Stop:
		Special_Reset();
		break;
	case PC_Special_Continue:
		Special_Continue();
		break;
	case PC_Special_Cacel:
		Special_Cacel();
		break;
	default:
		break;
	}
}

void Function::Inquire_Limit() {
	uint8_t tmp[3];

	Limit::RefreshData();
	tmp[0] = Limit::Data;
	tmp[1] = 0x00;
	tmp[2] = 0x00;
	Protocol::Send(PC_Post_Complete, 3, PC_Inquire_Limit, tmp);
}

void Function::Inquire_ExLimit() {
	ExLimit::RefreshData();
	Protocol::Send(PC_Post_Complete, 3, PC_Inquire_ExLimit, ExLimit::Data.byte);
}

void Function::Inquire_Valve() {
	uint8_t tmp[3];
	tmp[0] = (uint8_t) PowerDev::Status;
	tmp[2] = (uint8_t) (PowerDev::Status >> 8);
	tmp[1] = (uint8_t) ((PowerDev::Status >> 16) & 0x7f);
	Protocol::Send(PC_Post_Complete, 3, PC_Inquire_Valve, tmp);
}

void Function::Inquire_Encoder(uint8_t no) {
	TwoWordtoByteSigned_Typedef count;
	switch (no) {
	case 2:
		count.twoword = U_Encoder1.Get();
		break;
	case 4:
		count.twoword = U_Encoder2.Get();
		break;
	default:
		break;
	}
	Protocol::Send(PC_Post_Complete, 4, PC_Inquire_Encoder, count.byte);
}
#pragma GCC diagnostic ignored "-Wunused-parameter"
void Function::Inquire_ADC(uint8_t no) {
	switch (no) {
	case 0:
		Protocol::Send(PC_Post_Complete, 2, PC_Inquire_ADC, U_ADC1::Data.byte);
		break;
	case 1: {
//		Protocol::Send(Salve_AC, PC_Inquire_ADC);
//		if (SPIBUS::CheckReady()) {
//			Protocol::Receive(Salve_AC, data.byte, 2);
//		}
//		Protocol::Send(PC_Post_Complete, 2, PC_Inquire_ADC, data.byte);
	}
		break;
	default:
		break;
	}
}

#pragma GCC diagnostic pop
void Function::Inquire_ExADC(uint8_t no) {
	ExADC::RefreshData();
	Protocol::Send(PC_Post_Complete, 2, PC_Inquire_ExADC, ExADC::Data[no].byte);
}

void Function::Inquire_Tigger() {
}

void Function::Inquire_Motor() {
	WordtoByteSigned_Typedef speed;
	speed.word = (PowerDev::Status & MotorMask) == 0 ? -TIM3->CCR4 : TIM3->CCR4;
	Protocol::Send(PC_Post_Complete, 2, PC_Inquire_Motor, speed.byte);
}
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
void Function::Inquire_Special(uint16_t num) {
	uint8_t tmp[4096];
	uint16_t index = 0;

	for (; index < num; index++) {
		tmp[index * 4] = ASBSWAAE_Pos[index].byte[0];
		tmp[index * 4 + 1] = ASBSWAAE_Pos[index].byte[1];
		tmp[index * 4 + 2] = ASBSWAAE_Pos[index].byte[2];
		tmp[index * 4 + 3] = ASBSWAAE_Pos[index].byte[3];
		tmp[index * 2 + (uint16_t) num * 4] = ASBSWAAE_ADC[index].byte[0];
		tmp[index * 2 + (uint16_t) num * 4 + 1] = ASBSWAAE_ADC[index].byte[1];
	}
	Protocol::Send(PC_Post_Complete, num * 6, PC_Inquire_Special, tmp);
}
#pragma GCC diagnostic pop

void Function::Inquire_DAC(uint8_t no) {
	switch (no) {
	case 0:
		break;
	case 1:
//		Protocol::Send(Salve_AC, PC_Inquire_DAC);
		break;
	default:
		break;
	}
}

void Function::Inquire_SpecialADCWithTime(uint16_t num) {
	uint8_t tmp[4096];
	uint16_t index = 0;
	for (; index < num; ++index) {
		tmp[index << 1] = ADCDATA[index].byte[0];
		tmp[(index << 1) + 1] = ADCDATA[index].byte[1];
	}
	Protocol::Send(PC_Post_Complete, num << 1, PC_Inquire_SpecialADCWithTime,
			tmp);
}

void Function::Inquire_SpecialADCWithTrigger(uint8_t num) {
	WordtoByteSigned_Typedef tmp[256];
	uint16_t index = 0;
	for (; index < num; ++index) {
		tmp[index].word = ADCDATA[index].word;
	}
	Protocol::Send(PC_Post_Complete, num << 1, PC_Inquire_SpecialADCTrigger,
			(uint8_t*) tmp);
}

void Function::Inquire_SpecialDoubleADCWithTrigger(uint8_t num) {
	WordtoByteSigned_Typedef tmp[256];
	uint16_t index = 0;
	for (; index < num; ++index) {
		tmp[index].word = ADCDATA[index].word;
	}
	for (; index < num << 1; ++index) {
		tmp[index].word = ADCDATA2[index - num].word;
	}
	Protocol::Send(PC_Post_Complete, num << 2,
			PC_Inquire_SpecialDoubleADCTrigger, (uint8_t*) tmp);
}

void Function::Inquire_SpecialStatus(PC_Typedef pc) {
	uint8_t status = 0x00;
	switch (pc) {
	case PC_AutoControl_SpecialADCWithTime:
		status = (uint8_t) AutoControl_SpecialADCWithTime_Busy;
		break;
	case PC_AutoControl_SpecialADCWithTrigger:
		status = (uint8_t) AutoControl_SpecialADCWithTrigger_Busy;
		break;
	case PC_AutoControl_SpecialADCDoubleWithTrigger:
		status = (uint8_t) AutoControl_SpecialDoubleADCWithTrigger_Busy;
		break;
	case PC_AutoControl_SpecialMotorPosition:
		status = (uint8_t) AutoControl_SpecialMotorPosition_Busy;
		break;
	default:
		break;
	}
	Protocol::Send(PC_Post_Complete, 1, PC_Inquire_SpecialStatus, &status);
}

void Function::Inquire_Status(uint8_t no) {
	uint8_t status = 0x00;
	switch (no) {
	case 1:
		status = (uint8_t) U_StepMotor1.IsBusy();
		break;
	case 2:
		status = (uint8_t) U_StepMotor2.IsBusy();
		break;
	default:

		break;
	}
	Protocol::Send(PC_Post_Complete, 1, PC_Inquire_Status, &status);
}

void Function::Control_Valve(uint32_t status) {
	PowerDev::Valve(status);
}

void Function::Control_Motor(int16_t speed) {
	PowerDev::Motor(speed);
}

void Function::Control_SM(uint8_t no, uint8_t status) {
	U_StepMotor::Dir_Typedef dir =
			status == 0 ? U_StepMotor::Dir_CW : U_StepMotor::Dir_CCW;
	switch (no) {
	case 1:
		if (status == 0xff) {
			U_StepMotor1.Stop();
		} else {
			U_StepMotor1.Run(dir);
		}
		break;
	case 2:
		if (status == 0xff) {
			U_StepMotor2.Stop();
		} else {
			U_StepMotor2.Run(dir);
		}
		break;
	default:
		break;
	}
}

void Function::Control_DAC(uint8_t no, uint16_t data) {
	switch (no) {
	case 0:
		U_DAC::RefreshData((uint16_t) (data & 0xfff));
		break;
	case 1:

		break;
	default:
		break;
	}
}

void Function::Control_ValveOpen(uint32_t status) {
	PowerDev::ValveOpen(status);
}

void Function::Control_ValveClose(uint32_t status) {
	PowerDev::ValveClose(status);
}

void Function::AutoControl_SM_By_Step(uint8_t no, int32_t step) {
	U_StepMotor::Dir_Typedef dir;
	if (step > 0) {
		dir = U_StepMotor::Dir_CW;
	} else {
		dir = U_StepMotor::Dir_CCW;
		step = -step;
	}
	switch (no) {
	case 1:
		U_StepMotor1.Move(step, dir);
		break;
	case 2:
		U_StepMotor2.Move(step, dir);
		break;
	default:
		break;
	}
}

void Function::AutoControl_SM_By_Limit(uint8_t no, uint8_t status,
		uint8_t limitNo) {
	U_StepMotor::Dir_Typedef dir =
			status == 0 ? U_StepMotor::Dir_CW : U_StepMotor::Dir_CCW;
	if (!AutoControl_SM_By_Limit_Judge(limitNo)) {
		return;
	}
	switch (no) {
	case 1:
		U_StepMotor1.Run(dir);
		while (AutoControl_SM_By_Limit_Judge(limitNo)) {
			if (!U_StepMotor1.IsBusy()) {
				break;
			}
		}
		U_StepMotor1.Stop();
		break;
	case 2:
		U_StepMotor2.Run(dir);
		while (AutoControl_SM_By_Limit_Judge(limitNo)) {
			if (!U_StepMotor2.IsBusy()) {
				break;
			}
		}
		U_StepMotor2.Stop();
		break;
	default:
		break;
	}
}

bool Function::AutoControl_SM_By_Limit_Judge(uint8_t limitNo) {
	Limit::RefreshData();
	return ((Limit::Data & (1 << limitNo)) == 0);
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
void Function::AutoControl_SM_By_Step_With_ADC_And_Encoder(uint8_t no,
		int32_t step, uint8_t encoderNo, uint8_t adcNo, uint16_t num) {
	uint16_t index = 0;
	AutoControl_SM_By_Step(no, step);
	switch (no) {
	case 1:
		while (U_StepMotor1.GetCurStep() < U_StepMotor1.GetTgtStep()) { //速度 10k 加速度100k时会导致步进电机自己停止
			if (!U_StepMotor1.IsBusy()) {
				break;
			}
			if (U_StepMotor1.GetCurStep()
					>= (U_StepMotor1.GetTgtStep() / num * index)) {
				switch (encoderNo) {
				case 2:
					ASBSWAAE_Pos[index].twoword = U_Encoder1.Get();
					break;
				case 4:
					ASBSWAAE_Pos[index].twoword = U_Encoder2.Get();
					break;
				default:
					break;
				}
				ExADC::RefreshData();
				ASBSWAAE_ADC[index].word = ExADC::Data[adcNo].word;
				index++;
			}
		}
		break;
	case 2:
		while (U_StepMotor2.GetCurStep() < U_StepMotor2.GetTgtStep()) {
			if (U_StepMotor2.GetCurStep()
					>= (U_StepMotor2.GetTgtStep() / num * index)) {
				if (!U_StepMotor2.IsBusy()) {
					break;
				}
				switch (encoderNo) {
				case 2:
					ASBSWAAE_Pos[index].twoword = U_Encoder1.Get();
					break;
				case 4:
					ASBSWAAE_Pos[index].twoword = U_Encoder2.Get();
					break;
				default:
					break;
				}
				ExADC::RefreshData();
				ASBSWAAE_ADC[index].word = ExADC::Data[adcNo].word;
				index++;
			}
		}
		break;
	default:
		break;
	}
}

#pragma GCC diagnostic pop

void Function::AutoControl_SpecialADCWithTime(uint8_t ms, uint8_t adcNo,
		uint16_t num) {
	AutoControl_SpecialADCWithTime_Busy = true;
	for (uint16_t i = 0; i < num; ++i) {
		U_SystemTick::WaitMilliSecond(ms);
		ExADC::RefreshData();
		ADCDATA[i].word = ExADC::Data[adcNo].word;
	}
	AutoControl_SpecialADCWithTime_Busy = false;
}

void Function::AutoControl_SpecialADCWithTrigger(uint8_t sensorNo,
		uint8_t moment, uint8_t adcNo, uint8_t num) {
	static uint64_t last;
	static uint64_t triggerTime;
	static uint32_t timeSpan;

	AutoControl_SpecialADCWithTrigger_Busy = true;

	Limit::Waitting(sensorNo); //等待到达凹槽
	last = U_SystemTick::GetMicroSecond();
	Limit::Waitting(sensorNo); //第二次到达凹槽
	timeSpan = U_SystemTick::GetMicroSecond() - last; //计算凹槽之间时间
	triggerTime = timeSpan / 255.0 * moment; //计算触发事件
	for (uint8_t i = 0; i < num; ++i) {
		LED::Turn(Color_Yellow);
		//等待到达采样位置所需时间
		U_SystemTick::WaitMicroSecond(triggerTime);
		//开始采样
		ExADC::RefreshData();
		ADCDATA[i].word = ExADC::Data[adcNo].word;
		//等待下一个凹槽
		LED::Turn(Color_Blue);
		Limit::Waitting(sensorNo);
	}
	//等待到达采样位置所需时间
	U_SystemTick::WaitMicroSecond(triggerTime);
	//在采样位置停止
	PowerDev::Motor(0);

	AutoControl_SpecialADCWithTrigger_Busy = false;
}

void Function::AutoControl_SpecialDoubleADCWithTrigger(uint8_t sensorNo,
		uint8_t moment, uint8_t adcNo, uint8_t adcNo2, uint8_t num) {
	static uint64_t last;
	static uint64_t triggerTime;
	static uint32_t timeSpan;

	AutoControl_SpecialDoubleADCWithTrigger_Busy = true;

	Limit::Waitting(sensorNo); //等待到达凹槽
	last = U_SystemTick::GetMicroSecond();
	Limit::Waitting(sensorNo); //第二次到达凹槽
	timeSpan = U_SystemTick::GetMicroSecond() - last; //计算凹槽之间时间
	triggerTime = timeSpan / 255.0 * moment; //计算触发事件
	for (uint8_t i = 0; i < num; ++i) {
		LED::Turn(Color_Yellow);
		//等待到达采样位置所需时间
		U_SystemTick::WaitMicroSecond(triggerTime);
		//开始采样
		ExADC::RefreshData();
		ADCDATA[i].word = ExADC::Data[adcNo].word;
		ADCDATA2[i].word = ExADC::Data[adcNo2].word;
		//等待下一个凹槽
		LED::Turn(Color_Blue);
		Limit::Waitting(sensorNo);
	}
	//等待到达采样位置所需时间
	U_SystemTick::WaitMicroSecond(triggerTime);
	//在采样位置停止
	PowerDev::Motor(0);

	AutoControl_SpecialDoubleADCWithTrigger_Busy = false;
}

void Function::AutoControl_SpecialMotorPosition(uint8_t sensorNo, uint16_t ms) {
	AutoControl_SpecialMotorPosition_Busy = true;

	Limit::Waitting(sensorNo);
	U_SystemTick::WaitMilliSecond(ms);
	PowerDev::Motor(0);

	AutoControl_SpecialMotorPosition_Busy = false;
}

void Function::Setting_SM_Speed(uint8_t no, uint16_t speed, uint32_t tgtAcc) {
	switch (no) {
	case 1:
		U_StepMotor1.SetSpeed(speed, tgtAcc);
		break;
	case 2:
		U_StepMotor2.SetSpeed(speed, tgtAcc);
		break;
	default:
		break;
	}
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
void Function::Setting_Valve_Default(uint32_t status) {

}

#pragma GCC diagnostic pop

void Function::Setting_Encoder_Zero(uint8_t no) {
	switch (no) {
	case 2:
		U_Encoder1.Set(0);
		break;
	case 4:
		U_Encoder2.Set(0);
		break;
	default:
		break;
	}
}

void Function::Setting_Protect_Limit(uint8_t no, uint8_t status,
		uint8_t limitNo) {
	U_StepMotor::Dir_Typedef dir =
			status == 0 ? U_StepMotor::Dir_CW : U_StepMotor::Dir_CCW;
	if (limitNo == 0xff) {
		limitNo = 0;
	}
	uint8_t limit = 1 << limitNo;
	switch (no) {
	case 1:
		switch (dir) {
		case U_StepMotor::Dir_CW:
			Setting::MainSetting.Unit.U_StepMotor1_CWLimit = limit;
			U_StepMotor1.SetCWLimit(
					Setting::MainSetting.Unit.U_StepMotor1_CWLimit);
			break;
		case U_StepMotor::Dir_CCW:
			Setting::MainSetting.Unit.U_StepMotor1_CCWLimit = limit;
			U_StepMotor1.SetCCWLimit(
					Setting::MainSetting.Unit.U_StepMotor1_CCWLimit);
			break;
		default:
			break;
		}
		break;
	case 2:
		switch (dir) {
		case U_StepMotor::Dir_CW:
			Setting::MainSetting.Unit.U_StepMotor2_CWLimit = limit;
			U_StepMotor2.SetCWLimit(
					Setting::MainSetting.Unit.U_StepMotor2_CWLimit);
			break;
		case U_StepMotor::Dir_CCW:
			Setting::MainSetting.Unit.U_StepMotor2_CCWLimit = limit;
			U_StepMotor2.SetCCWLimit(
					Setting::MainSetting.Unit.U_StepMotor2_CCWLimit);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	Setting::Save();
}

void Function::Setting_PIDParam(uint8_t no, DoubletoByte_Typedef p,
		DoubletoByte_Typedef i, DoubletoByte_Typedef d,
		DoubletoByte_Typedef set) {
	switch (no) {
	case 0:
		PID.SetTunings(p.d, i.d, d.d);
		PIDParam.SetPoint = set.d;
		PID.Clear();
		break;
	case 1: {
//		uint8_t data[33];
//		uint8_t index = 0;
//		for (uint8_t j = 0; j < 8; ++j) {
//			data[index++] = p.byte[j];
//		}
//		for (uint8_t j = 0; j < 8; ++j) {
//			data[index++] = i.byte[j];
//		}
//		for (uint8_t j = 0; j < 8; ++j) {
//			data[index++] = d.byte[j];
//		}
//		for (uint8_t j = 0; j < 8; ++j) {
//			data[index++] = set.byte[j];
//		}
//		Protocol::Send(Salve_AC, PC_Setting_PIDParam, 33, data);
	}
		break;
	default:
		break;
	}
}

void Function::Setting_PIDInput(uint8_t no, DoubletoByte_Typedef now) {
	switch (no) {
	case 0:
		PIDParam.Input = now.d;
		break;
	case 1:
		break;
	default:
		break;
	}
}

void Function::Setting_PIDEnable(uint8_t no, uint8_t state) {
	switch (no) {
	case 0:
		if (state != 0) {
			PIDEnable = true;
		} else {
			PIDEnable = false;
			U_DAC::RefreshData((uint16_t) 2048);
		}
		break;
	case 1:
		break;
	default:
		break;
	}
}

void Function::Setting_SM_RelDir(uint8_t no, uint8_t status) {
	U_StepMotor::Dir_Typedef dir =
			status == 0 ? U_StepMotor::Dir_CW : U_StepMotor::Dir_CCW;
	switch (no) {
	case 1:
		U_StepMotor1.SetRelativeDir(dir);
		Setting::MainSetting.Unit.U_StepMotor1_RelativeDir = dir;
		break;
	case 2:
		U_StepMotor2.SetRelativeDir(dir);
		Setting::MainSetting.Unit.U_StepMotor2_RelativeDir = dir;
		break;
	default:
		break;
	}
	Setting::Save();
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
void Function::Setting_USART(uint8_t com) {
}

void Function::Setting_Address(uint8_t add) {
	Setting::MainSetting.Unit.SalveAddress = add;
	Protocol::DevAdd = Setting::MainSetting.Unit.SalveAddress;
	Setting::Save();
}
#pragma GCC diagnostic pop

void Function::Special_Reset() {
	NVIC_SystemReset();
}

void Function::Special_Stop() {
}

void Function::Special_Continue() {
}

void Function::Special_Cacel() {
}
