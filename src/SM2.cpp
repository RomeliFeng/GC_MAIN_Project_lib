/*
 * SM2.cpp
 *
 *  Created on: 2017年6月29日
 *      Author: Romeli
 */

#include <SM2.h>
#include "Limit.h"

#define TIM_ACC TIM6
#define TIM_ACC_IRQn TIM6_IRQn
#define TIM_ACC_RCC RCC_APB1Periph_TIM6
#define TIM_PUL TIM5
#define TIM_PUL_IRQn TIM5_IRQn
#define TIM_PUL_RCC RCC_APB1Periph_TIM5

#define PUL_PIN GPIO_Pin_0
#define DIR_PIN GPIO_Pin_1
#define EN_PIN GPIO_Pin_2

#define DIR_SET GPIO_SetBits(GPIOA, DIR_PIN);
#define DIR_RESET GPIO_ResetBits(GPIOA, DIR_PIN);
#define EN_SET GPIO_SetBits(GPIOA, EN_PIN);
#define EN_RESET GPIO_ResetBits(GPIOA, EN_PIN);

#define STARTSPEED 200

uint32_t SM2::TgtStep = 10000;
uint32_t SM2::CurStep = 0;
uint32_t SM2::GearStep = 0;
uint32_t SM2::TgtAcc = 200000;
uint16_t SM2::MaxSpeed = 20000;
SM_DIR_Typedef SM2::CurDir = SM_DIR_Upward;
bool SM2::SpeedAcc = true;
bool SM2::NoStep = false;
bool SM2::FullSpeed = false;
bool SM2::GearSpeed = false;
bool SM2::Busy = false;

//摩擦试验机
//uint8_t SM2::UpwardLimit = 0x20;
//uint8_t SM2::BackwardLimit = 0x08;
//SM_DIR_Typedef SM2::DefaultDir = SM_DIR_Backward;

uint8_t SM2::UpwardLimit = 0x04;
uint8_t SM2::BackwardLimit = 0x08;
SM_DIR_Typedef SM2::DefaultDir = SM_DIR_Upward;

void SM2::Init() {
	GPIOInit();
	TIMInit();
	NVICInit();
}

void SM2::Move(uint32_t step, SM_DIR_Typedef dir) {
	TIM_Cmd(TIM_ACC, DISABLE);
	TIM_Cmd(TIM_PUL, DISABLE);

	Limit::RefreshData();
	switch (dir) {
	case SM_DIR_Upward:
		if ((Limit::Data & UpwardLimit) != 0) {
			return;
		}
		break;
	case SM_DIR_Backward:
		if ((Limit::Data & BackwardLimit) != 0) {
			return;
		}
		break;
	default:
		break;
	}

	EN_RESET
	;
	//设置方向
	SetDir(dir);

	GearSpeed = false;
	NoStep = false;
	CurStep = 0;
	TgtStep = step;
	Busy = true;

	if (SpeedAcc) {
		FullSpeed = false;
		//计算减速区间
		uint32_t airStep = ((uint64_t) (MaxSpeed - STARTSPEED)
				* (uint64_t) MaxSpeed / TgtAcc) >> 1;
		if ((TgtStep >> 1) > airStep) {
			//大于两倍空中时间，完整的加减速曲线
			GearStep = airStep;
		} else {
			GearStep = TgtStep >> 1;
		}
		//设置加速度定时器当前速度为200
		TIM_ACC->CNT = STARTSPEED;

		//开始计算初始速度
		TIM_PUL->ARR = SystemCoreClock / TIM_ACC->CNT >> 3; //默认八分频
		TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;

		//开始
		TIM_Cmd(TIM_ACC, ENABLE);
		TIM_Cmd(TIM_PUL, ENABLE);
	} else {
		FullSpeed = true;

		//设置速度；不加速
		TIM_ACC->CNT = MaxSpeed;

		//开始计算初始速度
		TIM_PUL->ARR = SystemCoreClock / TIM_ACC->CNT >> 3; //默认八分频
		TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;

		//开始 无加速过程
//		TIM_Cmd(TIM_ACC, ENABLE);
		TIM_Cmd(TIM_PUL, ENABLE);
	}
}

void SM2::Run(SM_DIR_Typedef dir) {
	TIM_Cmd(TIM_ACC, DISABLE);
	TIM_Cmd(TIM_PUL, DISABLE);

	Limit::RefreshData();
	switch (dir) {
	case SM_DIR_Upward:
		if ((Limit::Data & UpwardLimit) != 0) {
			return;
		}
		break;
	case SM_DIR_Backward:
		if ((Limit::Data & BackwardLimit) != 0) {
			return;
		}
		break;
	default:
		break;
	}

	EN_RESET
	;
//设置方向
	SetDir(dir);

	NoStep = true;
	Busy = true;

	if (SpeedAcc) {
		FullSpeed = false;

		//设置加速度定时器当前速度为200
		TIM_ACC->CNT = STARTSPEED;

		//开始计算初始速度
		TIM_PUL->ARR = SystemCoreClock / TIM_ACC->CNT >> 3; //默认八分频
		TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;

		//开始
		TIM_Cmd(TIM_ACC, ENABLE);
		TIM_Cmd(TIM_PUL, ENABLE);
	} else {
		FullSpeed = true;

		//设置速度；不加速
		TIM_ACC->CNT = MaxSpeed;

		//开始计算初始速度
		TIM_PUL->ARR = SystemCoreClock / TIM_ACC->CNT >> 3; //默认八分频
		TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;

		//开始 无加速过程
//		TIM_Cmd(TIM_ACC, ENABLE);
		TIM_Cmd(TIM_PUL, ENABLE);
	}
}

void SM2::Stop() {
	TIM_Cmd(TIM_ACC, DISABLE);
	TIM_Cmd(TIM_PUL, DISABLE);
	FullSpeed = false;
	GearSpeed = false;
	Busy = false;
}

void SM2::SetSpeed(uint16_t speed, uint32_t tgtAcc) {
	if (tgtAcc < 1500) {
		SpeedAcc = false;
	} else {
		SpeedAcc = true;
		TgtAcc = tgtAcc;
	}
	MaxSpeed = speed < 200 ? 200 : speed;
	TIM_Cmd(TIM_ACC, DISABLE);
	TIM_ACC->PSC = SystemCoreClock / TgtAcc - 1;
	TIM_ACC->ARR = MaxSpeed;
	TIM_Cmd(TIM_ACC, ENABLE);
}

void SM2::Unlock() {
	EN_SET
	;
}

void SM2::SetDir(SM_DIR_Typedef dir) {
	CurDir = dir;
	if (dir == DefaultDir) {
		DIR_SET
		;
	} else {
		DIR_RESET
		;
	}
}

void SM2::GPIOInit() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = PUL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DIR_PIN | EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, EN_PIN);
	GPIO_ResetBits(GPIOA, PUL_PIN);
}

void SM2::TIMInit() {
	RCC_APB1PeriphClockCmd(TIM_PUL_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(TIM_ACC_RCC, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_DeInit(TIM_ACC);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock / TgtAcc - 1; //根据加速度计算
	TIM_TimeBaseInitStructure.TIM_Period = MaxSpeed; //计数器的值为当前速度
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM_ACC, &TIM_TimeBaseInitStructure);

//初始化脉冲发生定时器
	TIM_DeInit(TIM_PUL);
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7;  //8分频 最低速度为 138
	TIM_TimeBaseInitStructure.TIM_Period = 36000; //通过当前加速度计算周期
	TIM_TimeBaseInit(TIM_PUL, &TIM_TimeBaseInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //关闭互补输出
	TIM_OCInitStructure.TIM_Pulse = MaxSpeed / TIM_ACC->CNT >> 1; //脉冲宽度
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //高电平有效
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //低电平有效
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; //低电平
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //低电平
	TIM_OC1Init(TIM_PUL, &TIM_OCInitStructure); //初始化
}

void SM2::NVICInit() {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM_ACC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM_PUL_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM_ACC, TIM_IT_Update);
	TIM_ClearITPendingBit(TIM_PUL, TIM_IT_Update);
	TIM_ITConfig(TIM_ACC, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM_PUL, TIM_IT_Update, ENABLE);
}

extern "C" {

void TIM6_IRQHandler(void) {
	TIM_ACC->CR1 &= (uint16_t) (~((uint16_t) TIM_CR1_CEN)); //关闭ACC累加
	TIM_ACC->CNT = SM2::MaxSpeed;
	TIM_ClearITPendingBit(TIM_ACC, TIM_IT_Update);
}

void TIM5_IRQHandler(void) {
	TIM_PUL->SR = (uint16_t) ~TIM_IT_Update;
	SM2::CurStep++;
//预定步数未到，继续累加
	if ((SM2::CurStep < SM2::TgtStep) || SM2::NoStep) {
		//有加减速
		if (SM2::SpeedAcc) {
			if (((SM2::TgtStep - SM2::CurStep) > SM2::GearStep)
					|| SM2::NoStep) {
				//未到达减速区间
				if (SM2::FullSpeed == false) {
					if (TIM_ACC->CNT == SM2::MaxSpeed) {
						SM2::FullSpeed = true;
						TIM_PUL->ARR = SystemCoreClock / SM2::MaxSpeed >> 3;
						TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;
					} else {
						TIM_PUL->ARR = SystemCoreClock / TIM_ACC->CNT >> 3;
						TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;
					}
				}
			} else {
				//到达减速区间
				if (SM2::GearSpeed == false) {
					//开始进行减速计算
					TIM_Cmd(TIM_ACC, DISABLE);
					TIM_ACC->CNT = SM2::MaxSpeed - TIM_ACC->CNT;
					TIM_Cmd(TIM_ACC, ENABLE);
					SM2::GearSpeed = true;
				} else {
					//根据减速计算速度
					uint16_t speed = TIM_ACC->ARR - TIM_ACC->CNT;
					//限制最小速度
					speed = speed < 200 ? 200 : speed;
					TIM_PUL->ARR = SystemCoreClock / speed >> 3;
					TIM_PUL->CCR1 = TIM_PUL->ARR >> 1;
				}
			}
		}
	} else {
		TIM_PUL->CR1 &= (uint16_t) (~((uint16_t) TIM_CR1_CEN)); //到达指定步数，停止输出脉冲
		TIM_ACC->CR1 &= (uint16_t) (~((uint16_t) TIM_CR1_CEN)); //到达指定步数，停止计算加速度
		SM2::FullSpeed = false;
		SM2::GearSpeed = false;
		SM2::Busy = false;
	}
}

}

