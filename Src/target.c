/*
 * Copyright (C) 2017 Obermaier Johannes
 *
 * This Source Code Form is subject to the terms of the MIT License.
 * If a copy of the MIT License was not distributed with this file,
 * you can obtain one at https://opensource.org/licenses/MIT
 */

#include "target.h"

void targetSysCtrlInit( void )
{
	targetSysOff();
	targetSysReset();
}

void targetSysReset( void )
{
  HAL_GPIO_WritePin(TARGET_RESET_GPIO_Port, TARGET_RESET_Pin, GPIO_PIN_RESET);
}

void targetSysUnReset( void )
{
  HAL_GPIO_WritePin(TARGET_RESET_GPIO_Port, TARGET_RESET_Pin, GPIO_PIN_SET);
}


void targetSysOff( void )
{
  HAL_GPIO_WritePin(TARGET_PWR_GPIO_Port, TARGET_PWR_Pin, GPIO_PIN_RESET);
}

void targetSysOn( void )
{
  HAL_GPIO_WritePin(TARGET_PWR_GPIO_Port, TARGET_PWR_Pin, GPIO_PIN_SET);
}
