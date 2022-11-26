/*
	The MIT License (MIT)
	
	Copyright (c) 2019 Lance A. Endres
  Copyright (c) 2022 Derrian D. Frederick
	
	Permission is hereby granted, free of charge, to any person obtaining a copy of
	this software and associated documentation files (the "Software"), to deal in
	the Software without restriction, including without limitation the rights to
	use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
	the Software, and to permit persons to whom the Software is furnished to do so,
	subject to the following conditions:
	
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
	FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
	COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
	IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
	CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "UpDownCycleButton.h"

// Constructors.
UpDownCycleButton::UpDownCycleButton(int pin, byte* value) :
	ResettableButton(pin),
	_minValue(ZEROBASED),
	_maxValue(2),
	_value(value),
  _direction(INCREMENT)
{
}

UpDownCycleButton::UpDownCycleButton(int pin, byte* value, int maxValue) :
	ResettableButton(pin),
	_minValue(ZEROBASED),
	_maxValue(maxValue),
	_value(value),
  _direction(INCREMENT)
{
}

UpDownCycleButton::UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, int maxValue) :
	ResettableButton(pin),
	_minValue(minValue),
	_maxValue(maxValue),
	_value(value),
  _direction(INCREMENT)
{
}

UpDownCycleButton::UpDownCycleButton(int pin, byte* value, DIRECTION direction, int maxValue) :
	ResettableButton(pin),
	_minValue(ZEROBASED),
	_maxValue(maxValue),
	_value(value),
  _direction(direction)
{
}

UpDownCycleButton::UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, DIRECTION direction, int maxValue) :
	ResettableButton(pin),
	_minValue(minValue),
	_maxValue(maxValue),
	_value(value),
  _direction(direction)
{
}

//UpDownCycleButton::UpDownCycleButton(int pin, byte* value, int debounceInterval) :
//	ResettableButton(pin, debounceInterval),
//	_minValue(ZEROBASED),
//	_maxValue(2),
//	_value(value),
//  _direction(INCREMENT)
//{
//}
//
//UpDownCycleButton::UpDownCycleButton(int pin, byte* value, int maxValue, int debounceInterval) :
//	ResettableButton(pin, debounceInterval),
//	_minValue(ZEROBASED),
//	_maxValue(maxValue),
//	_value(value),
//  _direction(INCREMENT)
//{
//}
//
//UpDownCycleButton::UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, int maxValue, int debounceInterval) :
//	ResettableButton(pin, debounceInterval),
//	_minValue(minValue),
//	_maxValue(maxValue),
//	_value(value),
//  _direction(INCREMENT)
//{
//}
//
//UpDownCycleButton::UpDownCycleButton(int pin, byte* value, DIRECTION direction, int maxValue, int debounceInterval) :
//	ResettableButton(pin, debounceInterval),
//	_minValue(ZEROBASED),
//	_maxValue(maxValue),
//	_value(value),
//  _direction(direction)
//{
//}
//
//UpDownCycleButton::UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, DIRECTION direction, int maxValue, int debounceInterval) :
//	ResettableButton(pin, debounceInterval),
//	_minValue(minValue),
//	_maxValue(maxValue),
//	_value(value),
//  _direction(direction)
//{
//}
//

// Destructor.
UpDownCycleButton::~UpDownCycleButton()
{
}

// Set the starting value.
void UpDownCycleButton::setMinimum(CYCLEBASE base)
{
	_minValue = base;
}

// Set the maximum value.
void UpDownCycleButton::setMaximum(int maxValue)
{
	_maxValue = maxValue;
}

// Set the direction.
void UpDownCycleButton::setDirection(DIRECTION direction)
{
  _direction = direction;
}

void UpDownCycleButton::changeDirection()
{
	if (_direction == INCREMENT)
	{
		_direction = DECREMENT;
	}
	else
	{
		_direction = INCREMENT;
	}

}

bool UpDownCycleButton::updateValue()
{
	// Catch transitions from HIGH to LOW.
	switch (update())
	{
		case BUTTONSUITE::WASSHORTPRESSED:
		{
			if (_direction)
			{
				incrementValue();
			}
      else
      {
        decrementValue();
      }
			break;
		}

		case BUTTONSUITE::WASLONGPRESSED:
		{
			// If the reset on long press mode is enabled, we reset, otherwise we treat
			// this as a regular buttom push.
			if (_resetOnLongPress)
			{
				reset();
			}
			else if (_direction)
			{
				incrementValue();
			}
      else
      {
        decrementValue();
      }
			break;
		}

		default:
		{
      return false;     //False is button was not short pressed or long pressed
			break;
		}
	}

	 return true;         //Return true if button wan short pressed or long pressed.
}

byte UpDownCycleButton::getDirection()
{
  return _direction;
} 

void UpDownCycleButton::reset()
{
	*_value = _minValue;
}

void UpDownCycleButton::incrementValue()
{
	// Button was pushed, so increment counter.
	(*_value)++;

	// If we go over the max value, reset.
	if (*_value > _maxValue)
	{
		reset();
	}
}

void UpDownCycleButton::decrementValue()
{
	// Button was pushed, so decrement counter, but if value was zero then reset.
	if (*_value != 0)
	{
		(*_value)--;
	}
	else
	{
		reset();
	}

	// If we go under the min value, reset.
	if (*_value < _minValue )
	{
		reset();
	}
}