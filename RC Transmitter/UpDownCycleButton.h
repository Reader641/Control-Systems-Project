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

/*
	Turns a push button (momentary button) into a button used to cycle through states.
	Pressing the button causes the value to increment or decrement depending an internal state.
  Once the maximum or minimum value	is reached, the value automatically resets to the initial value.
  The value can also be reset to the initial value by the user (with a long press, if enabled) or
	programmically reset.

	For example:
	If the minimum value is set to 0 and the maximum is set to 4, the button will
	initially return 0.  Repeated presses of the button will return 1-2-3-4, in order.
	A fifth press of the button will return 0 and the cycle will continue in the other direction
  if the internal state is set to decrement.
*/

/*
	To use a button with this library, the button should be wired with one side
	connected to the Arduino pin and the other side connected to ground such that
	when the button is pressed, the pin is brought to ground (LOW).
*/

#ifndef UPDOWNCYCLEBUTTON_H
#define UPDOWNCYCLEBUTTON_H

#include <Arduino.h>
#include "ResettableButton.h"

class UpDownCycleButton : public ResettableButton
{
	// Enums.
	public:
		// Specifies the base number to count from.  It's the number that will be returned if no button
		// pushes have been detected.  The default is zero.
		enum CYCLEBASE
		{
			ZEROBASED,
			ONEBASED
		};

    enum DIRECTION
    {
      DECREMENT,
      INCREMENT
    };

	public:
    UpDownCycleButton(int pin, byte* value);
    UpDownCycleButton(int pin, byte* value, int maxValue);
    UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, int maxValue);
    UpDownCycleButton(int pin, byte* value, DIRECTION direction, int maxValue);
    UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, DIRECTION direction, int maxValue);
//   UpDownCycleButton(int pin, byte* value, int debounceInterval);
//   UpDownCycleButton(int pin, byte* value, int maxValue, int debounceInterval);
//   UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, int maxValue, int debounceInterval);
//   UpDownCycleButton(int pin, byte* value, DIRECTION direction, int maxValue, int debounceInterval);
//   UpDownCycleButton(int pin, byte* value, CYCLEBASE minValue, DIRECTION direction, int maxValue, int debounceInterval);

		~UpDownCycleButton();

	// Set up functions.  Normally, these would be called in the "setup" function of your sketch.
	public:
		// Set the starting value.  It can be a zero based or one based counter.
		void setMinimum(CYCLEBASE base);

		// Set the maximum value.
		void setMaximum(int maxValue);

    // Set the direction value. It can be an incremnting button or decrementing button
		void setDirection(DIRECTION direction);

		// Changes the direction of the button to the opposite. It can be an incremnting button or decrementing button
		void changeDirection();

	// Status access functions.  Call in the "loop" to get the status of the button.
	public:
		// Checks the button and returns the current state, updates the value(toggled on or off).
		bool updateValue();
    void setValue(byte* value);

    byte getDirection();   //Returns the direction of the button

	// Other control functions.
	public:
		// Return to default state.
		void reset();

	private:
		// Increment the value to the next one or reset if the maximum has been reached.
		void incrementValue();

    // Decrement the value to the previous one or reset if the minimum has been reached.
		void decrementValue();

	private:
		byte _minValue;
		int _maxValue;
		byte* _value;
    byte _direction;
};

#endif