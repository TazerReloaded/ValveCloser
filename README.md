# ValveCloser
Firmware for a device to control a manual ball valve with the BTT S42B V2.0.

# Purpose
Intended for use as a "smart servo", you set the desired position and it goes there, reporting errors if any. Could also be used as a "minimal" firmware for experiments. Motor movement is only controlled by the encoder reading, no complicated step calculations. If the motor does not move the expected amount in a given interval, an error is reported. Same goes for over-temperature and encoder errors. No LCD support, configuration in header file, minimal serial communication.

# License
Copyright (c) 2021 TazerReloaded
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
