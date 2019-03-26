# PWMDriver
A driver for the Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685.
http://www.adafruit.com/product/815

Built for the NVIDIA Jetson TX2 Development Kit.

In order to run the example, edit example/servoExample.cpp and uncomment the lines:

Install:

<blockquote>$ sudo apt-get install libi2c-dev i2c-tools</blockquote>

As an example, after installation, in a Terminal execute:

<blockquote>$ sudo i2cdetect -y -r 1</blockquote>

