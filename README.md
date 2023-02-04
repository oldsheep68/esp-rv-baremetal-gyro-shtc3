# esp-rv-baremetal-gyro-shtc3

Within the examples directory, there are four examples, showing how to use the FIFO access of icm42670.

There is an example for each packet structure.
The advantage of accessing through the fifo is, that all the different values are valied on a single time stamp. The time intervall between the different packets are in some structures available.
Furtheremore, fifo-access over i2c is very efficient and takes much less time than reading each register indvidualy.
At the time of writing, you need the  additional features contained in https://github.com/oldsheep68/icm42670.git, to make  this examples compile.
Within the fifo_p3 example, a sensore fustion library is integrated, which returns and keeps you the actual position of roll, yaw and pitch angles. On the esp32-c3 rust board, a full loop takes near to 5ms of calculation (because there is no fpu in the chip)
All examples are tested on the esp32-c3 rust board: https://github.com/esp-rs/esp-rust-board, which contains the sensor and has I2C access to it.
