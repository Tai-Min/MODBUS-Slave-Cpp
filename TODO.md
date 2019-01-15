* fix some bytecount/ quantity etc from uint8 to uint16
* add illegal data error for quantities that are greater than in doc (ie max 0x7b0 coils)
* allow to write coils and holding registers
* add slave device busy exception
* add readAsFloat and readAsDouble
* update unit tests
