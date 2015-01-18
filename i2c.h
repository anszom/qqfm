#ifndef I2C_H
#define I2C_H
void i2c_send(uint8_t reg, uint8_t byte);
inline  static void i2c_send_word(uint8_t reg, uint8_t b0, uint8_t b1) {}
int i2c_init(uint8_t addr);
void i2c_writeData(const uint8_t *data, uint8_t size);
#endif
