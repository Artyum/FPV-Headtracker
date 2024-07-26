#include "i2c.h"

//Function to initialize master
void i2c_init(void) {
	//TWSR = 0;		//Prescaler: 1 (default)
	TWBR = 12;		//400KHz @ 16MHz
}

uint8_t i2c_start(void) {
	uint8_t i=I2C_ILOOP;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  // Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
	while ((i) && (!I2C_TWINTSET)) i--; // Wait till start condition is transmitted //czeka a¿ TWINT bedzie wysokie
	if (I2C_TWINTSET) return I2C_RETTWSR; else return 0; //TWSR & 0xF8 - zamaskowanie preskalera w TWSR
}

uint8_t i2c_write_data(uint8_t data) {
	uint8_t i=I2C_ILOOP;
	TWDR=data; // put data in TWDR
	TWCR = (1<<TWINT) | (1<<TWEN);
	while ((i) && (!I2C_TWINTSET)) i--;
	if (I2C_TWINTSET) return I2C_RETTWSR; else return 0;
}

uint8_t i2c_read_data_ack(uint8_t *data) {
	uint8_t i=I2C_ILOOP;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while ((i) && (!I2C_TWINTSET)) i--;
	if (I2C_TWINTSET) {
		*data = TWDR;
		return I2C_RETTWSR;
	}
	else return 0;
}

uint8_t i2c_read_data_nack(uint8_t *data) {
	uint8_t i=I2C_ILOOP;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while ((i) && (!I2C_TWINTSET)) i--;
	if (I2C_TWINTSET) {
		*data = TWDR;
		return I2C_RETTWSR;
	}
	else return 0;
}

uint8_t i2c_stop(void) {
	uint8_t i=I2C_ILOOP;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	while ((i) && (!I2C_TWINTSET)) i--;
	if (I2C_TWINTSET) return I2C_RETTWSR; else return 0;
}

uint8_t i2c_read_byte(uint8_t sla, uint8_t adr, uint8_t *data) {
	//Start
	if (i2c_start() != 0x08) { i2c_stop(); return 0; }

	//Send address SLAVE + WRITE
	if (i2c_write_data((sla<<1) | I2C_WRITE) != 0x18) { i2c_stop(); return 0; }

	//Send subaddress
	if (i2c_write_data(adr) != 0x28 ) { i2c_stop(); return 0; }

	//Repeated Start
	if (i2c_start() != 0x10) { i2c_stop(); return 0; }

	//Send address SLAVE + READ
	if (i2c_write_data((sla<<1) | I2C_READ) != 0x40) { i2c_stop(); return 0; }

	//Data read
	if (i2c_read_data_nack(data) != 0x58) { i2c_stop(); return 0; }

	i2c_stop();
	return 1;
}

uint8_t i2c_write_byte(const uint8_t sla, const uint8_t adr, uint8_t data) {
	//Start
	if (i2c_start() != 0x08) { i2c_stop(); return 0; }

	//Send address SLAVE + WRITE
	if (i2c_write_data((sla<<1) | I2C_WRITE) != 0x18) { i2c_stop(); return 0; }

	//Send subaddress
	if (i2c_write_data(adr) != 0x28 ) { i2c_stop(); return 0; }

	//Send byte
	if(i2c_write_data(data) != 0x28 ) { i2c_stop(); return 0; }

	i2c_stop();
	return 1;
}

uint8_t i2c_read_sequence(const uint8_t sla, uint8_t adr, uint8_t dest[], const uint8_t cnt) {
	//Start
	if (i2c_start() != 0x08) { i2c_stop(); return 0; }

	//Send address SLAVE + WRITE
	if (i2c_write_data((sla<<1) | I2C_WRITE) != 0x18) { i2c_stop(); return 0; }

	//Send subaddressu + autoincrement
	if (sla == I2C_ADDR_M) adr |= 0x80;
	if (i2c_write_data(adr) != 0x28 ) { i2c_stop(); return 0; }

	//Repeated Start
	if (i2c_start() != 0x10) { i2c_stop(); return 0; }

	//Send address SLAVE + READ
	if (i2c_write_data((sla<<1) | I2C_READ) != 0x40) { i2c_stop(); return 0; }

	//Data read
	for (uint8_t i=0; i<cnt; i++) {
		if (i < cnt-1) {
			//With ACK
			if (i2c_read_data_ack(&dest[i]) != 0x50) { i2c_stop(); return 0; }
		}
		else {
			//With NACK - last byte
			if (i2c_read_data_nack(&dest[i]) != 0x58) { i2c_stop(); return 0; }
		}
	}

	i2c_stop();
	return 1;
}
