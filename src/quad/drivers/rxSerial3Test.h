#ifndef __RXSERIAL3TEST_H
#define __RXSERIAL3TEST_H

void rxSerial3TestInit(void);
//void gpsSetPrintfSerialPort(void);
void rxSerial3TestWrite(uint8_t ch);
uint8_t rxSerial3TestRead(void);
void rxSerial3TestPrint(const char *str);

#endif	// __RXSERIALTEST_H
