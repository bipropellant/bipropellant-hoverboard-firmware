

#ifdef FLASH_STORAGE
int flashposn(int *len);
int readFlash( unsigned char *data, int len );
int writeFlash( unsigned char *data, int len );
unsigned short readFlash16( volatile unsigned short *data, uint16_t len );
unsigned short writeFlash16( volatile unsigned short *data, uint16_t len );
#endif
