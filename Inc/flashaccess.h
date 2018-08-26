

#ifdef FLASH_STORAGE
int flashposn(int *len);
int readFlash( unsigned char *data, int len );
int writeFlash( unsigned char *data, int len );
#endif
