#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "comms.h"

#ifdef FLASH_STORAGE


// pages are 2k=0x400 bytes

// from flasharea.c, which reserves a page of flash for our use.
extern int flashlen;
extern volatile const uint8_t flash_data[];


// routines to read and write config data
// we will write at incrementing locations through the flash until we exhaust it, then erase and start over.

// data will be written 32 bit aligned, with the last long being a length.
// everything AFTER will still be 0xFFFFFFFF
int writeFlash( unsigned char *data, int len ){
    char tmp[40];
    unsigned short *p = (unsigned short *)flash_data;
    int i = (flashlen/2) - 1;
    for (; i > 0; i--){
        if (p[i] != 0xffff){
            break;
        }
    }
    // if we found something, we want to start on the short after it.
    if (i > 0){
        i++;
    }

    sprintf(tmp, "\r\npe %d", i); 
    consoleLog(tmp);

    HAL_FLASH_Unlock();
    unsigned char *start = (unsigned char *)&p[i];
    // if we can't fit in all our data (expanded to 2 byte align) plus the length
    if (flashlen-(i*2) < (((len+1)/2)*2+2)){
        // must erase flash page, and then write at start
        i = 0;
        start = flash_data;
        FLASH_EraseInitTypeDef eraseinfo;
        eraseinfo.NbPages = ((flashlen+2047)/2048);
        eraseinfo.PageAddress = flash_data;
        eraseinfo.TypeErase = FLASH_TYPEERASE_PAGES;

        uint32_t PageError = 0;
        HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&eraseinfo, &PageError);

        if (res != HAL_OK){
            sprintf(tmp, "\r\nerase fail %d", (int)res); 
            consoleLog(tmp);
            HAL_FLASH_Lock();
            return -1;
        }

        sprintf(tmp, "\r\n\r\n\r\n\r\n ****** ERASED FLASH at %x (%d pages)", (int)eraseinfo.PageAddress, eraseinfo.NbPages); 
        consoleLog(tmp);

    }

    // for all our longs to store
    unsigned short *src = (unsigned short *) data;
    unsigned short *dest = (unsigned short *)start;
    for (int j = 0; j < ((len+1)/2); j++){
        HAL_StatusTypeDef res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)dest, (uint32_t)*src);

        if (res != HAL_OK){
            sprintf(tmp, "\r\nwrite fail %d", (int)res); 
            consoleLog(tmp);
            HAL_FLASH_Lock();
            return -1;
        }

        dest++;
        src++;
    }

    // now finish with the len
    HAL_StatusTypeDef res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dest, len);
    if (res != HAL_OK){
        sprintf(tmp, "\r\nwrite fail %d", (int)res); 
        consoleLog(tmp);
        HAL_FLASH_Lock();
        return -1;
    }

    sprintf(tmp, "\r\nwrote flash at %x len %d", start, len); 
    consoleLog(tmp);
    HAL_FLASH_Lock();

    return len;
}


// the last data written will be read
int readFlash( unsigned char *data, int len ){
    unsigned short *p = (unsigned short *)flash_data;
    int i = (flashlen/2) - 1;
    for (; i > 0; i--){
        if (p[i] != 0xffff){
            break;
        }
    }

    if (i > 0){
        int flen = p[i];
        unsigned char *start = (unsigned char *)&p[i];
        start -= ((flen+1)/2)*2; // get to aligned start of data
        if ((uint32_t)start < (uint32_t)flash_data){
            return -1;
        } 

        if (flen > len){
            flen = len;
        }
        memcpy(data, start, flen);
        return flen;
    }
    return -1;
}

int flashposn(int *len){
    unsigned short *p = (unsigned short *)flash_data;

    int i = (flashlen/2) - 1;
    for (; i > 0; i--){
        if (p[i] != 0xffff){
            break;
        }
    }

    if (i > 0){
        int flen = p[i];
        if (len){
            *len = flen;
        }
        return i*2 - ((flen+1)/2)*2;
    }
    return -1;
}

#endif