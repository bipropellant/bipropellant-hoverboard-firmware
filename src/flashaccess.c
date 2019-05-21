#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "comms.h"
#include "flashaccess.h"
#include <string.h>

#ifdef FLASH_STORAGE


// pages are 2k=0x400 bytes

// from flasharea.c, which reserves a page of flash for our use.
//extern int flashlen;
//extern volatile const uint8_t flash_data[];

// last two pages of the first half
volatile uint8_t *flash_data = (uint8_t *) 0x08000000 + 0x40000 - 0x800;
int flashlen = 4096; // two pages

// routines to read and write config data
// we will write at incrementing locations through the flash until we exhaust it, then erase and start over.


unsigned short writeFlash16( volatile unsigned short *data, uint16_t len ){
    int outlen = writeFlash( (unsigned char *)data, len*2 );
    if (outlen >= 0){
        outlen /=2;
    }
    return outlen;
}

// data will be written 32 bit aligned, with the last long being a length.
// everything AFTER will still be 0xFFFFFFFF
int writeFlash( unsigned char *data, int len ){
    char tmp[80];

    sprintf(tmp, "\r\nflash root %lx", (unsigned long)flash_data);
    consoleLog(tmp);

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
        start = (unsigned char *)flash_data;
        FLASH_EraseInitTypeDef eraseinfo;
        eraseinfo.NbPages = ((flashlen+2047)/2048);
        eraseinfo.PageAddress = (unsigned long)flash_data;
        eraseinfo.TypeErase = FLASH_TYPEERASE_PAGES;

        uint32_t PageError = 0;
        HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&eraseinfo, &PageError);

        if (res != HAL_OK){
            sprintf(tmp, "\r\nerase fail %d", (int)res);
            consoleLog(tmp);
            HAL_FLASH_Lock();
            return -1;
        }

        sprintf(tmp, "\r\n\r\n\r\n\r\n ****** ERASED FLASH at %lx (%ld pages)", eraseinfo.PageAddress, eraseinfo.NbPages);
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
    HAL_StatusTypeDef res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (unsigned long)dest, len);
    if (res != HAL_OK){
        sprintf(tmp, "\r\nwrite fail %d", (int)res);
        consoleLog(tmp);
        HAL_FLASH_Lock();
        return -1;
    }

    sprintf(tmp, "\r\nwrote flash at %lx len %d", (unsigned long)start, len);
    consoleLog(tmp);
    HAL_FLASH_Lock();

    return len;
}


unsigned short readFlash16( volatile unsigned short *data, uint16_t len ){
    int outlen = readFlash( (unsigned char *)data, len*2 );
    if (outlen >= 0){
        outlen /=2;
    }
    return outlen;
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
            char tmp[80];
            sprintf(tmp, "\r\nflashread - len %d seems wrong?", i);
            consoleLog(tmp);
            return -1;
        }

        if (flen > len){
            flen = len;
        }
        memcpy(data, start, flen);
        return flen;
    }
    char tmp[80];
    sprintf(tmp, "\r\nflashread - no end found?");
    consoleLog(tmp);
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


///////////////////////////////////////////////////
// generic flash routines for firmware upgrade use
// we have 256K, so pages 0-127, all 2k (0x800) long
// note: this won't write across pages.
// design expected len value 128 or 256.
// UNTESTED
int writeflashchunk( void *addr, unsigned char *data, int len ){
    char tmp[40];
    // don't allow write in areas < page 64
    if ((unsigned long)addr < (0x8000000 + 64*0x800))
        return -1;

    HAL_FLASH_Unlock();
    // if at the start of a page, then erase....
    if (!(((unsigned long)addr) % 0x800)){
        //unsigned long start = addr;
        FLASH_EraseInitTypeDef eraseinfo;
        // only ever 1 page here
        eraseinfo.NbPages = 1;
        eraseinfo.PageAddress = (unsigned long)addr;
        eraseinfo.TypeErase = FLASH_TYPEERASE_PAGES;

        uint32_t PageError = 0;
        HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&eraseinfo, &PageError);

        if (res != HAL_OK){
            sprintf(tmp, "\r\nerase fail %d", (int)res);
            consoleLog(tmp);
            HAL_FLASH_Lock();
            return -1;
        }
    }

    // write the data
    // for all our longs to store
    unsigned short *src = (unsigned short *) data;
    unsigned short *dest = (unsigned short *)addr;
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

    return len;
}


#endif
