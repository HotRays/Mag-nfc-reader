#include "hextochar.h"

uint8_t uid_dst[9] = {0};
uint8_t version_dst[2] = {0};

uint8_t * hex_to_char_uid(uint8_t * temp, uint16_t len)
{
	uint8_t str[9] = {0};
	uint8_t i = 0;
	for(i = 0; i<len;i++)
	{
			str[2*i] = temp[i]>>4;
			str[2*i+1] = temp[i]&0xf;
	}
			for(i = 0; i<(len*2);i++)
	{
			uid_dst[i] = HexToChar(str[i]);

	}	
	return uid_dst;  //dst中为字符串
}

uint8_t * hex_to_char_version(uint8_t * temp, uint16_t len)
{
	uint8_t str[2] = {0};
	uint8_t i = 0;
	for(i = 0; i<len;i++)
	{
			str[2*i] = temp[i]>>4;
			str[2*i+1] = temp[i]&0xf;
	}
			for(i = 0; i<(len*2);i++)
	{
			version_dst[i] = HexToChar(str[i]);

	}	
	return version_dst;  //dst中为字符串
}

uint8_t HexToChar(uint8_t temp)
{
    uint8_t dst;
    if (temp < 10){
        dst = temp + '0';
    }else{
        dst = temp -10 +'A';
    }
    return dst;
}

