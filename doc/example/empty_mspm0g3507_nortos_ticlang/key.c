#include "board.h"
#include "stdio.h"
#include "ti/driverlib/m0p/dl_core.h"
int key1,key2;

int key_scan()
{
    if (DL_GPIO_readPins(KEY_PORT,KEY_key1_PIN)) 
    {
    delay_ms(50);
     key1=1;
    }
    else {
    key1=0;
    }
    if (DL_GPIO_readPins(KEY_PORT,KEY_key2_PIN)) 
    {
    delay_ms(50);
     key2=1;
    }
    else {
    key2=0;
    }
    if (key1==0 && key2==0) 
    {
    return 1;
    }
    if (key1==1 && key2==0) 
    {
    return 2;
    }
    if (key1==0 && key2==1) 
    {
    return 3;
    }
    if (key1==1 && key2==1) 
    {
    return 4;
    }
    else {
    return 0;
    }
}
    