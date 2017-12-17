#include "USRled.h"

int detection_led(const char *LEDBrightness){
	printf("In the value in led \n");
	FILE* LEDHandle = NULL;
	//const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr1/brightness";
int i =0;
if((LEDHandle = fopen(LEDBrightness,"r+")) !=NULL){
		fwrite("1",sizeof(char),1,LEDHandle);
		fclose(LEDHandle);
	}
	else{
		printf("Cant OPen LED\n");
}
for(i=0;i<100;i++){ 
	
	usleep(2000);	
}
	
	if((LEDHandle = fopen(LEDBrightness,"r+")) != NULL){
		fwrite("0",sizeof(char),1,LEDHandle);
		fclose(LEDHandle);
		}
	return 0;
}
