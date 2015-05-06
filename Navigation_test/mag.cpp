//#include "fluoro.h"
#include "HMC5883"

mag::mag(){
	float x_scale= 1.0/(-384 + 448); //offset scale factor: 1.0/(max_x - min_x)
	float y_scale= 1.0/(330 + 497);    //offset scale factor: 1.0/(max_y - min_y)
	float z_scale= 1.0/(252 - 188);
	return;
}
//fluoro::fluoro(){
  //return;
//}

void mag::read(int *x, int *y, int *z) {
	//Tell the HMC5883 where to begin reading data
	Wire.beginTransmission(address);
	Wire.write(0x03); //select register 3, X MSB register
	Wire.endTransmission();
	
	
	//Read data from each axis, 2 registers per axis
	Wire.requestFrom(address, 6);
	if(6<=Wire.available()){
		*x = Wire.read()<<8; //X msb
		*x |= Wire.read(); //X lsb
		*z = Wire.read()<<8; //Z msb
		*z |= Wire.read(); //Z lsb
		*y = Wire.read()<<8; //Y msb
		*y |= Wire.read(); //Y lsb
	}
	
int mag::get_heading(int x, int y) {
	float heading=0;  
		
	//defines value of heading for various x,y,z values
	if ((x == 0)&&(y < 0))  
		heading= PI/2.0; 
	
	if ((x == 0)&&(y > 0))  
		heading=3.0*PI/2.0; 
		
	if (x < 0)  
		heading = PI - atan(y/x);  
	
	if ((x > 0)&&(y < 0))  
		heading = -atan(y/x);
	
	if ((x > 0)&&(y > 0))  
		heading = 2.0*PI - atan(y/x); 
	
	return  int(degrees(heading));  //convert heading from radians to degrees and return
}