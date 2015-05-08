#include "mag.h"

mag::mag(){
  
  return;
}

void mag::define(float xVal, float yVal, float zVal) {
  x_scale= xVal;  // 1.0/(-384 + 448); //offset scale factor: 1.0/(max_x - min_x)
  y_scale= yVal;  // 1.0/(330 + 497);  //offset scale factor: 1.0/(max_y - min_y)
  z_scale= zVal;  // 1.0/(252 - 188);  //offset scale factor: 1.0/(max_z - min_z)
}

void mag::init_setup() {
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void mag::calibrate(int duration) {
	float xArr[duration];
	float yArr[duration];
	float zArr[duration];
	int xmin, xmax, ymax, ymin, zmin, zmax;
	// calibration loop
	for(int i = 0; i < duration; i++) {
		//read x, y, z values and add to array
		raw_values(&xArr[i], &yArr[i], &zArr[i]);
		
		//set initial values for all minimums and maximums
		if(i == 0){
			
			xmin = xArr[i];
			xmax = xArr[i];
			
			ymin = yArr[i];
			ymax = yArr[i];
			
			zmin = zArr[i];
			zmax = zArr[i]; 
		}
		
		//check to see if current datum is the minimum or maximum x-value
		if (xmax < xArr[i]){xmax = xArr[i];}
		if (xmin > xArr[i]){xmin = xArr[i];}
		
		//check to see if current datum is the minimum or maximum y-value
		if (ymax < yArr[i]){ymax = yArr[i];}
		if (ymin > yArr[i]){ymin = yArr[i];}
		
		//check to see if current datum is the minimum or maximum z-value
		if (zmax < zArr[i]){zmax = zArr[i];}
		if (zmin > zArr[i]){zmin = zArr[i];}
		delay(100); //delay between each datum
	}
	
	float xVal = 1.0/(xmax - xmin);
	float yVal = 1.0/(ymax - ymin);
	float zVal = 1.0/(zmax - zmin);
	define(xVal, yVal, zVal);
}

void mag::get_values(float *x, float *y, float *z) {
  int _x, _y, _z;
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    _x = Wire.read()<<8; //X msb
    _x |= Wire.read(); //X lsb
    _z = Wire.read()<<8; //Z msb
    _z |= Wire.read(); //Z lsb
    _y = Wire.read()<<8; //Y msb
    _y |= Wire.read(); //Y lsb
  }
    *x = _x * x_scale;
    *y = _y * y_scale;
    *z = _z * z_scale;
    return;
}

void mag::raw_values(float *x, float *y, float *z) {
  int _x, _y, _z;
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    _x = Wire.read()<<8; //X msb
    _x |= Wire.read(); //X lsb
    _z = Wire.read()<<8; //Z msb
    _z |= Wire.read(); //Z lsb
    _y = Wire.read()<<8; //Y msb
    _y |= Wire.read(); //Y lsb
  }
    *x = _x;
    *y = _y;
    *z = _z;
    return;
}
	
int mag::get_heading(float x, float y) {
  float heading=0;
  
  //defines value of heading for various x,y,z values
  if ((x == 0)&&(y < 0))
    heading= 3.1415926/2.0;
  if ((x == 0)&&(y > 0))
    heading=3.0*3.1415926/2.0; 
  if (x < 0)
    heading = 3.1415926 - atan(y/x);
  if ((x > 0)&&(y < 0))
    heading = -atan(y/x);
  if ((x > 0)&&(y > 0))
    heading = 2.0*3.1415926 - atan(y/x);
  return int(degrees(heading)); //convert heading from radians to degrees and return
}
