#ifndef   MAG_H
#define   MAG_H

#include <Arduino.h>

class mag{
  public:
    mag();
	get_heading();
    void read(int *x, int *y, int *z);
  private:
	float x_scale;
	float y_scale;
	float z_scale;
#endif
