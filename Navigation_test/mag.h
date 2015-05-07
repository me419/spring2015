#ifndef   MAG_H
#define   MAG_H

#include <Arduino.h>
#include <Wire.h>

#define address  0x1E //0011110b, I2C 7bit address of HMC5883

class mag {
  public:
    mag();
    void init_setup();
    void define(float xVal, float yVal, float zVal);
    void raw_values(float *x, float *y, float *z);
    void get_values(float *x, float *y, float *z);
    int get_heading(float x, float y);
  private:
    float x_scale;
    float y_scale;
    float z_scale;
};

#endif
