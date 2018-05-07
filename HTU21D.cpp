// EE 4391 Senior Design
// Professor Hinkle and Dr. Compeau
// @author Team Eta
// Texas State University
 
// @section DESCRIPTION
// HTU21D Humidity and Temperature sensor.
// Datasheet, specs, and information:
// https://cdn.sparkfun.com/assets/6/a/8/e/f/525778d4757b7f50398b4567.pdf


#include "HTU21D.h"

HTU21D::HTU21D(PinName sda, PinName scl) {

    i2c_ = new I2C(sda, scl);
    //400KHz, as specified by the datasheet page 3.
    i2c_->frequency(400000);
}

int HTU21D::sample_ctemp(void) {

    char tx[1];
    char rx[2];

    tx[0] = TRIGGER_TEMP_MEASURE; // Triggers a temperature measure by feeding correct opcode.
    i2c_->write((HTU21D_I2C_ADDRESS << 1) & 0xFE, tx, 1);
    wait_ms(50); // Per datasheet, wait long enough for device to sample temperature
    
    // Reads triggered measure
    i2c_->read((HTU21D_I2C_ADDRESS << 1) | 0x01, rx, 2);
    wait_ms(1);
    
    // Algorithm from datasheet to compute temperature.
    unsigned int rawTemperature = ((unsigned int) rx[0] << 8) | (unsigned int) rx[1];
    rawTemperature &= 0xFFFC;

    float tempTemperature = rawTemperature / (float)65536; //2^16 = 65536
    float realTemperature = -46.85 + (175.72 * tempTemperature); //From page 14

    return (int)realTemperature;
}

int HTU21D::sample_ftemp(void){
    int temptemp = sample_ctemp();
    int ftemp = temptemp * 1.8 + 32; //Change this value to fit our Hexiwear or adjut temperature calibration  -- "16" used to be "32"
    
    return ftemp;
}

int HTU21D::sample_ktemp(void){
    int temptemp = sample_ctemp();
    int ktemp = temptemp + 274;
    
    return ktemp;
}

int HTU21D::sample_humid(void) {

    char tx[1];
    char rx[2];


    tx[0] = TRIGGER_HUMD_MEASURE; // Triggers a humidity measure by feeding correct opcode.
    i2c_->write((HTU21D_I2C_ADDRESS << 1) & 0xFE, tx, 1);
    wait_ms(16); // Per datasheet, wait long enough for device to sample humidity
    
    // Reads triggered measure
    i2c_->read((HTU21D_I2C_ADDRESS << 1) | 0x01, rx, 2);
    wait_ms(1);
    
    //Algorithm from datasheet.
    unsigned int rawHumidity = ((unsigned int) rx[0] << 8) | (unsigned int) rx[1];

    rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place
    
    //Given the raw humidity data, calculate the actual relative humidity
    float tempRH = rawHumidity / (float)65536; //2^16 = 65536
    float rh = 20 + (125 * tempRH); //From page 14 --- "20" used to be "-6"

    return (int)rh;
}

