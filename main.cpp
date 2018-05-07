/**
 * EE 4391 Senior Design
 * Professor Hinkle and Dr. Compeau
 * @author Team Eta
 * Texas State University
 *
 * @section DESCRIPTION
 *
 **/
#include "mbed.h"
#include "HTU21D.h"
#include "Hexi_OLED_SSD1351.h" /* Header file for OLED Display */
#include "string.h"
#include "Hexi_KW40Z.h"   /* Driver for button presses and BLE */
#include <math.h>       /* Used for ADC conversion algorithm */
#include "images.h"     /* BMPs of the images drawn to the OLED */
#include "ATParser.h"
#include "BufferedSerial.h"

/*****Function Prototypes*****/
void readSensors();
void ledColor();
void led_out();
void CalculatePPM();
void dataQueue();
void initModules();
void StartHaptic(void);
void StopHaptic(void const *n);
void clearString();
void sendSMS();
//void txTask(void);

AnalogIn COSensor(PTB6);   /* Carbon Monoxide sensor is attached to docking station port 3 */
Serial pc(USBTX, USBRX); // Serial interface
HTU21D tempHumid(PTB1,PTB0); // HTU21D Sensor
DigitalOut powerEN (PTB12); // Power Enable HTU21D Sensor

//GSM Pins
BufferedSerial SMS(PTD3, PTD2);
DigitalOut gsmp(PTB11);
DigitalOut gsmcts(PTB13);
DigitalOut gsmrts(PTC4);
DigitalIn gsmst(PTB2);

/* Instantiate the SSD1351 OLED Driver */ 
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);

BusOut led(PTC8, PTD0, PTC9);   /* RGB Port Configurations */
DigitalOut haptic(PTB9);        /* Port Configuration for vibrations */

//GSM Declarations
ATParser gsm = ATParser(SMS, "\r\n");
int value;
char buffer[100];
//string result;
char x;

/* Define timer for haptic feedback */
RtosTimer hapticTimer(StopHaptic, osTimerOnce);

int screen = 1;         //detects which screen the user is currently on
bool is_drawn1 = 0;      //detects if the screen is already drawn to the screen
bool is_drawn2 = 0;
bool is_drawn3 = 0;
//bool is_drawn4 = 0;

/* Screens */
const uint8_t *image1;          //homescreen

const uint8_t *image2;          //temp/humid

const uint8_t *image3;         //co screen

const int green = 5,
          red = 6,          /* LED Colors */
          black = 7,
          yellow = 4;
          
int led_color;      /* Variable to hold the current LED color */

Ticker sensor_read,      /* Used for the read sensor subroutine call */
       led_flash,        /* Used to determine the flash rate */
       ledcolor,        /* Used to determine the LED color */
       A2D_Convert,    /* Used to convert the analog signal to digital */
       push;            /* Used to push values into the data array */
      
char text[20];  /* Static Text Buffer for OLED Display */
char CO_level[20]; /* Dynamic Text Buffer for CO total */
char average[20]; /* Dynamic Text Buffer for the Average total */
char total_total[20]; /* Dynamic Text Buffer for the Total total */

/* Either unit16_t or double */
double     total,    /* Variable to store the total total */
           co_ppm = 0,   /* Variable used to store the CO PPM */ 
           prev_value;  /* Used for ARC Calculation */
double ARC;

// Temp / Humid. Variables
int s_ftemp = 0;
int s_ctemp;
int s_ktemp;
int s_humid;
         
uint16_t co_total; /* Variable to store the CO total */  
         
/*Create a Thread to handle sending BLE Sensor Data */ 
//Thread txThread;

/* Variables to handle the circular array algorithm */
const int SIZE = 500;
double dataSet [SIZE];
int pushIndex = 0;
bool calc = false;  
double avg = 0, sum = 0;     

/* Instantiate the Hexi KW40Z Driver (UART TX, UART RX) */ 
KW40Z kw40z_device(PTE24, PTE25);

/* Below are the functions to handle button presses and screens */
void ButtonRight(void)//CO screen button
{   
    StartHaptic();
    screen = 3;
}

void ButtonLeft(void) //Temp/Humidity Screen button
{
    StartHaptic();
    screen = 2;
}

void ButtonDown(void)//home screen button
{
    StartHaptic();
    screen = 1;
}
      
int main(){
    
    initModules();
    
    /* Subroutine Calls */
    sensor_read.attach(&readSensors, 1);   /* Read the sensor on a time interval */
    A2D_Convert.attach(&CalculatePPM, 2);  /* Convert the values read from the sensors to floating point ppm values */
    push.attach(&dataQueue, 1);            /* Push the value into the set and compute the average */
    ledcolor.attach(&ledColor, 0.5);       /* Determine the LED color */
    led_flash.attach(&led_out, 0.25);      /* Flash LED based on sensor data */
    
    /* Register callbacks to button presses */
    kw40z_device.attach_buttonDown(&ButtonDown);
    kw40z_device.attach_buttonLeft(&ButtonLeft);
    kw40z_device.attach_buttonRight(&ButtonRight);
    
    //txThread.start(txTask); /*Start transmitting Sensor Tag Data */
    
    while (1) { /* Loop to process and display data to the OLED */
    
        /* Get OLED Class Default Text Properties */
        oled_text_properties_t textProperties = {0};
        oled.GetTextProperties(&textProperties);
        
        /* Set text properties to green and right aligned for the dynamic text */
        textProperties.fontColor = COLOR_GREEN;
        textProperties.alignParam = OLED_TEXT_ALIGN_RIGHT;
        oled.SetTextProperties(&textProperties);
              
        if (screen == 3){  //Carbon Monoxide Screen
            if (!is_drawn3){
                is_drawn2 = 0;
                is_drawn1 = 0;
                //is_drawn4 = 0;
                oled.DrawImage(image3,0,0);
                is_drawn3 = 1;
            }
            if (co_ppm >= 50 and co_ppm < 100){
                textProperties.fontColor = COLOR_YELLOW;  //color the font yellow
                oled.SetTextProperties(&textProperties);
            }
            
            else if (co_ppm >= 100) {
                textProperties.fontColor = COLOR_RED;  //color the font red
                oled.SetTextProperties(&textProperties);
            }
            sprintf(CO_level,"%.2f",co_ppm);    /* Print the CO PPM to the screen */        
            oled.TextBox((uint8_t *)CO_level,35,76,35,15);
            
        }
        else if (screen == 2) {   //Temp Humid
            if (!is_drawn2){
                is_drawn1 = 0;
                is_drawn3 = 0;
                //is_drawn4 = 0;
                oled.DrawImage(image2,0,0);
                is_drawn2 = 1;
            }
            printf("Temperature: %d F\n\r", s_ftemp);
            printf("Humidity: %d %%\n\r", s_humid);
            printf("\n\r");

            /* Display Legends */
            strcpy((char *) text,"Temp.");
            oled.Label((uint8_t *)text,5,67);      
      
            /* Format the value */
            sprintf(text,"%i",s_ftemp);
            /* Display time reading in 35px by 15px textbox at(x=55, y=40) */
            oled.TextBox((uint8_t *)text,57,67,20,15); //Increase textbox for more digits
        
            /* Display Units */
            strcpy((char *) text,"dF");
            oled.Label((uint8_t *)text,80,67);     
      
            /* Set text properties to white and right aligned for the dynamic text */ 
            textProperties.fontColor = COLOR_BLUE;
            textProperties.alignParam = OLED_TEXT_ALIGN_RIGHT;
            oled.SetTextProperties(&textProperties);  
      
            /* Display Legends */
            strcpy((char *) text,"Humidity");
            oled.Label((uint8_t *)text,5,81);       
      
            /* Format the value */
            sprintf(text,"%i",s_humid);
            /* Display time reading in 35px by 15px textbox at(x=55, y=40) */
            oled.TextBox((uint8_t *)text,57,81,20,15); //Increase textbox for more digits
        
            /* Display Units */
            strcpy((char *) text,"%");
            oled.Label((uint8_t *)text,80,81); 
            
               
        }
        else if (screen == 1) {   //Home Screen
            if (!is_drawn1){
                is_drawn3 = 0;
                is_drawn2 = 0;
                //is_drawn4 = 0;
                oled.DrawImage(image1,0,0);
                is_drawn1 = 1;
            }        
        }
        if (s_ftemp > 80 || co_ppm > 35){
          sendSMS();  // SEND SMS
          wait_ms(500);
        }
        Thread::wait(500); 
    }
}
/*
void txTask(void){
    while (1){
        //Notify Hexiwear App that it is running Sensor Tag mode
        kw40z_device.SendSetApplicationMode(GUI_CURRENT_APP_SENSOR_TAG);   
        kw40z_device.SendHumidity(100*co_ppm);  //send ppm CO click value
        kw40z_device.SendPressure(100*total);     //send total
        Thread::wait(1000);
    }
}*/
void initModules() {    /* Function to initialize the system */

    /* Turns on the OLED Display*/
    oled.PowerON();
    //Turn on Temp/Humid Sensor
    powerEN = 0;
    
    //Initialize GSM
    gsmp = !gsmp;
    gsmcts = !gsmcts;
    gsmrts = !gsmrts;
    
    /* Sets the pointers to their respective images */
    image1 = homescreen_bmp;
    image2 = temphumid_bmp;
    image3  = co_bmp;
}

void readSensors(){  /* Function to read sensors */
    
    /* Grab the analog signal from the sensors as 16 bit unsigned integers */
    co_total = COSensor.read_u16();
    s_ftemp = tempHumid.sample_ftemp();
    s_ctemp = tempHumid.sample_ctemp();
    s_ktemp = tempHumid.sample_ktemp();
    s_humid = tempHumid.sample_humid();
    
    /* Grab the analog signal from the sensors as floats */
  
}

void ledColor(){    /* Function to determine the LED color */
    
    if ((total - avg) <= 10 and total < 50 and ARC < 10 ) {
        led = green;
        led_color = led;     /* Store the LED color for the flash function */
    }
    else if ((total - avg) > 10 and (total - avg) < 50 and total < 100 or (ARC >= 10 and ARC < 20)) {
        led = yellow;
        led_color = led;     /* Store the LED color for the flash function */
    }
    else if ((total - avg >= 50) or total >= 100 or avg >= 100 or ARC >= 20) {
        led = red;
        led_color = led;     /* Store the LED color for the flash function */
    }
}

void led_out() { /* Function to flash LEDs */

    if (led == green) {return;}   /* If green, don't blink */
    else if (led == black) {led = led_color;}
    else {led = black;}    
}

void CalculatePPM(){ /* Function to convert the analog signals to digital based on 16 bit adc steps */
    
    
    const double CO_Rl = 18500.0;               // CO_Rl (18.5kOhm) - Load resistance for CO sensor
    const double Vadc_33 = 0.0000503548;         // ADC step 3,3V/65535 0.00503mV (16bit ADC)
    //const double Vadc_5 = 5.0/65535;          // ADC step 5.0V/2^16 (65535, 16bit ADC)
    double Vrl;                                  // Output voltage
    double Rs;                                   // Rs (Ohm) - Sensor resistance
    double ratio;                                // Rs/Rl ratio
    double lgPPM;

    
    if (co_total > 65533)   //prevents NAN error from overflow of 16 bits
        co_total = 65530;
    Vrl = (double)co_total * Vadc_33;            // For 3.3V Vcc use Vadc_33
    Rs = CO_Rl * (3.3 - Vrl)/Vrl;                 // Calculate sensor resistance
    ratio = Rs/CO_Rl;                             // Calculate ratio
    lgPPM = (log10(ratio) * -0.8) + 0.9;       
    co_ppm = 6* pow(10,lgPPM) - 12;                 // Calculate carbon monoxide ppm
    
    
    if (co_ppm <0)
        co_ppm = 0;
    else if (co_ppm >= 1000)
        co_ppm = 999.99;
      
    /* Calculate the total */
    total = co_ppm ;    
    if (total < 0)
        total = 0;
    else if (total >= 1000)
        total = 999.99;
}

void dataQueue()
{   /* Beginning of function */
    if (pushIndex != SIZE and !calc){    /* Initially pushing values into the set */
        dataSet[pushIndex] = total;          //push value into the queue
        sum += dataSet[pushIndex];  //add the value to the sum
        pushIndex++;                    //increment the push index
        
        if (pushIndex == SIZE){
            avg = sum / SIZE;       //compute the average once the queue is full
            calc = true;                //flag calc
            ARC = dataSet[SIZE - 1] - dataSet[SIZE - 2];
            pushIndex = 0;              //reset the push index back to 0
        }
    }
    
    else if (pushIndex != SIZE and calc){   /* Pushing values into the set once the set is full */
        sum -= dataSet[pushIndex];          //subtract the value to be overriden from the sum
        dataSet[pushIndex] = total;           //push the value into the set
        sum += dataSet[pushIndex];          //add the value to the sum
        avg = sum / SIZE;                   //compute average
        if (pushIndex == 0){
            prev_value = dataSet[SIZE - 1];
            ARC = dataSet[pushIndex] - prev_value;
        }
        else {
            prev_value = dataSet[pushIndex - 1];
            ARC = dataSet[pushIndex] - prev_value;
        }
        pushIndex++;                        //increment the push index
        
        if (pushIndex == SIZE)
            pushIndex = 0;    //reset the push index back to 0
    }

} /* End of function */

void StartHaptic(void)
{
    hapticTimer.start(50);
    haptic = 1;
}

void StopHaptic(void const *n) {
    haptic = 0;
    hapticTimer.stop();
}

void sendSMS()
{
    gsm.printf("AT+CMGF=1\r"); //at command for send sms  
    wait_ms(1000);
    gsm.printf("AT+CMGS=");
    gsm.putc('"');
    gsm.printf("+12142988594");
    gsm.putc('"');
    gsm.printf("\r");
    wait_ms(1000);
    gsm.printf("Caution, Threshold has been met! ");
    wait_ms(1000);
    gsm.putc(0x1A);
}
