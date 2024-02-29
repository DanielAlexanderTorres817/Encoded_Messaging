/*
* This code simulates the game called "Pass it on!" using Adafruit Classic Playground boards.
* In this "game" we try to transmit a certain message between the team members, and if successfull,
* we should get the original message from the last team member.
*

*/


#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>


// Colors for Neofixels
#define white 0xFFFFFF
#define red 0xFF0000
#define green 0x00FF00
#define blue 0x0000FF
#define yellow  0xFFFF00
#define purple 0xFF00FF
#define cyan 0x00FFFF
#define orange 0xFF6400
#define pink 0xC83333
#define turquoise 0x1AFF33


// Number of readings (X, Y, Z) accelerometer does in one session
const int COUNTER = 40;


// Current readings of accelerometer
float X, Y, Z;


// Recording state for accelerometer. True if in the recording state. Otherwise false
bool record = false;


// True if gesture is detected by the algorithm. False if not
bool detected = false;


// Current number of readings done by the board
int dynamic_counter = 0;


// The maximum error the comparison between gestures can reach
float error_threshold = 5;


// The current gesture readings
float current_gesture[COUNTER][3] = {0.0};


// Array of colors for each neopixels
uint32_t color_array[10] = {white, red, green, blue, yellow, purple, cyan, orange, pink, turquoise};


// 10 pre-calculated gestures recorded by the same board. These are the gestures that
// we are going to compare with the current_gesture.
// All of them are stored in the flash of the borad.
// Since the float type is memory heavy, we had to store them in the flash.
const float gestures[10][COUNTER][3] PROGMEM = {
  
{{2.16, -0.71, 10.52}, {1.90, -0.54, 10.21}, {1.55, -0.39, 9.75}, {1.53, -0.30, 9.75}, {1.57, -0.30, 9.88}, {1.51, -0.25, 9.88}, {1.61, -0.38, 9.83}, {1.59, -0.57, 9.77}, {1.54, -0.72, 9.70}, {1.47, -0.79, 9.67}, {1.49, -0.82, 9.64}, {1.53, -0.89, 9.76}, {1.53, -0.88, 9.90}, {1.57, -1.03, 9.81}, {1.49, -0.95, 9.67}, {1.54, -0.81, 9.55}, {1.45, -0.40, 9.59}, {1.49, -0.16, 9.63}, {1.57, -0.45, 9.51}, {1.69, -0.58, 9.72}, {1.85, -0.73, 9.82}, {1.99, -0.52, 10.11}, {2.13, -0.59, 10.11}, {2.21, -0.82, 10.11}, {2.21, -0.92, 9.96}, {2.18, -1.06, 9.76}, {2.09, -0.94, 9.62}, {1.94, -0.93, 9.42}, {1.98, -0.74, 9.42}, {2.02, -0.67, 9.47}, {2.02, -0.47, 9.62}, {1.97, -0.42, 9.59}, {1.98, -0.33, 9.78}, {2.07, -0.67, 10.07}, {1.96, -0.81, 10.17}, {1.84, -1.02, 10.01}, {1.61, -0.80, 9.78}, {1.44, -0.60, 9.84}, {1.19, -0.45, 9.70}, {1.08, -0.36, 9.65}},


{{1.38, -0.25, 7.75}, {1.30, 0.03, 8.52}, {1.32, 0.14, 9.91}, {1.30, 0.10, 10.05}, {1.46, -0.04, 9.89}, {1.52, -0.03, 9.91}, {1.42, -0.02, 9.77}, {1.23, -0.02, 9.84}, {1.23, -0.20, 9.86}, {1.20, -0.31, 9.83}, {1.27, -0.49, 9.75}, {1.12, -0.44, 9.60}, {1.12, -0.41, 9.70}, {1.06, -0.45, 9.61}, {1.06, -0.28, 9.69}, {1.06, -0.02, 9.59}, {1.03, 0.16, 9.63}, {1.10, 0.17, 9.60}, {1.25, -0.21, 9.56}, {1.32, -0.19, 9.75}, {1.29, -0.13, 9.77}, {1.27, -0.03, 9.83}, {1.30, -0.18, 9.69}, {1.51, -0.24, 9.77}, {1.68, -0.25, 9.71}, {1.87, -0.29, 9.92}, {1.97, -0.52, 10.03}, {1.94, -0.47, 10.19}, {2.02, -0.37, 10.00}, {1.99, -0.07, 10.08}, {1.99, -0.44, 9.89}, {1.98, -0.71, 10.00}, {2.02, -1.19, 9.63}, {1.92, -0.87, 9.55}, {1.80, -0.82, 9.43}, {1.79, -0.74, 9.38}, {1.76, -0.56, 9.71}, {1.72, -0.19, 9.83}, {1.62, 0.09, 10.16}, {1.64, 0.07, 10.09}},


{{1.58, -0.19, 6.10}, {1.43, -0.17, 7.22}, {1.20, -0.27, 9.62}, {1.29, -0.36, 9.84}, {1.27, -0.60, 10.10}, {1.15, -0.73, 10.24}, {1.05, -0.89, 10.02}, {1.00, -0.92, 10.00}, {1.02, -1.08, 10.07}, {0.99, -1.23, 10.13}, {0.98, -1.37, 9.96}, {0.93, -1.46, 9.79}, {0.88, -1.47, 9.71}, {0.74, -1.40, 9.69}, {0.58, -1.25, 9.73}, {0.59, -1.29, 9.67}, {0.53, -1.17, 9.71}, {0.50, -1.09, 9.59}, {0.26, -0.77, 9.51}, {0.30, -0.68, 9.47}, {0.38, -0.43, 9.61}, {0.61, -0.42, 9.75}, {0.72, -0.19, 9.88}, {0.83, -0.16, 9.94}, {0.85, -0.12, 9.87}, {1.01, -0.35, 9.86}, {1.15, -0.48, 10.09}, {1.30, -0.54, 10.28}, {1.29, -0.58, 10.37}, {1.19, -0.69, 10.10}, {1.25, -0.94, 9.95}, {1.24, -1.18, 9.63}, {1.41, -1.48, 9.19}, {1.37, -1.45, 9.40}, {1.59, -1.40, 9.61}, {1.71, -1.02, 10.12}, {1.97, -0.51, 10.09}, {1.91, 0.08, 10.14}, {1.77, 0.20, 10.23}, {1.60, 0.16, 10.22}},


{{1.44, -0.62, 11.21}, {1.37, -0.61, 10.81}, {1.21, -0.56, 10.11}, {1.15, -0.69, 10.07}, {1.10, -0.73, 10.08}, {1.11, -0.74, 10.03}, {1.01, -0.61, 9.91}, {0.97, -0.62, 9.82}, {0.80, -0.61, 9.76}, {0.70, -0.57, 9.81}, {0.57, -0.48, 9.73}, {0.52, -0.28, 9.59}, {0.57, -0.20, 9.60}, {0.66, -0.18, 9.56}, {0.66, -0.14, 9.68}, {0.71, -0.02, 9.62}, {0.76, 0.25, 9.83}, {1.02, 0.19, 9.94}, {1.11, 0.11, 10.05}, {1.08, 0.13, 9.98}, {1.08, 0.13, 9.93}, {1.24, -0.09, 9.93}, {1.46, -0.43, 9.93}, {1.51, -0.67, 9.90}, {1.49, -0.64, 9.77}, {1.58, -0.62, 9.74}, {1.66, -0.65, 9.85}, {1.72, -0.71, 9.85}, {1.76, -0.71, 9.92}, {1.73, -0.57, 9.79}, {1.76, -0.34, 9.80}, {1.77, -0.20, 9.68}, {1.82, -0.02, 9.60}, {1.89, 0.02, 9.67}, {1.90, 0.01, 9.69}, {1.86, -0.05, 9.65}, {1.68, -0.06, 9.71}, {1.52, -0.28, 9.98}, {1.28, -0.48, 10.11}, {1.20, -0.71, 10.16}},


{{2.05, 0.71, -10.64}, {2.10, 0.81, -10.22}, {2.09, 0.87, -9.51}, {2.02, 0.80, -9.40}, {2.00, 0.78, -9.27}, {2.04, 0.85, -9.44}, {2.08, 1.01, -9.43}, {2.16, 1.24, -9.54}, {2.25, 1.21, -9.48}, {2.45, 1.25, -9.52}, {2.51, 1.19, -9.54}, {2.61, 1.33, -9.57}, {2.51, 1.23, -9.56}, {2.52, 1.00, -9.83}, {2.49, 0.88, -9.86}, {2.47, 0.89, -9.86}, {2.37, 0.88, -9.68}, {2.16, 0.65, -9.58}, {2.00, 0.40, -9.56}, {1.93, 0.27, -9.43}, {1.98, 0.20, -9.13}, {2.07, 0.30, -9.12}, {2.02, 0.29, -9.32}, {1.96, 0.34, -9.57}, {2.00, 0.27, -9.57}, {2.10, 0.18, -9.50}, {2.20, 0.11, -9.48}, {2.24, 0.12, -9.53}, {2.34, 0.23, -9.67}, {2.51, 0.23, -9.86}, {2.62, 0.18, -9.97}, {2.65, 0.21, -9.96}, {2.56, 0.37, -9.79}, {2.49, 0.61, -9.71}, {2.42, 0.85, -9.57}, {2.39, 1.06, -9.44}, {2.25, 0.98, -8.99}, {2.09, 0.62, -8.97}, {2.06, 0.37, -9.25}, {2.07, 0.27, -9.69}},


{{1.12, -0.87, -11.85}, {1.20, -0.55, -11.10}, {1.23, 0.08, -9.52}, {1.21, 0.27, -9.49}, {1.09, 0.17, -9.52}, {1.15, 0.17, -9.51}, {1.21, -0.01, -9.60}, {1.41, 0.14, -9.69}, {1.52, 0.14, -9.75}, {1.62, 0.18, -9.75}, {1.59, 0.08, -9.61}, {1.61, 0.04, -9.64}, {1.59, -0.08, -9.58}, {1.69, 0.04, -9.52}, {1.71, 0.10, -9.40}, {1.90, 0.37, -9.43}, {1.91, 0.40, -9.50}, {1.99, 0.58, -9.56}, {2.10, 0.85, -9.88}, {2.13, 0.99, -10.20}, {2.05, 0.98, -10.34}, {1.76, 0.73, -10.06}, {1.48, 0.60, -9.66}, {1.28, 0.56, -9.44}, {1.10, 0.41, -9.50}, {0.99, 0.42, -9.40}, {0.96, 0.63, -9.10}, {0.95, 0.88, -8.92}, {1.01, 1.04, -9.06}, {1.06, 0.94, -9.62}, {1.18, 0.83, -9.73}, {1.36, 0.63, -9.86}, {1.57, 0.70, -9.79}, {1.74, 0.63, -9.82}, {1.91, 0.79, -9.68}, {1.97, 0.54, -9.69}, {2.09, 0.39, -9.66}, {2.13, 0.26, -9.66}, {2.20, 0.31, -9.55}, {2.21, 0.44, -9.46}},


{{1.81, -0.31, -8.93}, {1.64, -0.17, -8.89}, {1.34, 0.01, -9.41}, {1.29, 0.04, -9.61}, {1.25, -0.02, -9.96}, {1.28, -0.04, -9.96}, {1.18, -0.04, -9.70}, {1.04, -0.06, -9.64}, {0.85, -0.02, -9.53}, {0.86, 0.15, -9.57}, {0.82, 0.19, -9.47}, {0.80, 0.02, -9.52}, {0.74, 0.04, -9.67}, {0.81, 0.13, -9.69}, {0.87, 0.42, -9.57}, {0.95, 0.33, -9.42}, {1.00, 0.26, -9.51}, {1.10, 0.23, -9.67}, {1.21, 0.17, -9.71}, {1.38, 0.26, -9.78}, {1.50, 0.22, -9.80}, {1.52, 0.31, -10.00}, {1.56, 0.35, -9.92}, {1.54, 0.24, -9.84}, {1.59, 0.10, -9.67}, {1.59, 0.12, -9.63}, {1.55, 0.17, -9.51}, {1.57, 0.33, -9.43}, {1.57, 0.39, -9.47}, {1.57, 0.57, -9.64}, {1.54, 0.66, -9.64}, {1.56, 0.64, -9.68}, {1.58, 0.64, -9.64}, {1.58, 0.69, -9.75}, {1.48, 0.69, -9.67}, {1.45, 0.63, -9.71}, {1.47, 0.84, -9.58}, {1.44, 0.98, -9.65}, {1.40, 1.01, -9.54}, {1.33, 0.80, -9.64}},


{{2.39, 0.20, -6.88}, {2.36, 0.41, -7.95}, {2.38, 0.96, -9.66}, {2.36, 0.76, -9.63}, {2.38, 0.70, -9.49}, {2.31, 0.58, -9.62}, {2.26, 0.65, -9.67}, {2.17, 0.55, -9.64}, {2.25, 0.68, -9.69}, {2.23, 0.66, -9.63}, {2.31, 0.98, -9.67}, {2.22, 1.08, -9.58}, {2.31, 1.57, -9.37}, {2.38, 2.00, -9.13}, {2.45, 2.32, -9.17}, {2.47, 2.36, -9.40}, {2.41, 2.16, -9.59}, {2.43, 2.20, -9.63}, {2.31, 2.04, -9.57}, {2.10, 1.83, -9.53}, {1.84, 1.33, -9.51}, {1.80, 0.92, -9.63}, {1.77, 0.36, -9.44}, {1.71, 0.10, -9.44}, {1.48, -0.20, -9.22}, {1.42, -0.25, -9.48}, {1.40, -0.35, -9.49}, {1.44, -0.10, -9.67}, {1.40, 0.32, -9.51}, {1.43, 0.64, -9.40}, {1.53, 1.03, -9.37}, {1.66, 1.18, -9.60}, {1.80, 1.63, -9.68}, {1.82, 1.81, -9.52}, {1.78, 1.93, -9.34}, {1.70, 1.68, -9.37}, {1.78, 1.49, -9.37}, {1.88, 1.56, -9.43}, {1.84, 1.58, -9.48}, {1.79, 1.63, -9.67}},


{{1.25, 0.88, -9.57}, {1.00, 1.10, -9.74}, {0.54, 1.36, -9.92}, {0.48, 1.52, -9.81}, {0.48, 1.46, -9.70}, {0.44, 1.29, -9.66}, {0.39, 1.22, -9.40}, {0.39, 1.14, -9.37}, {0.34, 0.94, -9.34}, {0.46, 0.57, -9.50}, {0.56, 0.32, -9.54}, {0.77, 0.29, -9.63}, {1.04, 0.38, -9.88}, {1.06, 0.33, -9.74}, {1.16, 0.30, -9.96}, {1.16, 0.26, -9.79}, {1.39, 0.34, -9.92}, {1.54, 0.53, -9.84}, {1.47, 0.57, -10.04}, {1.42, 0.77, -10.13}, {1.23, 0.84, -10.05}, {1.24, 1.22, -9.76}, {1.28, 1.56, -9.64}, {1.20, 1.73, -9.40}, {1.12, 1.57, -9.29}, {0.86, 1.18, -9.21}, {0.80, 0.97, -9.31}, {0.79, 0.87, -9.30}, {0.85, 0.65, -9.31}, {0.98, 0.43, -9.24}, {1.20, 0.28, -9.56}, {1.43, 0.28, -9.74}, {1.66, 0.27, -10.02}, {1.87, 0.34, -9.77}, {2.21, 0.32, -10.02}, {2.45, 0.42, -9.83}, {2.66, 0.54, -10.12}, {2.73, 0.79, -9.84}, {2.76, 0.88, -9.89}, {2.71, 0.83, -9.75}},


{{2.22, -2.30, -11.47}, {2.27, -1.55, -10.70}, {2.26, -0.15, -9.43}, {2.47, 0.18, -9.41}, {2.41, 0.11, -9.60}, {2.30, -0.12, -9.63}, {2.27, -0.20, -9.68}, {2.21, -0.24, -9.66}, {2.09, -0.33, -9.71}, {1.90, -0.47, -9.75}, {1.79, -0.41, -9.64}, {1.70, -0.31, -9.61}, {1.57, -0.30, -9.58}, {1.46, -0.30, -9.67}, {1.28, -0.36, -9.62}, {1.21, -0.37, -9.64}, {1.14, -0.39, -9.54}, {1.08, -0.44, -9.36}, {1.07, -0.28, -9.26}, {1.10, -0.34, -9.26}, {1.07, -0.19, -9.29}, {1.19, -0.23, -9.34}, {1.20, -0.34, -9.51}, {1.27, -0.70, -9.75}, {1.31, -1.03, -9.96}, {1.27, -1.42, -10.12}, {1.36, -1.36, -10.22}, {1.16, -1.36, -10.12}, {1.23, -1.06, -10.11}, {1.18, -0.53, -9.62}, {1.26, -0.25, -9.22}, {0.95, -0.18, -8.56}, {0.94, -0.74, -8.81}, {0.94, -0.97, -9.16}, {1.25, -0.83, -9.67}, {1.40, -0.39, -9.98}, {1.49, -0.11, -10.19}, {1.51, -0.17, -10.13}, {1.50, -0.44, -9.90}, {1.52, -0.57, -9.69}}


};




/**
* Turns on the corresponding neopixel with the given color
*
* This function takes in two values and lights up the
* neopixel corresponding to those values. Turns off the pixels
* when the delay is done.
*
* @param pixel specific pixel to light up (0 - 9)
* @param color color of the pixel
*/
void light_LED(int pixel, uint32_t color){
 CircuitPlayground.setPixelColor(pixel, color);


 // wait five seconds
 delay(5000);


 // remove all pixels
 CircuitPlayground.clearPixels();
}


/**
* FOR DEBUGGING AND RECORDING GESTURES ONLY
*
* Takes in a single gesture and prints it out.
*
* @param gesture gesture that needs to be print out
*/
void print_gesture(float gesture[COUNTER][3]) {
   Serial.print("{");
   for (int i = 0; i < COUNTER; i++) {
       Serial.print("{");
       for (int j = 0; j < 3; j++) {
           Serial.print(gesture[i][j]);
           if (j < 2) {
               Serial.print(", ");
           }
       }
       Serial.print("}");
       if (i < COUNTER - 1) {
           Serial.print(", ");
       }
   }
   Serial.println("}");
}


/**
* Gets the gesture value of the single gesture in the list of gestures.
*
* Takes in three values and returnes the corresponding value from the gesture.
* Since we sotore the values of the gestures in the flash, we use pgm_read_float_near
* to access it.
*
* @param gesture_index number of the gesture to access
* @param i reading inside a single gesture
* @param j value of the reading (X, Y, and Z)
* @return single reading (X, Y, and Z)
*/
float get_gesture_value(int gesture_index, int i, int j) {
   return pgm_read_float_near(&gestures[gesture_index][i][j]);
}




/**
* Compares the current_gesture with all the 10 gestures and returns the
* index of the gesture with the least error. If error excedes the error_threshold
* it throws an error by returning -1.
*
* It uses Euclidean distance to calculate "error" between two points in 3 dimensions
* Formula: d = sqrt((x1 - x1')^2 + (y1 - y1')^2 + (z1 - z1')^2) + sqrt((x2 - x2')^2 +...)
* The error in this case is not in percentage, but in units of distance.
*
* @param current Current gesutre that we are comparing
* @return if the gesture deteceted, it will return number from 0 to 9. Otherwise -1.
*/
int find_closest_gesture(float current[COUNTER][3]) {
   // index of the closest gesture
   int closest_gesture_index = -1;
   // minimum error to compare
   float min_error = 200;


   // goes through each gestures
   for (int i = 0; i < 10; i++) {
       float total_error = 0.0;
       // Goes through each 40 measurements of the gesture
       for (int j = 0; j < COUNTER; j++) {
           // error for a single measurement
           float error = 0.0; 
           // goes through x, y, z of a single gesture
           for (int k = 0; k < 3; k++) {
               float gesture_value = get_gesture_value(i, j, k);
               // (x1 - x1')^2 + (y1 - y1')^2 + (z1 - z1')^2
               error += pow(current[j][k] - gesture_value, 2);
           }
           // sqrt((x1 - x1')^2 + (y1 - y1')^2 + (z1 - z1')^2) + .....
           total_error += sqrt(error);
       }
       // Averaging the total_error
       total_error /= COUNTER;


       // stores the least error gesture
       if (total_error < min_error) {
           min_error = total_error;
           closest_gesture_index = i;
       }
   }
   // throws an error
   if (min_error > error_threshold) {
       return -1;
   }
   return closest_gesture_index;
}




/**
* Filters out the noice comming from the gesture
*
* It uses sliding windows technique (Data structures and algorithms) in order to
* filter out the noice of the current_gesture measurement.
*
* @param gesture Current gesutre that we are comparing
*/
void averageGesture(float gesture[COUNTER][3]) {
   // windows size for avg
   int window_size = 2;


   // temporary array for storing the result of averaging
   float temp[COUNTER][3] = {0.0};


   for (int i = 0; i < COUNTER; i++) {
       for (int j = 0; j < 3; j++) {
           float sum = 0.0;
           int count = 0;
           for (int k = -window_size / 2; k <= window_size / 2; k++) {
               if (i + k >= 0 && i + k < COUNTER) {
                   sum += gesture[i + k][j];
                   count++;
               }
           }
           temp[i][j] = sum / count;
       }
   }
   // Copy back to gesture
   memcpy(gesture, temp, sizeof(temp));
}
/**
* setup that runs only once
*/
void setup() {
 Serial.begin(9600);
 CircuitPlayground.begin();
 // increasing the sensetivity of accelerometer (resolution)
 CircuitPlayground.setAccelRange(LIS3DH_RANGE_2_G);
}


/**
* loop that runs forever
*/
void loop() {
 delay(1);


 // Checks if button is pressed or not
 CircuitPlayground.leftButton() ? record = true : 0;


 // if button is pressed start the recording
 if(record){
   // gets readings of the accelerometer
   X = CircuitPlayground.motionX();
   Y = CircuitPlayground.motionY();
   Z = CircuitPlayground.motionZ();
  
   // Counts how many measurements. Goes upto value of the COUNTER.
   dynamic_counter++;
  
   // Delay for every measurement
   delay(100);


   // Counts to 4 seconds for convenience
   if(dynamic_counter % 10 == 0){
       Serial.print(dynamic_counter / 10);
       Serial.println("s");
   }


   // Push values to array
   current_gesture[dynamic_counter - 1][0] = X;
   current_gesture[dynamic_counter - 1][1] = Y;
   current_gesture[dynamic_counter - 1][2] = Z;


   // If done measuring the current gesture
   if(dynamic_counter == COUNTER){
     // Plays tone when the session stops
     CircuitPlayground.playTone(500, 100);  


     dynamic_counter = 0;  // reset timer
    
     // average the current_gesture values (filter)
     averageGesture(current_gesture);


     // check motions
     int gesture = find_closest_gesture(current_gesture);


     // If the gesture is tetected go to nex stage
     gesture >= 0 ? detected = true : 0;
     if (detected){
      
       Serial.print("GOT IT! it is gesture: ");
       Serial.println(gesture + 1);
       // show the corresponging neopixel and light up corresponding color
       light_LED(gesture, color_array[gesture]);
     } else {
       // Throws an error (lights up a red LED)
       CircuitPlayground.redLED(true);
       delay(2000);
       CircuitPlayground.redLED(false);
       Serial.print("Gesture is NOT recognized");
     }
    
     // reset all state variables (flags)
     record = false;
     detected = false;
   }
 }
}
