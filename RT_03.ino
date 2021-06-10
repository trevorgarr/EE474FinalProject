/**
 *   University of Washington
 *   ECE/CSE 474,  5/19/2021
 *   
 *   @file   RT_03.ino
 *   @author    Dylan Hylander, Trevor Garrood
 *   @date      9-June-2021
 *   @brief   EE 474 Lab 4 RT-03
 *   
 *  This file combines blinking LED light ------------
 *  
 *  Acknowledgements: "kosme" on GitHub for arduinoFFT example code
 *  https://github.com/kosme/arduinoFFT/blob/master/Examples/FFT_02/FFT_02.ino
 */

#include <stdio.h>
#include <stdlib.h>
#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>
#include <queue.h>

typedef uint32_t TickTypet;

// Task 1/2 defines
#define BIT_3 1<<3
#define BIT_2 1<<2
#define BIT_1 1<<1
#define BIT_0 1<<0
#define CLR_2 ~(1<<2)
#define CLR_1 ~(1<<1)
#define CLR_0 ~(1<<0)
#define CLEAR 0b00000000
#define DELAY_TASK_2 1000
#define DELAY_50 50
#define DELAY_100 100
#define DELAY_150 150
#define DELAY_200 200

// Music note defines
#define NOTE_d4 6802 ///< 293 Hz
#define NOTE_e4 6079 ///< 329 Hz
#define NOTE_c4 7662 ///< 261 Hz
#define NOTE_c3 15385 ///< 130 Hz
#define NOTE_g3 10204 ///< 196 Hz
#define NO_NOTE 0 ///< pause between notes
#define SONG_STOP 3  // play song 3 times before stopping

// Task 3/4 defines
#define ARRAY_SIZE 64  // Size of random int array
#define SERIAL_BITS 19200  // number of bits per second for serial communication
#define TOTAL_FFT 5  // total number of FFT's to calculate

/*
   Task 3/4: These values can be changed in order to evaluate the functions
*/
const double sampling = 40;
const uint8_t amplitude = 4;
uint8_t exponent;
const double startFrequency = 2;
const double stopFrequency = 16.4;
const double step_size = 0.1;

/*
  Task 3/4: These are the input and output vectors
            Input vectors receive computed results from FFT
*/
double vReal[ARRAY_SIZE];
double vImag[ARRAY_SIZE];

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
double (*arrayPtr)[ARRAY_SIZE];  // pointer to random int array
long *timePtr;  // pointer to avg time for a FFT

int numBlinks = 0;  // number of blink cycles
int numSongs = 0;  // number of song cycles
int numFFT = 0;  // number of FFT calculations 

//  declare the Queue handles
static QueueHandle_t    queue1;
static QueueHandle_t    queue2;

// define two tasks for Blink & SongPlay
void TaskBlink( void *pvParameters );
void TaskSongPlay( void *pvParameters );

//  declare the Queue handles
static QueueHandle_t    queue1;
static QueueHandle_t    queue2;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 19200 bits per second:
  Serial.begin(SERIAL_BITS);
  while (!Serial) ;  // wait for it to comeup
  
  // FFT setup
  srand(0);  // Set rand seed
  // initialize array in memory all set to random doubles
  double d[ARRAY_SIZE] = {0};
  arrayPtr = &d;  // intialize global array pointer to array d of random ints
  
  // Set FFT array values to random integers
  for(int i = 0 ; i < ARRAY_SIZE ; i++ ) {
    d[i] = rand();
    // Serial.print("i: ");  Serial.print(i);  Serial.print("   num: ");  Serial.println(d[i]);
  }

  exponent = FFT.Exponent(ARRAY_SIZE);

  //  Set up Queues
  queue1 = xQueueCreate(ARRAY_SIZE, sizeof(double));
  queue2 = xQueueCreate(ARRAY_SIZE, sizeof(double));


  xTaskCreate(
    TaskSetupFFT
    ,  "SetupFFT"
    ,  2000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    TaskFFT
    ,  "FFT"
    ,  2000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*---------------------- Tasks ---------------------*/

/*
  SetupFFT
  Creates an array of random integers and a FreeRTOS Queue
*/
void TaskSetupFFT(void *pvParameters) {
  
  // put into queue
  xQueueSendToBack(queue1, arrayPtr, 0);

  // take out from queue
  xQueueReceive(queue2, &timePtr, 0);

//  long avgTime = *timePtr / TOTAL_FFT;
//  Serial.print("Average Wall Clock Time Per FFT: ");  Serial.println(avgTime);  // Prints time to calculate FFT's
}

/*
  FFT
  Caclculates 5 FFT's for an array of random integers from a FreeRTOS Queue
*/
void TaskFFT(void *pvParameters) {
  vTaskDelay( DELAY_200 / portTICK_PERIOD_MS ); // wait for 200 ms
  TickTypet xStart, xEnd, xDifference;
  xStart = xTaskGetTickCount();
  for (;;) {
    numFFT++;  // number of FFT's calculated
    xQueueReceive(queue1, &arrayPtr, 0);
    for (int i = 0; i < TOTAL_FFT; i++) {  // stop after 5 FFT's
      for(double frequency = startFrequency; frequency<=stopFrequency; frequency+=step_size)
      {
        // Build raw data 
        double cycles = (((ARRAY_SIZE-1) * frequency) / sampling);
        for (uint16_t i = 0; i < ARRAY_SIZE; i++)
        {
          vReal[i] = *arrayPtr[i];
          vImag[i] = 0; //Reset the imaginary values vector for each new frequency
        }
        FFT.Windowing(vReal, ARRAY_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data
        FFT.Compute(vReal, vImag, ARRAY_SIZE, exponent, FFT_FORWARD); // Compute FFT 
        FFT.ComplexToMagnitude(vReal, vImag, ARRAY_SIZE); // Compute magnitudes
        double x = FFT.MajorPeak(vReal, ARRAY_SIZE, sampling);
      }
    }
    xEnd = xTaskGetTickCount();
    xDifference = xEnd - xStart;
    timePtr = &xDifference;
    long avgTime = xDifference / TOTAL_FFT;
    Serial.print("Average Wall Clock Time Per FFT: ");  Serial.println(avgTime);  // Prints time to calculate FFT's
    xQueueSendToBack(queue2, timePtr, 0);
  }
}
