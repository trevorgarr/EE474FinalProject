/*
 * Trevor Garrood
 * EE 474 Lab 4
 */

// --------- Define Statements --------
#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

// Task 1 and 2 defines
#define DELAY_TASK_1 250
#define BIT_3 1<<3
#define BIT_2 1<<2
#define BIT_1 1<<1
#define BIT_0 1<<0
#define CLR_2 ~(1<<2)
#define CLR_1 ~(1<<1)
#define CLR_0 ~(1<<0)
#define CLEAR 0b00000000
#define DELAY_TASK_2 1000

// Music note defines
#define NOTE_d4 6802 // 293 Hz
#define NOTE_e4 6079 // 329 Hz
#define NOTE_c4 7662 // 261 Hz
#define NOTE_c3 15385 // 130 Hz
#define NOTE_g3 10204 // 196 Hz
#define NO_NOTE 0 // pause between notes
#define D4 293 // 293 Hz
#define E4 329 // 329 Hz
#define C4 261 // 261 Hz
#define C3 130 // 130 Hz
#define G3 196 // 196 Hz

// Seven Segment Defines
#define DIGITAL_PIN_THREE 3
#define DIGITAL_PIN_TWO 2
#define DIGITAL_PIN_ONE 1
#define DIGITAL_PIN_ZERO 0
#define LED_INTERVAL 4
#define LED_INTERVAL_OFFSET 1
#define MAX_TIME 100
#define DIGITAL_LEN 4
#define SEG_LEN 8
#define SEV_SEG_LEN 7

// ATM Defines
#define INIT_STATE 0
#define PSWD_IN_STATE 1
#define MENU_STATE 2
#define SELECT_STATE 3
#define DEPOSIT_MENU_STATE 4
#define DEPOSIT_STATE 5
#define WTHDRW_MENU_STATE 6
#define WTHDRW_STATE 7
#define PSWD_MENU_STATE 8
#define PSWD_STATE 9
#define RESET_STATE 10
#define EXIT_MENU_STATE 11
#define EXIT_STATE 12
#define IDLE_STATE 13

// --------------- Variables --------------------

// Keypad Variables
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {25, 27, 29, 31}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {33, 35, 37, 39}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// ATM Variables
byte password[4] = {0, 0, 0, 0};
int passwordLoc = 0;
int keyInputCount = 0;
bool correctPassword = true; // prone to attacks lmao
bool withdrawal = false;
bool deposit = false;
int currentState = IDLE_STATE;
int balance = 0;
String depositAmount = "";
int depositLoc = 0;
String withdrawAmount = "";
int withdrawLoc = 0;

// LCD Variables
const int rs = 32, en = 30, d4 = 28, d5 = 26, d6 = 24, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Seven Segment Variables
byte seven_seg_digits[10][7] = { { 1,1,1,1,1,1,0 },  // = 0
                                 { 0,1,1,0,0,0,0 },  // = 1
                                 { 1,1,0,1,1,0,1 },  // = 2
                                 { 1,1,1,1,0,0,1 },  // = 3
                                 { 0,1,1,0,0,1,1 },  // = 4
                                 { 1,0,1,1,0,1,1 },  // = 5
                                 { 1,0,1,1,1,1,1 },  // = 6
                                 { 1,1,1,0,0,0,0 },  // = 7
                                 { 1,1,1,1,1,1,1 },  // = 8
                                 { 1,1,1,0,0,1,1 }   // = 9
                               };
byte digitPins[] = {14, 11, 12, 13}; // D1, D2, D3, D4
byte segmentPins[] = {9, 2, 3, 5, 6, 8, 7, 4}; // A, B, C, D, E, F, G, DP
byte numberArray[] = {0, 0, 0, 0}; // Start at 0000

// ISR Variables
char TCNT2init= 130;

// Task Initializers
void TaskBlink( void *pvParameters );
void TaskSongPlay( void *pvParameters );
void TaskKeypad( void *pvParameters );
void TaskServo( void *pvParameters );


// -------------------- Setup ---------------------------

void setup() {
  // Setup RT-3
  //randomSeed(l);
  
  //Setup for Seven Segment Display
  for (int i = 0; i < DIGITAL_LEN; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH); // common cathode: HIGH is off
  }
  for (int j = 0; j < SEG_LEN; j++) {
    pinMode(segmentPins[j], OUTPUT);
    digitalWrite(segmentPins[j], LOW); // common cathode: LOW is off
  }

  // setup for ISR
  DDRD |= 0b0000100;
  TCCR2A &= CLEAR;
  TCCR2B &= CLEAR;
  //Set WGM2 bits 3:0 to 0b1111 corresponding to "Fast PWM" with Top set to the value stored in OCR2A
  TCCR2A |= 0b00000011;
  TCCR2B |= 0b00011000;
  // Prescale 128 (Timer/Counter started)
  TCCR2B |= 0b00000101;
  // Set COM2A0 and COM2A1 to 0b01 to select the compare mode: "Toggle OC2A on Compare Match"
  TCCR2A |= 0b01000000;
  TIMSK2 |= (1<<TOIE2);
  TCNT2 = TCNT2init;

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskSongPlay
    ,  "SongPlay"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    TaskKeypad
    ,  "Keypad"
    ,  800  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    TaskServo
    ,  "Servo"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  delay(500);
  
  vTaskStartScheduler();
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/*
 * Blinks an LED
 */
void TaskBlink(void *pvParameters) {
  //pinMode(47, OUTPUT);
  DDRL |= BIT_2; // pin 47

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(47, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100ms
    digitalWrite(47, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for 200ms
  }
}

/*
 * Plays the close encounters theme
 */
void TaskSongPlay(void *pvParameters) {
  DDRL |= BIT_3; // pin 46 -> OC5A
  //pinMode(46, OUTPUT);

  int taskTwoPlays = 0;

  //setup for clock for speaker
  TCCR5A &= CLEAR;
  TCCR5B &= CLEAR;
  //Set WGM4 bits 3:0 to 0b1111 corresponding to "Fast PWM" with Top set to the value stored in OCR5A
  TCCR5A |= 0b00000011;
  TCCR5B |= 0b00011000;
  //Set timer 5 prescaler to 8?
  TCCR5B |= 0b00000010;
  // Set COM5A0 and COM5A1 to 0b01 to select the compare mode: "Toggle OC5A on Compare Match"
  // 5A corresponds to OC5A, which is pin 64 on the mega 
  TCCR5A |= 0b01000000;
  for (;;) {
    OCR5A = NOTE_d4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_e4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_c4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_c3;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_g3;
    vTaskDelay( (DELAY_TASK_2 * 4) / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NO_NOTE;
    vTaskDelay( (DELAY_TASK_2 * 1.5) / portTICK_PERIOD_MS ); // wait for 200ms
    taskTwoPlays++;
    if (taskTwoPlays == 3) {
      currentState = INIT_STATE;
      vTaskSuspend(NULL);
    }
  }
}  

/*
 * Brains of the ATM. Oversee the UI and menu system as well as sends
 * data to other tasks
 */
void TaskKeypad(void *pvParameters) {
  lcd.begin(16, 2);

  for (;;) {
    char key = keypad.getKey(); // Read the key
    if (currentState == IDLE_STATE) {
      
    } else if (currentState == INIT_STATE) {
      lcd.print("Enter Pswd: ");
      currentState = PSWD_IN_STATE;
    } else if (currentState == PSWD_IN_STATE) {
      if (key){
        lcd.print("*");
        if (password[keyInputCount] != (key - '0')) {
          correctPassword = false;
        }
        keyInputCount++;
        vTaskDelay(250 / portTICK_PERIOD_MS);
        if (keyInputCount == 4) {
          lcd.clear();
          if (correctPassword) {
            lcd.print("Welcome");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            currentState = MENU_STATE;
          } else {
            lcd.print("Incorrect Pswd");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            currentState = RESET_STATE;
          }
        }
      }
    } else if (currentState == MENU_STATE) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("A) Add B) Wthdrw");
      lcd.setCursor(0, 1);
      lcd.print("C) Pswd D) Exit");
      currentState = SELECT_STATE;
    } else if (currentState == SELECT_STATE) {
      if (key) {
        if (key == 'A') {
          currentState = DEPOSIT_MENU_STATE;
        } else if (key == 'B') {
          currentState = WTHDRW_MENU_STATE;
        } else if (key == 'C') {
          currentState = PSWD_MENU_STATE;
        } else if (key == 'D') {
          currentState = RESET_STATE;
        } else {
          lcd.clear();
          lcd.print("Invalid Input");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          currentState = MENU_STATE;
        }
      }
    } else if (currentState == DEPOSIT_MENU_STATE) {
      lcd.clear();
      lcd.print("Deposit: ");
      currentState = DEPOSIT_STATE;
    } else if (currentState == DEPOSIT_STATE) {
      if (key){
        lcd.print(key);
        if (depositLoc == 6 || key == '#') {
          char buf[depositAmount.length()+1];
          depositAmount.toCharArray(buf, depositAmount.length()+1);
          int depositNum = atoi(buf);
          balance += depositNum;
          lcd.clear();
          lcd.print("Thank you");
          Serial.print("Current Balance: ");
          Serial.println(balance);
          deposit = true;
          vTaskDelay(1500 / portTICK_PERIOD_MS);
          currentState = EXIT_MENU_STATE;
          depositAmount = "";
          depositLoc = 0;
        } else if (depositLoc < 6) {
          depositAmount += key;
          depositLoc++;
        }
      }
    } else if (currentState == WTHDRW_MENU_STATE) {
      lcd.clear();
      lcd.print("Withdraw: ");
      currentState = WTHDRW_STATE;
    } else if (currentState == WTHDRW_STATE) {
      if (key){
        lcd.print(key);
        if (withdrawLoc == 6 || key == '#') {
          char buf[withdrawAmount.length()+1];
          withdrawAmount.toCharArray(buf, withdrawAmount.length()+1);
          int withdrawNum = atoi(buf);
          if (withdrawNum <= balance) {
            balance -= withdrawNum;
            lcd.clear();
            lcd.print("Thank you");
            Serial.print("Current Balance: ");
            Serial.println(balance);
            withdrawal = true;
          } else {
            lcd.clear();
            lcd.print("Insufficient $");
          }
          vTaskDelay(1500 / portTICK_PERIOD_MS);
          currentState = EXIT_MENU_STATE;
          withdrawAmount = "";
          withdrawLoc = 0;
        } else if (withdrawLoc < 6) {
          withdrawAmount += key;
          withdrawLoc++;
        }
      }
    } else if (currentState == PSWD_MENU_STATE) {
      lcd.clear();
      lcd.print("New Pswd: ");
      currentState = PSWD_STATE;
    } else if (currentState == PSWD_STATE) {
      if (key){
        if (passwordLoc < 4) {
          lcd.print(key);
          password[passwordLoc] = key - '0';
          passwordLoc++;
        } else if (passwordLoc == 4 && key == '#') {
         lcd.clear();
         lcd.print("Password Set");
         vTaskDelay(1500 / portTICK_PERIOD_MS);
         currentState = EXIT_MENU_STATE;
         passwordLoc = 0;
        }
      }
    } else if (currentState == EXIT_MENU_STATE) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("A) Continue");
      lcd.setCursor(0, 1);
      lcd.print("B) Exit");
      currentState = EXIT_STATE;
    } else if (currentState == EXIT_STATE) {
      if (key){
        if (key == 'A') {
          currentState = MENU_STATE;
        } else if (key == 'B') {
          currentState = RESET_STATE;
        }
      }
    } else if (currentState == RESET_STATE) {
      lcd.clear();
      keyInputCount = 0;
      correctPassword = true;
      currentState = INIT_STATE;
      depositAmount = "";
      depositLoc = 0;
      withdrawAmount = "";
      withdrawLoc = 0;
      passwordLoc = 0;
    }
  }
}

/*
 * Drives a servo motor when either a deposit or withdrawal is made
 */
void TaskServo(void *pvParameters) {
  int pos = 0;
  pinMode(53, OUTPUT);
  for (;;) {
    if (deposit || withdrawal) {
      digitalWrite(53, HIGH);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      digitalWrite(53, LOW);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      deposit = false;
      withdrawal = false;
    }
  }
}

/*
 * Sets up ISR vector for TIMER2
 */
volatile static int isrCount;
byte digitsPos[4] = {DIGITAL_PIN_THREE, DIGITAL_PIN_TWO, DIGITAL_PIN_ONE, DIGITAL_PIN_ZERO};

ISR(TIMER2_OVF_vect) { 
  TCNT2 = TCNT2init;
  static int digitPos;
  static int state;
  intToNumber(balance, numberArray);
  if (isrCount == 0) {
    if (++digitPos >= 4) {
      digitPos = 0;
    }
    showDigit(numberArray[digitPos], digitsPos[digitPos]);
  }
  if (isrCount == 3) {
    hideDigit(DIGITAL_PIN_ZERO);
    hideDigit(DIGITAL_PIN_ONE);
    hideDigit(DIGITAL_PIN_TWO);
    hideDigit(DIGITAL_PIN_THREE);
  }
  if (isrCount == 4) {
    isrCount = -1;
  }
  isrCount++;
};

/*--------------------------------------------------*/
/*---------------- Helper Functions ----------------*/
/*--------------------------------------------------*/

/*
 * Displays a number on the Seg7 display at a specific digitPin location
 * @params {integer} number - a number 0-9 to be displayed on the seg7
 * @params {integer} digitPin - specific digit pin 0-3 to be written to
 */
void showDigit(int number, int digitPin) {
  digitalWrite(digitPins[digitPin], LOW);
  for (int i = 0; i < SEV_SEG_LEN; i++) {
    digitalWrite(segmentPins[i], seven_seg_digits[number][i]);
  }
}

/*
 * Hides a digit and allows for the others to be written to. HIGH is OFF
 * @params {integer} digit - a digit 0-3 designating which digit on seg7 to turn off
 */
void hideDigit(int digit) {
  digitalWrite(digitPins[digit], HIGH);
}

/*
 * converts and integer number to an array of digits
 * @params {integer} num - a number 0-9999 to be converted to an array
 * @params {byte} numberArray - an array of digits converted from the number passed in 
 */
void intToNumber(int num, byte numberArray[]) {
  for (int i = 0; i < 4; i++) {
    numberArray[i] = num % 10;
    num /= 10;
  }
}
