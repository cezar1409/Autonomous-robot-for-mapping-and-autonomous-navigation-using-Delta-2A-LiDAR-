// Motor A pins
#define PWMA 12    // Speed control
#define DIRA1 34   // Direction control 1
#define DIRA2 35   // Direction control 2

// Motor B pins
#define PWMB 8     // Speed control
#define DIRB1 37   // Direction control 1
#define DIRB2 36   // Direction control 2

// Motor C pins
#define PWMC 9     // Speed control
#define DIRC1 43   // Direction control 1
#define DIRC2 42   // Direction control 2

// Motor D pins
#define PWMD 5     // Speed control
#define DIRD1 A4   // Direction control 1
#define DIRD2 A5   // Direction control 2

// Motor control macros
#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG 
#endif

// PWM speed settings
#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 130;

// Track current movement state
char current_movement = 'Z';  // Start with stopped state

bool should_change_movement(char new_movement) {
  if (new_movement == current_movement) {
    return false;  // No change needed if same movement
  }
  current_movement = new_movement;
  return true;
}

// Movement functions for mecanum wheel configuration
// Wheel layout:
//    A-----B   
//    |     |
//    |     |
//    C-----D

// Forward movement:
//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE() {
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);    
}

// Backward movement:
//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK() {
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// Left sideways movement:
//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2() {
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// Right sideways movement:
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2() {
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}

// Rotate left (counterclockwise):
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3() {
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// Rotate right (clockwise):
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3() {
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

// Stop all motors:
//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP() {
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

// Process serial commands for robot movement
void UART_Control() {
  char Uart_Date = 0;
  if(SERIAL.available()) {
    Uart_Date = SERIAL.read();
    
    if (!should_change_movement(Uart_Date)) {
      return;  // Skip if same movement
    }
    
    switch(Uart_Date) {
      case 'A':  // Forward
        ADVANCE(); 
        M_LOG("Forward\r\n");        
        break;
      case 'B':  // Backward
        BACK();
        M_LOG("Backward\r\n");
        break;
      case 'G':  // Left (sideways)
        LEFT_2();   
        M_LOG("Left\r\n");       
        break;
      case 'C':  // Right (sideways)
        RIGHT_2();  
        M_LOG("Right\r\n");        
        break;
      case 'E':  // Rotate Left
        LEFT_3();
        M_LOG("Rotate Left\r\n");
        break;
      case 'F':  // Rotate Right
        RIGHT_3();
        M_LOG("Rotate Right\r\n");
        break;
      case 'Z':  // Stop
        STOP();     
        M_LOG("Stop\r\n");       
        break;
      case 'L':  // High speed
        Motor_PWM = 240;
        M_LOG("High speed\r\n");
        break;
      case 'M':  // Normal speed
        Motor_PWM = 130;
        M_LOG("Normal speed\r\n");
        break;
    }
  }
}

// Initialize all motor control pins
void IO_init() {
	pinMode(PWMA, OUTPUT);
	pinMode(DIRA1, OUTPUT);pinMode(DIRA2, OUTPUT);
	pinMode(PWMB, OUTPUT);
	pinMode(DIRB1, OUTPUT);pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
	pinMode(DIRC1, OUTPUT);pinMode(DIRC2, OUTPUT);
	pinMode(PWMD, OUTPUT);
	pinMode(DIRD1, OUTPUT);pinMode(DIRD2, OUTPUT);
	STOP();
}

void setup() {
	SERIAL.begin(9600);
	IO_init();
}

void loop() {
  UART_Control();
}
