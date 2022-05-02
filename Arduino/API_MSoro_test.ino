/* BasicMSoRo.ino */
/* Author: Arun Niddish Mahendran
 * Last updated: 4/26/22
 * This code enables gait-based control of a multi-limbed robot with
 * with bang-bang (i.e., on/off) actuation. The user will use the 
 * user-defined parameter block to define
 *   1. Arduino pin numbers
 *   2. time constants
 *   3. fundamental locomotion gaits
 * and can control gait switching by manipulating the loop block. 
 * Future work:
 *   1. More advanced gait switching by recognizing common nodes, 
 *      including automatic gait switching handling
 *   2. Communication with MATLAB for feedback control
 *   3. Automatic gait permutation handling
 */


/* User-defined parameters */
const int8_t motor[] ={2,3,5,6,9,10,11,12};      // motor pins on Arduino
const int T_transition = 450;    // total transition time constant in ms.
const int T_unspool = 50;        // motor unspooling time constant in ms.
int8_t cycle[20][20];
int8_t cycle_gait[20];
//int8_t cycle1[20]; 
//int8_t cycle2[20];                       // input gait cycle 2
String cmd;
char inString[50];
String no_of_times;
int gait_value_int;
String gait_type;
String gait_value;
String ser_read_string;
int no_of_times_int;
String check;
String flag;
String gait_length;
int gait_length_int;
int i,j ;
int size_gait[20];
String gaits = "ABCDEFGHIJ";
int index;
/* If you want to use more than 2 cycles make sure to also add variables
 * to define their lengths. 
 */
/* Initializations (should not change) */
const int8_t number_of_motors = sizeof(motor)/ 2;    // bang-bang control
/* If more robot states are desired (e.g., including intermediate actuation
 * states), the exponent base here can be adjusted. 
 */
const int number_of_states = pow(2, number_of_motors);
/* Initialize matrix to store all actuation permutations (robot states). */
int state_matrix[number_of_motors][number_of_states] = { };   
/* Use booleans to avoid excessive / unwanted motor unspooling. */
bool just_curled[number_of_motors] = { };
bool just_relaxed[number_of_motors] = { };
//const int8_t cycle1_size = sizeof(cycle1);
//const int8_t cycle2_size = sizeof(cycle2);

/* Setup function for communication and state definition (runs once). */
void setup() {
  Serial.begin(9600); // baud rate for serial monitor communication (bps).
  define_states();
  delay(5000);
}


/* Anything placed inside the loop function will cycle continously forever
 * unless interrupts are added. 
 */
void loop() 
{
  while(Serial.available()==0){}
    cmd = serial_read(); /* Initializing the action to be performed - to define gaits("define") or execute the gaits("start") */
    if(cmd == "define")
    {
      define_cycle();
    }
    if(cmd == "start")
    {
      start_cycle();
    }
}

/* This  function defines a matrix of robot states. Basically, it labels each
 * state with a binary number that corresponds to which motors are on or off.
 * 1 = on, 0 = off 
 */
void define_states() {
  for (int k=0; k<= number_of_states-1; k++) {
    int spacing=1;
    for (int j=number_of_motors-1; j>=0; j--) {
      if (state_matrix[j][k]==0 && k+spacing<=number_of_states-1){
        state_matrix[j][k+spacing]=1;
      }
      spacing = spacing*2;
    }
  }
  for (int m=0; m<=number_of_states-1; m++) {
    /* This part is optional as it just prints the binary value of each 
     *  state number (helpful for debugging).
     */
    Serial.print("State ");
    Serial.print(m+1);
    Serial.print(" = ");
    for (int n=0; n<=number_of_motors-1;n++){
      Serial.print(state_matrix[n][m]);
    }
    Serial.println(" ");
    }
}

/* This  function controls the motor based on the gait cycle (i.e., array of
 * state numbers) provided.
 */
void cycle_through_states (int8_t *cycle, int8_t cycle_size) {
  for (int i=0; i<cycle_size;i++) {
    unsigned long transition_start = millis();
    Serial.print("State ");
    Serial.print(cycle[i]);
    Serial.print(": ");
    for (int j=0; j<=3; j++) {
      Serial.print(state_matrix[j][cycle[i]-1]);
      if (state_matrix[j][cycle[i]-1] == 0 && just_relaxed[j]==false) {
        digitalWrite(motor[2*j], LOW);
        digitalWrite(motor[2*j+1], HIGH);
        just_relaxed[j] = true;
        just_curled[j] = false;
      }
      else if (state_matrix[j][cycle[i]-1] == 1)  {
        digitalWrite(motor[2*j],HIGH);
        digitalWrite(motor[2*j+1], LOW);
        just_relaxed[j] = false;
        just_curled[j] = true;
      }
      if (j==3) {
        delay(T_unspool);
        for (int k=0; k<=3; k++) {
          if (state_matrix[k][cycle[i]-1] == 0) {
            digitalWrite(motor[2*k+1], LOW);
          }
          if (k==3){
            while (millis() -transition_start<= T_transition-1) {
            delay(1);
            }
            Serial.println("");
          }
        }
       }
      }
    }
  for (int j=0; j<=3; j++) {
  digitalWrite(motor[2*j],LOW);
  digitalWrite(motor[2*j+1],LOW);
}
}


/* This function takes input for defining the gait types. 
 *  Example A = [2,3,5,9];
 *  "define A 2 3 5 9 end " - Leave a space after the 'end'
 */
void define_cycle()
{
  gait_type = serial_read();
  index = gaits.indexOf(gait_type);
  Serial.print("The index is ");
  Serial.println(index);
  gait_value = "exist"; /* Initially the gait value exists which is sent from MATLAB*/
  i = 0;
      while(gait_value != "end")
       {
          gait_value = serial_read();
          gait_value_int = gait_value.toInt();
          if(gait_value != "end" && gait_value_int != 0 )
          {
          cycle[index][i] = (int8_t)gait_value_int;
          Serial.println(cycle[index][i]);
          i = i+1;  
          }  
       size_gait[index] = i;
       }
       
        Serial.println("#Defined"); /* This is printed so that Matlab can read it from Serial port for acknowledging that gait defining process 
                                     has been completed*/   
}

/*
 * For the given definition of gaits type, this function performs the execution of the gait and number of times it needs to be performed.
 * Example: "start A 12 " - Performs gait A for 12 times. Leave a space after the '12'.
 */
void start_cycle()
{
  byte ser_read;
  gait_type = serial_read();
  index = gaits.indexOf(gait_type);
  for(j = 0;j<size_gait[index];j++)
    {
     cycle_gait[j] = cycle[index][j];
    } 
  no_of_times = serial_read();
  no_of_times_int = no_of_times.toInt();
      for (int k=0; k<no_of_times_int ; k++) 
        {
            cycle_through_states(cycle_gait, size_gait[index]);
        } 

        Serial.print("#Completed Cycle ");
        Serial.print(gaits.charAt(index));
        Serial.print(" ");
        Serial.print(no_of_times_int);
        Serial.println(" times");

}

/* This functions reads data from serial port*/
String serial_read()
{
  byte ser_read;
  ser_read = Serial.readBytesUntil(' ', inString, 100);
  inString[ser_read] = '\0';
  ser_read_string = String(inString);
  return ser_read_string;
}
