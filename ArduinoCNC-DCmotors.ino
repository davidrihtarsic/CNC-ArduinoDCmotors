/*
   --== GBRL for DC motors ==--
   written by: David Rihtaršič
   tested by : Tomaž Kušar

   based on  :https://github.com/grbl/grbl/wiki
              "Grbl is a free, open source, high performance software for controlling
              the motion of machines that move, that make things, or that make things
              move, and will run on a straight Arduino. If the maker movement was an
              industry, Grbl would be the industry standard." (Sungeun K. Jeon Ph.D.)
   version   :0.95 (stable, but beta)
   date      :2016-03-11

   --== ABOUT ==--
    ... to do
   --== INSTRUCTIONS FOR USERS ==--
   --== Connecting DC motors & stepping switches ==--
                                                     ______
                                        ___         |      |
                                    ---|   |--------|      |------
                                   |   |PWR|        | USB  |      |
                                   |   |___|        |______|      |
                                   |                       oo     |
                                  []                       oo  13 []
                                  [] ioref                 oo  12 []
                                  [] rst                       11 []
                                  [] 3.3                       10 []
                                  [] 5V     _____               9 []----o Spindle controll
                                  [] GND   [  A  ]              8 []
                                  [] GND   [  T  ]                |
                                  [] VIN   [  m  ]              7 []----o DC motor X pinA
                                   |       [  e  ]              6 []----o DC motor X pinB
                  Limit SW z o----[] 14    [  g  ]              5 []----o DC motor Y pinA
                  Limit SW y o----[] 15    [  a  ]              4 []----o DC motor Y pinB
                  Limit SW x o----[] 16    [  3  ]              3 []----o DC motor Z pinA
               Position SW z o----[] 17    [  2  ]              2 []----o DC motor Z pinB
               Position SW y o----[] 18    [  8  ]    ooo       1 []
               Position SW x o----[] 19    [__P__]    ooo       0 []
                                   \_____________________/-------´

   --== Motor circuit example ==--
   Motors can be driven by L293 full H-bridge circuit...

                                       [+5V]        [+5V..+12V]
                                         o               o
                 < DC motor Y pinB >--+  |  +---------+  |  +--< DC motor X  pinA >
                                      |  +--[en1,2 vc1]--+  |
                                      +-----[1a     4a]-----+    /---\
                         +------------------[1y     4y]---------( DCx )--+
                         |          [GND]o--[GND   GND]--o[GND]  \---/   |
                         |   /---\  [GND]o--[GND   GND]--o[GND]          |
                         +--( DCy )---------[2y     3y]------------------+
                             \---/    +-----[2a     3a]------+
                                      |  +--[vc2 3,4en]--+   |
                 < DC motor Y pinB >--+  |  +---------+  |   +--< DC motor X  pinB >
                                         o               o
                                    [+5V..+12V]        [+5V]

   --== Switches circuit diagram ==--

                               __T__
       < Limit SW z >----------O   O---+
                                       |
                                      ===
                                       =
*/
byte Motor[3][2] = {
  {6, 7}, //DC motor x - connected on pin7 & pin6
  {4, 5}, //DC motor y - connected on pin5 & pin4
  {2, 3}  //DC motor z - connected on pin3 & pin2
};
byte Spindle[2] = {
  9, // Spindle controll pin
  9 // ... to do
};
byte Position_switch[3] = {
  19, //Position switch X pin
  18, //Position switch Y pin
  17  //Position switch Z pin
};
byte Limit_switch[3] = {
  16, //Limit switch X pin
  15, //Limit switch Y pin
  14  //Limit switch Z pin
};
/*
   --== Set the pitch of axes ==--
   Set how many mms does tool move along the axes at every step click - from short
   to short.
*/
float pitch[3] = {
  0.7,    // pitch in mms for x axes
  0.7,    // pitch in mms for y axes
  0     // pitch in mms for z axes // 0 - no Z axes
};
/*  Upload to Arduino... That’s it.

   --== INSTRUCTIONS FOR DEVELOPERS ==--
   As you can see there is a lot to do...
   Bellow you can find bool variable “debug” which can be set to true. If so,
   in the code you can do something like:
   if (debug) Print(“some test variables goes here…”);
   and you’ll see what is going on…
*/
bool debug = false;
/*
   --== To Do ==--
   Here are some “to do” priorities that should be done in forthcoming future...
   - all motors can move at the same time
   - accuracy of the motor position can be pitch/2 (advanced: pitch/4)
   - stop function
   - arc G02

   --== BUGS ==--
*/

/*
    --== $G - View gcode parser state ==--
    https://github.com/grbl/grbl/wiki/Configuring-Grbl-v0.9#g---view-gcode-parser-state
    The default setting are:
    [G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 S0.0 F500.0]
*/
String motion_mode = "G0";
String Coordinate_system_select = "G54" ;
String plane_select = "G17" ;
String distance_mode = "G90" ;
String arc_IJK_distance_mode = "G91.1" ;
String feed_rate_mode = "G94";
String units_mode = "G21";
String cutter_radius_compensation = "G40";
String tool_length_offse = "G49";
String program_mode = "M0";
String spindle_state = "M5";
String coolant_state = "M9";

void view_parser_state() {
  //Println("[G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 S0.0 F500.0]");
  Print("[");
  Print(motion_mode); Print(" ");                       // "G0"
  Print( Coordinate_system_select ); Print(" ");        // "G54"
  Print( plane_select ); Print(" ");                    // "G17"
  Print( distance_mode ); Print(" ");                   // "G90"
  Print( arc_IJK_distance_mode ); Print(" ");           // "G91.1"
  Print( feed_rate_mode ); Print(" ");                  // "G94"
  Print( units_mode ); Print(" ");                      // "G21"
  Print( cutter_radius_compensation ); Print(" ");      // "G40"
  Print( tool_length_offse ); Print(" ");               // "G49"
  Print( program_mode ); Print(" ");                    // "M0"
  Print( spindle_state ); Print(" ");                   // "M5"
  Print( coolant_state ); Println(" T0 S0.0 F500.0]");    // "M9";
}  /*
   --== CURRENT STATUS ==--
   https://github.com/grbl/grbl/wiki/Configuring-Grbl-v0.9#---current-status
   The ? command immediately returns Grbl's active state and the real-time current
   position,both in machine coordinates and work coordinates. For example:
   <Idle,MPos:5.529,0.560,7.000,WPos:1.529,-5.440,-0.000>
*/
bool new_xyz[3] = {false,false,false};
bool Position_switch_value[3];        //Value of x,y and z position switches
float dMove[3] = {0, 0, 0};           //movement in x, y, z direction
float XYZ[3] = {0, 0, 0};             //recived values for x,y,z
bool active_state_hold = false;
long n_cliks_to_perform[3] = {0, 0, 0};//num of clicks in x,y,z direction
float move_every_nth_clicks[3]={0,0,0};
//float move_motor_next_nth_click[3]={0,0,0};
long old_Position_switch_state[3] = {
  digitalRead(Position_switch[0]),
  digitalRead(Position_switch[1]),
  digitalRead(Position_switch[2])
};
bool motor_direction[3];              //direction of x, y & z (0= neg, 1= pos)
float WPos[3] = {0.0, 0.0, 0.0};     //
float MPos[3] = {0.0, 0.0, 0.0};     //
String active_state = "Idle";        // Idle, Run, Hold, Door, Home, Alarm, Check
bool realtime_cmd_busy = false;
void view_current_status() {
  //Println("<Idle,MPos:5.529,0.560,7.000,WPos:1.529,-5.440,-0.000>"
  realtime_cmd_busy = true;
  sei();
  Print(("<" + active_state));
  Print((",MPos:" + String(MPos[0]) + "," + String(MPos[1]) + "," + String(MPos[2])));
  Println((",WPos:" + String(WPos[0]) + "," + String(WPos[1]) + "," + String(WPos[2]) + ">"));
  realtime_cmd_busy = false;
}
void find_home_position() {
  active_state  = "Run";
  for (byte i = 0 ; i < 3; i++) {
    //send motor to -x direction
    if (pitch[i] != 0) {
      motor_direction[i] = false;
      digitalWrite(Motor[i][0], motor_direction[i]); //run the motor
      digitalWrite(Motor[i][1], !motor_direction[i]); //run the motor
      while (digitalRead(Limit_switch[i]) == HIGH) {}
      digitalWrite(Motor[i][0], LOW); //run the motor
      digitalWrite(Motor[i][1], LOW); //run the motor
      WPos[i] = 0;
      MPos[i] = 0;
    }
  }
  active_state = "Idle";
}

void set_spindle() {
  if (spindle_state == "M3"){
      digitalWrite(Spindle[0], HIGH);
  } else if (spindle_state == "M4"){
      digitalWrite(Spindle[1], HIGH);
  } else if (spindle_state == "M5"){
      digitalWrite(Spindle[0], LOW);
      digitalWrite(Spindle[1], LOW);
  }
}

void move_x_y_z() {
  //To Do
  active_state = "Run";
  calculate_dMove();            //calculating the mm to move in any direction
  calculate_clicks_to_perfprm();
  set_move_direction();         //direction of move (false = negative, True = pos.dirr.)
  // --== CONTROLLING THE MOTORS ==--
  // katera smer ima največ premikov...
  long max_cliks = n_cliks_to_perform[0];
  for (byte i = 0; i < 3; i++){
    if (n_cliks_to_perform[i] > max_cliks) max_cliks = n_cliks_to_perform[i];  
  }
  if (debug) Debug (("max cliks=" + String(max_cliks)));

  //izračun po kolikih klikih naj se motor premika
  for (byte i = 0; i < 3; i++){
    if (n_cliks_to_perform[i]!= 0){
      if (n_cliks_to_perform[i] != max_cliks){
        move_every_nth_clicks[i] = max_cliks / double(n_cliks_to_perform[i]+1);
      } else move_every_nth_clicks[i]=1;
    }
    if (debug) Debug (("motor"+ String(i) + " move every " + String(move_every_nth_clicks[i])));
  }

  //zaženemo motorje od točke do točke
  int running_motors[3] = {0,0,0};
  float move_motor_next_nth_click[3]={0,0,0};
  for (byte i = 0; i < 3; i++){
    move_motor_next_nth_click[i]=move_every_nth_clicks[i];
  }
  
  for (byte n = 1; n < max_cliks+1; n++){
  //vključi posamezni motor, če je to potrebno
    for (byte m = 0 ; m < 3; m++){
        if (debug) Debug("n=" + String(n) + " mM[" + String(m) + "]=" + String(move_motor_next_nth_click[m]));
        if ( (n >= move_motor_next_nth_click[m]) && (move_motor_next_nth_click[m] != 0) )  {
          move_motor_next_nth_click[m] = move_motor_next_nth_click[m]+move_every_nth_clicks[m];
          digitalWrite(Motor[m][0], motor_direction[m]); //run the motor
          digitalWrite(Motor[m][1], !motor_direction[m]); //run the motor
          running_motors[m]=1;
      }
    }
    if (debug) Debug("running motors=" + String(running_motors[0]) + String(running_motors[1]) + String(running_motors[2]));
  //preveri stikala za pomik in izračun nove pozicije
    while (running_motors[0] || running_motors[1] || running_motors[2]){
      for (byte m = 0 ; m < 3; m++){
            if ((Position_switch_edge_detected(m) == true) && running_motors[m]!=0 ){
              delay(50);
              //izračun nove pozicije
              if (motor_direction[m] == true) {
                WPos[m] += pitch[m]/2;
                MPos[m] += pitch[m]/2;
              } else {
                WPos[m] -= pitch[m]/2;
                MPos[m] -= pitch[m]/2;
              }
              digitalWrite(Motor[m][0], LOW); //run the motor
              digitalWrite(Motor[m][1], LOW); //run the motor
              running_motors[m]=0;
            }
      }      
    } 
    if (debug) Debug("n= " + String(n) + "nX= " + String(move_motor_next_nth_click[0]) + "nY= " + String(move_motor_next_nth_click[1]) + "nZ= " + String(move_motor_next_nth_click[2]));
  }
  
  for (byte i = 0; i < 3; i++){
    dMove[i] = 0;
    XYZ[i] = 0;
    move_every_nth_clicks[i]=0; 
    new_xyz[i] = false;
  }
  
  if (active_state=="Run") active_state = "Idle";
}

/*  --== sub function for MOVE INSTRUCTIONS ==--
    calculate_dMove()
    set_dirrection_of_move()
*/
void calculate_dMove() {
  if (distance_mode == "G90") { //if distance is in ABSOLUTE MODE - need to calculate dX
    for (byte i = 0 ; i < 3; i++) {
      if (new_xyz[i]) dMove[i] = XYZ[i] - WPos[i];
      if (debug) Debug("dMove[" + String(i)+"]="+String(dMove[i]));
    }
  } else if (distance_mode == "G91") { //REALTIVE move
    //no need to calculate dMove _x _y _z
    for (byte i = 0 ; i < 3; i++) {
      if (new_xyz[i]) dMove[i] = XYZ[i];
      if (debug) Debug("dMove[" + String(i)+"]="+String(dMove[i]));
    }
  }
}

void calculate_clicks_to_perfprm() {
  //CALCULATING THE n_clicks for every x direction
  for (byte i = 0 ; i < 3; i++) {
    if (pitch[i]!= 0){
      n_cliks_to_perform[i] = abs(dMove[i] / pitch[i]*2);
      if (dMove[i] < 0 && motor_direction[i] == true){
        n_cliks_to_perform[i]++;
        WPos[i] += pitch[i]/2;
        MPos[i] += pitch[i]/2;
      }
      if (dMove[i] > 0 && motor_direction[i] == false){
        n_cliks_to_perform[i]++;
        WPos[i] -= pitch[i]/2;
        MPos[i] -= pitch[i]/2;
      }
      if (debug) Debug((String(dMove[i]) + " " + String(n_cliks_to_perform[i])));
    } else n_cliks_to_perform[i]=0;
  }  
}

void set_move_direction() {
  for (byte i = 0 ; i < 3; i++) {
    if (dMove[i] < 0) {
      motor_direction[i] = false;   //neg. direction - x is decreasing
    } else {
      motor_direction[i] = true;    //pos. direction - x is increasing
    }
  }
}

bool Position_switch_edge_detected(int i){
  int signal_state = digitalRead(Position_switch[i]);
  //if (debug) Debug("SW-" + String(i) + " " + String(signal_state));
  if (signal_state == old_Position_switch_state[i]){
    return false; 
  }else {
    old_Position_switch_state[i] = signal_state;
    return true;
  }
}
/*  --== inputBuffer ==--
    InpitBuffer is buffer of 256 bytes and is used for string the data send
    from UART GRBL sender.
*/
byte inputBuffer[256];       // a string to hold incoming data
byte BufferIndex = 0;        // last saved char in inputBuffer
byte BufferRead = 0;         // last yet read inputBuffer
char inChar = 0;
char rdChar = 0;
String g_cmd = "";
int find_next_letter_in_string(String str, int start) {
  int i = start;
  while (i <= str.length()) {
    if ( ((int)str[i] < (int)'A' || (int)str[i] > (int)'Z') && ((int)str[i] != (int)'$') ) {
      i++;
    } else {
      break; //letter founded
    }
  }
  if (i > str.length()) i = -1;
  return i;
}
String read_sub_string(String str, byte Start, byte Stop) {
  String read_str = "";
  for (byte i = Start; i < Stop ; i++) {
    if (i > str.length()) break;
    read_str += str[i];
  }
  return read_str;
}

/*     --== GBRL DATA ==--
        A lot of gbrl data is still not supported...
        The supported code is:
        - Realtime commands
           '?' - Current status
        - G-CODE:
           G0 - move
           G1 - move
*/
bool command_complete = false;

void setup() {
  // initialize serial:
  BufferIndex=0;
  BufferRead=0;
  UART_config();
  // Motor Pinous
  for (byte i = 0; i < 3; i++) {
    for (byte pin = 0; pin < 2; pin++) {
      pinMode(Motor[i][pin], OUTPUT);
      digitalWrite(Motor[i][pin],LOW);
    }
  }
  //Set spindle pinout
  pinMode(Spindle[0],OUTPUT);
  pinMode(Spindle[1],OUTPUT);
  // Position & Limit switch Pinouts
  for (byte i = 0; i < 3; i++) {
    pinMode(Position_switch[i], INPUT_PULLUP);
    pinMode(Limit_switch[i], INPUT_PULLUP);
  }
  interrupts();
}

void loop() {
  while (BufferRead != BufferIndex) {
    if (debug) Debug(("rdBuff=" + String(BufferRead) + " str_Buff=" + String(BufferIndex)));
    rdChar = (char)inputBuffer[BufferRead++];
    if (rdChar == '\n') {
      //some neasty code .. to be executed by the G code...
      g_cmd.toUpperCase();
      g_cmd.trim();
      Println(g_cmd);
      int ndIndex = 0;
      int stIndex = 0;
      while (ndIndex >= 0) { //while second Letter is stil there to read...
        stIndex = find_next_letter_in_string(g_cmd, ndIndex);
        ndIndex = find_next_letter_in_string(g_cmd, stIndex + 1);
        String CMD = read_sub_string(g_cmd, stIndex, ndIndex);
        if (CMD == "$") {
          //specil case detected...read till the end of g_cmd
          //to do... $ can be as $G,$H, $12=123, $RST=$, $RST=* ...
          //should be something like: ...read g_cmd till '\0' or '=' or '\n' is found
          while (g_cmd[ndIndex] != '\0') {
            CMD += g_cmd[ndIndex++];
          }
          ndIndex = -1;
        }

        if (       CMD == "G0" ||
                   CMD == "G1" ||
                   CMD == "G10") {
          motion_mode = CMD;
        } else if (CMD == "G90" ||
                   CMD == "G91") {
          distance_mode = CMD;
        } else if (CMD == "M3" ||
                   CMD == "M4") {
          spindle_state = CMD;
          set_spindle();  
        } else if (CMD == "M5") {
          spindle_state = CMD;
          set_spindle();
        } else if (CMD == "$G") {
          view_parser_state();
        } else if (CMD == "$H") {
          find_home_position(); 
        } else if (CMD.indexOf('X') == 0) {
          CMD = read_sub_string(g_cmd, stIndex + 1, ndIndex);
          XYZ[0] = CMD.toFloat(); new_xyz[0] = true;
          if (debug) Debug(("XYZ[0]=" + String(XYZ[0])));
        } else if (CMD.indexOf('Y') == 0) {
          CMD = read_sub_string(g_cmd, stIndex + 1, ndIndex);
          XYZ[1] = CMD.toFloat(); new_xyz[1] = true;
          if (debug) Debug(("XYZ[1]=" + String(XYZ[1])));
        } else if (CMD.indexOf('Z') == 0) {
          CMD = read_sub_string(g_cmd, stIndex + 1, ndIndex);
          XYZ[2] = CMD.toFloat(); new_xyz[2] = true;
          if (debug) Debug(("XYZ[2]=" + String(XYZ[2])));
        }
      }
      g_code_handler();
      g_cmd = "";
      Println("ok"); //When g_cmd is completed, we must respond with 'ok'  aaaaaaaa... not !! OK
    }
    else {
      g_cmd += rdChar;// reading commands ... until '\n' is found...
    }
  }
}

void g_code_handler() {
  //if we have to move?
  if ((motion_mode == "G0" || motion_mode == "G1")){// && (new_xyz)) { //do the line move
    while (active_state != "Idle"){} //če je status različen od IDLE naj se ne premika
    move_x_y_z();
  } else if (motion_mode == "G10") {//do the x,y,z coordinate settings
    for (byte i = 0; i < 3; i++) {
      if (XYZ[i] != WPos[i]) {
        WPos[i] = XYZ[i];
      }
      motion_mode = "G0";
    }
  }
}

/*   SerialEvent occurs whenever a new data comes in the
     hardware serial RX.Multiple bytes of data may be available.
     And some serial functions...
*/
ISR(USART_RX_vect) {
//  digitalWrite(13, !digitalRead(13)); //test for incomming data :)
  inChar = UDR0;
  // save the gbrl data ... or respond immediately ...
  switch (inChar) {
    case '?': if (!realtime_cmd_busy)view_current_status(); break; //'?' - RealTime command - CURRENT STATUS - it can be sent at any time...
    case '!': active_state="Hold";break;//active_state_hold = !(active_state_hold); if (active_state_hold){active_state="Hold";}else{active_state="Idle";}break;
    case '~': active_state="Idle";break; 
    case 24: asm volatile ("  jmp 0"); //Ctrl-x
    case ' ': break; //we do not want to save SPACEs...
    default : // Any other character is stored into Buffer and is read when we have time
      inputBuffer[BufferIndex++] = inChar;
      //if (debug) Debug(String(int(inChar)));
  }
}
void Print(String str) {
  byte i = 0;
  while (str[i] != '\0') {
    while (!(UCSR0A & (1 << UDRE0))) {} //Transmit is busy .. wait
    UDR0 = str[i];
    i++;
  }
}
void Println(String str) {
  byte i = 0;
  str += '\n';
  while (str[i] != '\0') {
    while (!(UCSR0A & (1 << UDRE0))) {} //UART Tx is busy .. wait for it...
    UDR0 = str[i];                      //Transmit char
    i++;                                //next str index
  }
}

void Debug(String str) {
  Print("@ ");
  Println(str);
}
void UART_config() {
  UCSR0A = B00000010; // [U2X]transm. speed 2x (zaradi manjšega err)
  UCSR0B = B10011000; // [RxCi]-interrupt(en),[Rx(en)], [Tx(en)]
  UCSR0C = B00000110; // [UCSZn1][UCSZn1]- 8 bit communication
  UBRR0H = 0;        // baud =
  UBRR0L = 16;       // 115200 , err = 2.1%
}
