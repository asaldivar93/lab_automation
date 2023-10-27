String inputString = "";
String slaveString = "";
String ADDRESS = "mas";

float rpm_setpoint[] = {0, 0, 0, 0, 0, 0, 1500, 1500};
float rpm[8];
float tmp_setpoint[] = {0, 0, 0, 0, 0, 0, 0, 0};
float tmp[8];
float sample_time = 1000000;
boolean newCommand = false;
boolean comm_confirmed = false;

void update_rpm_sp(float rpm_setpoint[8]){
  String command = "stir 1";
  for(byte i = 0; i < 8; i++){
    command = command + "," + (String) rpm_setpoint[i];
  }
  command = command + ",!";
  Serial2.println(command);
  while(!comm_confirmed){
    parseSerial2();
    int slave_ans = slaveString.toInt();
    if( slave_ans == 115){
      comm_confirmed = true;
    }
  }
  slaveString = "";
  comm_confirmed = false;
  Serial.println("rpm_sp updated");
}

void check_rpm_board(){
  String command = "stir 2,!";
  Serial2.println(command);
  while(!comm_confirmed){
    parseSerial2();
    int slave_ans = slaveString.toInt();
    if( slave_ans == 115){
      comm_confirmed = true;
    }
  }
  slaveString = "";
  comm_confirmed = false;
  Serial.println("Stirrer board : ok");
}

void update_tmp_sp(float tmp_setpoint[8]){
  String command = "tmp 1";
  for(byte i = 0; i < 8; i++){
    command = command + "," + (String) tmp_setpoint[i];
  }
  command = command + ",!";
  Serial2.println(command);
  while(!comm_confirmed){
    parseSerial2();
    int slave_ans = slaveString.toInt();
    if( slave_ans == 115){
      comm_confirmed = true;
    }
  }
  slaveString = "";
  comm_confirmed = false;
  Serial.println("tmp_sp updated");
}

void check_tmp_board(){
  String command = "tmp 2,!";
  Serial2.println(command);
  while(!comm_confirmed){
    parseSerial2();
    int slave_ans = slaveString.toInt();
    if( slave_ans == 115){
      comm_confirmed = true;
    }
  }
  slaveString = "";
  comm_confirmed = false;
  Serial.println("Temperature board : ok");
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  delay(500);
  Serial.println("\nInitializing Boards");
  check_rpm_board();
  update_rpm_sp(rpm_setpoint);
  check_tmp_board();
  update_tmp_sp(tmp_setpoint);
}

void loop() {
  parseSerial();
  parseString(inputString);
  inputString = "";
}

void parseSerial(void){
  while(Serial.available()){
    char inChar = char(Serial.read());
    inputString += inChar;
    if(inChar == '!'){
      newCommand = true;
      break;
    }
  }
}

void parseSerial2(void){
  while(Serial2.available()){
    char inChar = char(Serial2.read());
    slaveString += inChar;
    if(inChar == '!'){
      break;
    }
  }
}

void parseString(String inputString){
  if(newCommand){
    String ADDR = inputString.substring(0, inputString.indexOf(' '));
    
    if(ADDR == ADDRESS){
      int firstcomma = inputString.indexOf(',');
      String cmd = inputString.substring(inputString.indexOf(' '), firstcomma);
      int command = cmd.toInt();

      if(command == 1){
        Serial2.println("stir 3,!");
        while(!comm_confirmed){
          parseSerial2();
          String ADDR = slaveString.substring(0, inputString.indexOf(' '));
    
          int firstcomma = slaveString.indexOf(',');
          int lastcomma = firstcomma;
          for (byte i = 0; i < 8; i++){
            int nextcomma = slaveString.indexOf(',', lastcomma + 1);
            String val = slaveString.substring(lastcomma + 1, nextcomma);
            float value = val.toFloat();
            rpm[i] = value;
            lastcomma = nextcomma;
          }
          int nextcomma = slaveString.indexOf(',', lastcomma + 1);
          String val = slaveString.substring(lastcomma + 1, nextcomma);
          int value = val.toInt();
          
          Serial.println(value);
          if(value == 115){
            comm_confirmed=true;
          }
        }
        comm_confirmed = false;
        slaveString = "";

        Serial2.println("tmp 3,!");
        while(!comm_confirmed){
          parseSerial2();
          String ADDR = slaveString.substring(0, inputString.indexOf(' '));
    
          int firstcomma = slaveString.indexOf(',');
          int lastcomma = firstcomma;
          for (byte i = 0; i < 8; i++){
            int nextcomma = slaveString.indexOf(',', lastcomma + 1);
            String val = slaveString.substring(lastcomma + 1, nextcomma);
            float value = val.toFloat();
            tmp[i] = value;
            lastcomma = nextcomma;
          }
          int nextcomma = slaveString.indexOf(',', lastcomma + 1);
          String val = slaveString.substring(lastcomma + 1, nextcomma);
          int value = val.toInt();
          
          Serial.println(value);
          if(value == 115){
            comm_confirmed=true;
          }
        }
          
        Serial.println(tmp[7]);
        comm_confirmed = false;
        slaveString = "";
      }
    }
    
    inputString = "";
    newCommand = false;
  }
}
