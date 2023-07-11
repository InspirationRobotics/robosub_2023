#include<Servo.h>

Servo s1; 
bool load;
bool shot;

void setup() {
  load = true;
  shot = false;

  Serial.begin(9600);
  
  //load
  s1.write(30);
  s1.attach(9);
  Serial.println("Is it safe to shoot? Enter y if yes");

}

void loop() {
  char rx_byte;
  char rx_byte2;

  if(Serial.available()>0) {
    
    rx_byte = Serial.read();

    //shoot
    if(rx_byte == 'y' && load == true && shot == false){
      Serial.flush();
      Serial.println("\nIt is safe to shoot, enter s to shoot");
      while(rx_byte == 'y'){
        rx_byte2 = Serial.read();
        if(rx_byte2 == 's') {
          s1.write(0);
          shot = true;
          load = false;
          rx_byte = 'k';
          Serial.println("\nShot");
          Serial.println("Enter l to load");
        }
      }
    }
    
    //load
    else if(rx_byte == 'l' && load == false && shot == true) {
      s1.write(30);
      shot = false;
      load = true;
      Serial.println("\nLoaded");
      Serial.println("Is it safe to shoot? Enter y if yes");
    }   
  }
}
