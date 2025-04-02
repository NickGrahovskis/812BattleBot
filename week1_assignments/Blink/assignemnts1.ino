/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

// the setup function runs once when you press reset or power the board
const int btnpin=9;
const int btnpin1=10;
const int btnpin2 = 8;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(btnpin, INPUT_PULLUP);
  pinMode(btnpin1, INPUT_PULLUP);
  pinMode(btnpin2, INPUT_PULLUP);
  

}

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);  
      delay(1000);                      
      digitalWrite(LED_BUILTIN, LOW);   
      delay(1000); 
      
    while(digitalRead(btnpin1) == LOW){
      digitalWrite(LED_BUILTIN, HIGH);  
      delay(500);                      
      digitalWrite(LED_BUILTIN, LOW);   
      delay(500); 
    }
    
    
      while(digitalRead(btnpin2) == LOW){
        digitalWrite(LED_BUILTIN, HIGH);  
        delay(2000);                      
        digitalWrite(LED_BUILTIN, LOW);   
        delay(2000); 
      }

    


    
    
  
}
