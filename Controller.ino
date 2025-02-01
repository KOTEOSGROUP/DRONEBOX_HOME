#include <Bluepad32.h>
#include "BTS7960.h"
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

  const uint8_t EN =32;
  const uint8_t EN2 =23;
  const uint8_t EN3 =5;
  const uint8_t EN4 =4;
  
  const uint8_t L_PWM =13;//motor 1 lpwm
  const uint8_t R_PWM =26;//motor 1 rpwm
  
  const uint8_t L_PWM2 =25;//motor 2 lpwm
  const uint8_t R_PWM2 =27;//motor 2  rpwm
   
  const uint8_t L_PWM3 =22;//motor 3 lpwm
  const uint8_t R_PWM3 =21;//motor 3 rpwm
   
  const uint8_t L_PWM4 =18;//motor 4 lpwm
  const uint8_t R_PWM4 =19;//motor 4 rpwm
  
    BTS7960 motorController(EN,EN,L_PWM,R_PWM);//contorlador motor 1
    BTS7960 motorController2(EN2,EN2,L_PWM2,R_PWM2);//contorlador motor 2
    BTS7960 motorController3(EN3,EN3,L_PWM3,R_PWM3);//contorlador motor 3
    BTS7960 motorController4(EN4,EN4,L_PWM4,R_PWM4);//contorlador motor 4
  
// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

double moveMotors(double x, double y,double velocity){
  motorController.Enable();
  motorController2.Enable();
  motorController3.Enable();
  motorController4.Enable();
  int en = map(x, -512, 512, 0, 180);
  int en2 = map(y, -512, 512, 0, 180);
 int v=int(velocity);
    if(v>=200){v=200;} 
  //derecha
  Serial.println(v);
  int dir =0;
  if(en>=170){
      motorController.TurnRight(v);
      motorController2.TurnRight(v); 
      motorController3.TurnRight(v);
      motorController4.TurnRight(v); 
      Serial.println(v);
      Serial.println("derecha");
          if(velocity<200){
         velocity=pow(velocity,1.15);
         }
      dir=1;
    }
    //izquierda
  if(en<=10){
     motorController.TurnLeft(v);
     motorController2.TurnLeft(v);
     motorController3.TurnLeft(v);
     motorController4.TurnLeft(v);
     Serial.println(velocity);
     Serial.println("izquierda");
         if(velocity<199){
         velocity=pow(velocity,1.15);
         }
     dir=2;
  }
  //atras
    if(en2>=170){
      motorController.TurnLeft(v);
      motorController2.TurnRight(v);
      motorController3.TurnLeft(v);
      motorController4.TurnRight(v);
      Serial.println(velocity);
      Serial.println("atras");
        if(velocity<199){
         velocity=pow(velocity,1.15);
         }
      dir=3;
    }
   //adelante
  if(en2<=10){
     motorController.TurnRight(v);
     motorController2.TurnLeft(v);
     motorController3.TurnRight(v);
     motorController4.TurnLeft(v);
     Serial.println(velocity);
     Serial.println("adelante");
          if(velocity<200){
         velocity=pow(velocity,1.15);
         }
     dir=4;
  }
//quierto
  if(en<=170 && en>=10 && en2>=10 && en2<=170){
       if(velocity>2){
     velocity=pow(velocity,0.99);
     v=int(velocity);
     }
   switch (dir){
    case 0:
     motorController.TurnRight(0);
     motorController2.TurnLeft(0);
     motorController3.TurnRight(0);
     motorController4.TurnLeft(0);
   break;
   case 1:
    motorController.TurnRight(v);
    motorController2.TurnRight(v);
    motorController3.TurnRight(v);
    motorController4.TurnRight(v);
    if(v<=2){
      dir=0;
      } 
   break;
   case 2:
    motorController.TurnLeft(v);
    motorController2.TurnLeft(v);
    motorController3.TurnLeft(v);
    motorController4.TurnLeft(v);
    if(v<=2){
      dir=0;
      } 
   break;
   case 3:
    motorController.TurnLeft(v);
    motorController2.TurnRight(v);
    motorController3.TurnLeft(v);
    motorController4.TurnRight(v);
       if(v<=2){
      dir=0;
      } 
   break;
   case 4:
    motorController.TurnRight(v);
    motorController2.TurnLeft(v);
    motorController3.TurnRight(v);
    motorController4.TurnLeft(v);
       if(v<=2){
      dir=0;
      } 
   break;
   default:
   break;
   }

     Serial.println(v);
     Serial.println("quieto");
     
  }

  return velocity;
  }
int Stop(){
   motorController.Stop();
   motorController2.Stop();
   motorController3.Stop();
   motorController4.Stop();
   Serial.println("stop");
   return 2;
}
double processControllers(double v) {
  double vel=0;

    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
                vel=moveMotors(myController->axisX(),myController->axisY(),v);
                if(myController->buttons()==0xc0){
                  vel=Stop();
                }
                
     
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
   if(vel>=200){vel=200;} 
   Serial.println(vel); 
   return vel;          
}



double vel=2;
// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

  pinMode(32,OUTPUT);//en

  pinMode(12,OUTPUT);//motor 1 lpwm
  pinMode(13,OUTPUT);//motor 1 rpwm
  pinMode(25,OUTPUT);//motor 2 lpwm
  pinMode(27,OUTPUT);//motor 2 rpwm

    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
       digitalWrite(32,HIGH);
    bool dataUpdated = BP32.update();
      if(dataUpdated)
         vel= processControllers(vel);
         if(vel<=2){
          vel=2;
          }
      
    
    
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    delay(150);
}
