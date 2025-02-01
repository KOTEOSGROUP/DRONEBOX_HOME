#include <Bluepad32.h>
#include "BTS7960.h"
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
  //velocidad al encender el dronebox
  int v=0;

  const uint8_t EN1 =32;
  const uint8_t EN2 =23;
  const uint8_t EN3 =5;
  const uint8_t EN4 =4;
  
  const uint8_t L_PWM1 =13;//motor 1 lpwm
  const uint8_t R_PWM1 =26;//motor 1 rpwm
  
  const uint8_t L_PWM2 =25;//motor 2 lpwm
  const uint8_t R_PWM2 =27;//motor 2 rpwm
   
  const uint8_t L_PWM3 =22;//motor 3 lpwm
  const uint8_t R_PWM3 =21;//motor 3 rpwm
   
  const uint8_t L_PWM4 =18;//motor 4 lpwm
  const uint8_t R_PWM4 =19;//motor 4 rpwm
  //Los motores del lado derecho son 1 y 3, los del izquierdo son 2 y 4
  BTS7960 motor1(EN1,EN1,L_PWM1,R_PWM1);//contorlador motor 1
  BTS7960 motor2(EN2,EN2,L_PWM2,R_PWM2);//contorlador motor 2
  BTS7960 motor3(EN3,EN3,L_PWM3,R_PWM3);//contorlador motor 3
  BTS7960 motor4(EN4,EN4,L_PWM4,R_PWM4);//contorlador motor 4

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

// a partir de aqui no se trata de la coneccion con el control.

//freno absoluto o de emergencia.
void frenof(){
   motor1.Stop();
   motor2.Stop();
   motor3.Stop();
   motor4.Stop();
   Serial.println("stop");
}

//freno gradual.
void freno(){
  if (v>0){
    v = v-10;
  }
  Serial.println("freno");

}

//aceleracion.
void accel(){
  if (v<200){
    v = v+10;
  }
  Serial.println("aceleracion");
 
}

//avance y giro
int inputr(double r){
  int r1 = map(r, -512, 512, 0, 180);
  int r2;
  if(r1>=170){
    r2=1;
    }
   if(r1<=10){
     r2=-1;
   }
   if(r1<=170&&r1>=10){
    r2=0;
   }
    
  return r2;
}

//procesamiento de direccion para los motores.
void movimiento(int x,int y){
     if (x == -1 && y == 0){
        motor1.TurnRight(v);
        motor2.TurnRight(v);
        motor3.TurnRight(v);
        motor4.TurnRight(v);
        Serial.println("izquierda");
     }
     if (x == 1 && y == 0){
        motor1.TurnLeft(v);
        motor2.TurnLeft(v);
        motor3.TurnLeft(v);
        motor4.TurnLeft(v);
        Serial.println("derecha");
     }
     if (y == -1 && x == 0){
        motor1.TurnLeft(v);
        motor2.TurnRight(v);
        motor3.TurnLeft(v);
        motor4.TurnRight(v);
        Serial.println("adelante");        
        
     }
     if (y == 1 && x == 0){
        motor1.TurnRight(v);
        motor2.TurnLeft(v);
        motor3.TurnRight(v);
        motor4.TurnLeft(v);
        Serial.println("atras");
     }
     

     Serial.println("x =" + x);
     Serial.println("y =" + y);
     Serial.println("v =" + v);


}

void inputs(){
  double x0,y0;
  int x,y;
  for(auto myController : myControllers){
    if (myController && myController->isConnected() && myController->hasData()&& myController->isGamepad()){
      //processGamepad(myController);
      //control de frenos.
      if(myController->buttons()==0x0040){
        freno();
      }
      //control de aceleracion.
      if(myController->buttons()==0x0080){
        accel();
      }
      //Freno de emergencia.
      if(myController->buttons()==0x0030){
        frenof();
        v=0;
      }
      //extraer la direccion.
      x0 = myController->axisX();
      y0 = myController->axisY();
      x = inputr(x0);
      y = inputr(y0);
      Serial.println(x);
      Serial.println(y);
      //generar el movimiento.
      movimiento(x,y);
      
    }
  }

}

void setup() {
  Serial.begin(115200);
  
  pinMode(32,OUTPUT);//en
  pinMode(12,OUTPUT);//motor 1 lpwm
  pinMode(13,OUTPUT);//motor 1 rpwm
  pinMode(25,OUTPUT);//motor 2 lpwm
  pinMode(27,OUTPUT);//motor 2 rpwm

  //preparacion de control blkuetooth.
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  //llamar al chequeo de coneccion del motor.
  BP32.setup(&onConnectedController, &onDisconnectedController);
  //olvidar todos los dispositibvos en startup.
  BP32.forgetBluetoothKeys();
  //deshabilita funciones extra de controles.
  BP32.enableVirtualDevice(false);
  
  //habilitar todos los motores.
  motor1.Enable();
  motor2.Enable();
  motor3.Enable();
  motor4.Enable();
}

void loop() {
  //habilitar los motores.
  digitalWrite(32,HIGH);
  //updatear los datos del control.
  bool dataUpdated = BP32.update();

  if(dataUpdated){
   inputs();
  }
  delay(150);
}
