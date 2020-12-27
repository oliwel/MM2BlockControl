#include "MaerklinMotorola.h"

#define INPUT_PIN 2

#define SENSOR_DECAY 0.01
#define SENSOR_DECAY_CLOSE 0.1
#define SENSOR_DECAY_OPEN 0.01

#define CONTACT_COUNT 3

volatile MaerklinMotorola mm(INPUT_PIN);
volatile bool hasNewData;

enum ContactMode {
  ContactMode_Forward,
  ContactMode_Detect,
  ContactMode_Brake,
  ContactMode_MoveOut,
};

struct Contact {
  uint8_t pin_gbm; // number of the input pin for this contact
  uint8_t pin_brake; // number of the output pin to run the relay
  ContactMode mode;
  float lastValue;
  unsigned long lastUpdate;
  bool closed;
  uint8_t portid;
};

Contact Contacts[CONTACT_COUNT];
uint8_t PortId[] = { 42, 43, 21 };

void DEBUG(String message) {
  Serial.println( String(millis()) + " [DEB]: " + message);
}

void INFO(String message) {
  Serial.println( String(millis()) + " [INF]: " + message);
}

void setBrake() {

}

void setContactStatus (int ii, bool Status) {
  if (Status) {
    if (Contacts[ii].mode != ContactMode_MoveOut) {
      Contacts[ii].mode = ContactMode_MoveOut;
      INFO("Start move out " + String(ii) );
    } else {
      DEBUG("Ignore move out on " + String(ii) );
    }
  } else {
    if (Contacts[ii].mode != ContactMode_Detect) {           
      INFO("Reset block " + String(ii));
      Contacts[ii].mode = ContactMode_Detect;
      digitalWrite(3, LOW);
    } else {
      DEBUG("Ignore reset on " + String(ii) );
    }
  }
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  bitSet(TCCR1B, CS12);  // 256 prescaler
  bitSet(TIMSK1, TOIE1); // timer overflow interrupt
  
  Serial.begin(115200);
  Serial.flush();
  Serial.println("Starting up");

  for (int ii = 0; ii<CONTACT_COUNT; ii++) {        
    Contacts[ii] = Contact{ 
      static_cast<uint8_t>(A0 + ii), 
      static_cast<uint8_t>(3 + ii), 
      ContactMode_Detect, 
      HIGH, 
      0, 
      false, 
      PortId[ii] 
    };
    pinMode(Contacts[ii].pin_brake, OUTPUT);
    digitalWrite(Contacts[ii].pin_brake, LOW);
    pinMode(Contacts[ii].pin_gbm, INPUT_PULLUP);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  mm.Parse(); 
  MaerklinMotorolaData* Data = mm.GetData();
  if(Data) {
    /*    
      Serial.print("Address: "); Serial.print(Data->Address);
      Serial.print(" -  Function: "); Serial.print(Data->Function);
      Serial.print(" -  Stop: "); Serial.print(Data->Stop);
      Serial.print(" -  ChangeDir: "); Serial.print(Data->ChangeDir);
      Serial.print(" -  Speed: "); Serial.print(Data->Speed);
      Serial.print(" -  Magnet: " + String(Data->IsMagnet ? "yes" : "no"));      
      Serial.println();
    */
    // Check for aux decoder / Data->IsMagnet
    bool isOn;
    int8_t portid;

    if(Data->IsMagnet) {
      hasNewData = true;
      // false for "turn off all" command (called after swichting time has passed)
      if(Data->MagnetState) {
        DEBUG("Command " + String(Data->SubAddress) + " for " + String(Data->Address));

        // Bit0 = green/red
        isOn = Data->SubAddress & 1;

        // Bit 1+2 = decoder number     
        // SubAddress 0 = Decoder #1 - green
        // SubAddress 7 = Decoder #4 - red        
        // Address 1 = Decoders 1 to 4, SubAdress Bits 2+3 are decoder offset
        portid = ((Data->Address - 1) * 4) + (Data->SubAddress >> 1) + 1;
          
        DEBUG("Decoder # " + String(Data->PortAddress) + " Switch " + (Data->DecoderState));

        for (int ii = 0; ii<CONTACT_COUNT; ii++) {
            if (PortId[ii] == portid) {
              setContactStatus(ii, isOn);
              break;                
            }
        }
      } else {
        // no use case for this
        //Serial.println("Reset for " + String(Data->Address));
      }
    }
  }

  for (int ii = 0; ii<CONTACT_COUNT; ii++) {  
    Contact* me = &Contacts[ii];    
    float decay = me->closed ? SENSOR_DECAY_OPEN : SENSOR_DECAY_CLOSE;
    me->lastValue = me->lastValue*(1.0-decay)+digitalRead(me->pin_gbm)*decay;

    if (me->lastValue < 0.1 && !me->closed) {
      INFO("State change to closed for " + String(ii));
      me->closed = true;      
      me->lastUpdate = millis();
      if (me->mode == ContactMode_Detect)  {
        Serial.println(String(millis()) + ": Brake " + String(ii));
        me->mode = ContactMode_Brake;
        digitalWrite(me->pin_brake, HIGH);
      }
    } else if (me->lastValue > 0.9 && me->closed) {
      INFO("State change to open for " + String(ii));
      me->closed = false;      
      me->lastUpdate = millis();
    }
  }  
}

void isr() {
  mm.PinChange();
}

ISR(TIMER1_OVF_vect) {
  if (hasNewData) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    hasNewData = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}