#include "MaerklinMotorola.h"

#define INPUT_PIN 2

#define SENSOR_DECAY 0.01
#define SENSOR_DECAY_CLOSE 0.1
#define SENSOR_DECAY_OPEN 0.01

#define CONTACT_COUNT 6

volatile MaerklinMotorola mm(INPUT_PIN);
volatile bool hasNewData;
char blinkCount = 0;


enum ContactMode {
  ContactMode_Forward, // track power on, do nothing
  ContactMode_MoveOut, // train was released by user, restore track power, monitor brake contact
  ContactMode_Follow, // track power controlled by parent block
  ContactMode_Detect, // track power on, wait for brake contact
  ContactMode_Brake, // break contact has fired, remove track power
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
// Move to EE Storage later
// Non-Zero value is an address for the head section
// Zero values are appended as childs to it
uint8_t PortId[] = { 112, 0, 0, 113, 0, 0 };

void DEBUG(String message) {
  Serial.println( String(millis()) + " [DEB]: " + message);
}

void INFO(String message) {
  Serial.println( String(millis()) + " [INF]: " + message);
}


Contact* getNextBlock(int ii) {
  if (ii >= CONTACT_COUNT) {
    return;
  }
  // check if this is a head block
  if (PortId[ii] != 0) { 
    return;
  }
  // not a head block
  return &Contacts[ii];
}
 
void startMoveOut(int ii) {
  Contact* me = &Contacts[ii];
  if (me->mode != ContactMode_MoveOut) {
      me->mode = ContactMode_MoveOut;
      // Remove Brake-on-DC Relay
      digitalWrite(me->pin_brake, LOW);
      INFO("Start move out " + String(ii));

      while (me = getNextBlock(++ii)) {
        if (me->mode != ContactMode_Follow) {
          return;
        }
        DEBUG("Power on Follow-Up Block " + String(ii));
        digitalWrite(me->pin_brake, HIGH);
      }
    } else {
      DEBUG("Ignore move out on " + String(me->portid) );
    }
}

void triggerContactChange(int ii, bool ContactState) {
  Contact* me = &Contacts[ii];
  me->closed = ContactState;
  me->lastUpdate = millis();

  // Start Brake
  if (me->mode == ContactMode_Detect && me->closed)  {
    INFO("Start Brake on " + String(ii));
    me->mode = ContactMode_Brake;
    digitalWrite(me->pin_brake, HIGH);
    
    while (me = getNextBlock(++ii)) {
      if (me->mode != ContactMode_Forward) {
        return;
      }
      if (me->closed) {
        DEBUG("Set Follow-Up Brake on " + String(ii));
        me->mode = ContactMode_Follow;
        digitalWrite(me->pin_brake, HIGH);
      } else {
        DEBUG("Set Detect on Follow-Up Block " + String(ii));
        me->mode = ContactMode_Detect;
        return;
      }
    }
    
    return;
  }

  if (me->mode == ContactMode_MoveOut && !me->closed) {
    me->mode = ContactMode_Detect;
    INFO("MoveOut complete on " + String(ii));
    while (me = getNextBlock(++ii)) {
      if (!me->closed) {
        DEBUG("Release Follow-Up Block " + String(ii));
        me->mode = ContactMode_Forward;
      } else if (me->closed) {  
        DEBUG("Pull-to-front from Follow-Up Block " + String(ii));
        startMoveOut(ii);
        return;
      }
      return;
    }
  }
}


void resetContact (int ii) {  
  Contact* parent;
  while (Contact* me = getNextBlock(ii)) {
    // contact is open, assume detect
    if (!me->closed) {
      if (!parent || parent->closed) {
        INFO("Reset block to Detect " + String(ii));
        me->mode = ContactMode_Detect;
        digitalWrite(me->pin_brake, LOW);
      } else {
        INFO("Reset block to Forward " + String(ii));
        me->mode = ContactMode_Forward;
        digitalWrite(me->pin_brake, LOW);       
      }
    } else {
      digitalWrite(me->pin_brake, HIGH);
      if (me->mode != ContactMode_Follow || (parent && parent->mode == ContactMode_Detect)) {
        me->mode = ContactMode_Brake;
      }
    }
  }
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);

  // use timer1 to blink onboard LED when there is data on the bus
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
      (PortId[ii] > 0) ? ContactMode_Detect : ContactMode_Forward,
      // initial value to high / open, closed pins will trigger on startup
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

    if(Data->IsMagnet && Data->MagnetState) {
      hasNewData = true;
      DEBUG("Decoder # " + String(Data->PortAddress) + " Switch " + (Data->DecoderState));
      for (int ii = 0; ii<CONTACT_COUNT; ii++) {
        if (PortId[ii] == Data->PortAddress) {
          if (Data->DecoderState == MM2DecoderState_Green) {
            startMoveOut(ii);
          } else if (Data->DecoderState == MM2DecoderState_Red) {
            resetContact(ii);
          }
          break;                
        }
      }    
    }
  }
  

  for (int ii = 0; ii<CONTACT_COUNT; ii++) {  
    Contact* me = &Contacts[ii];
    float decay = me->closed ? SENSOR_DECAY_OPEN : SENSOR_DECAY_CLOSE;
    me->lastValue = me->lastValue*(1.0-decay)+digitalRead(me->pin_gbm)*decay;
    if (me->lastValue < 0.1 && !me->closed) {
      INFO("State change to closed for " + String(ii));      
      triggerContactChange(ii, true);
    } else if (me->lastValue > 0.9 && me->closed) {
      INFO("State change to open for " + String(ii));      
      triggerContactChange(ii, false);
    }
  }  

  // Debugging - move this to a dedicated LED per Port
  // and leave on board led for communication
  // Blink n times within a one second interval based on Contact state
  // 2: move out, 3: wait for train, 5 (flashing): brakes on
  char blinkCount = millis() / 200 % 10;
  if ( (blinkCount & 1) && ((blinkCount / 2) <= Contacts[0].mode)) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

}

void isr() {
  mm.PinChange();
}

ISR(TIMER1_OVF_vect) {
  /*if (hasNewData) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    hasNewData = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }*/
}
