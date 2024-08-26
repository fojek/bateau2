#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

#define ENA 3
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 A0
#define IN4 A1

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role
//bool role - true;

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

typedef struct {
  uint8_t dir1;
  uint8_t vitesse1;
  uint8_t dir2;
  uint8_t vitesse2;
  uint8_t checksum;
} Payload;

Payload payload, payloadBuffer;

typedef struct Moteur {
  uint8_t num;
  uint8_t enablePin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t vitesse;
  uint8_t direction;
} Moteur;

Moteur moteurs[2];

void Moteur_init(Moteur* mot, uint8_t num, uint8_t en, uint8_t in1, uint8_t in2) {
  mot->enablePin = en;
  mot->in1Pin = in1;
  mot->in2Pin = in2;

  mot->num = num;

  // Moteur 1
  pinMode(mot->enablePin, OUTPUT);
  pinMode(mot->in1Pin, OUTPUT);
  pinMode(mot->in2Pin, OUTPUT);

  // Etat initial
  digitalWrite(mot->enablePin, LOW);
  digitalWrite(mot->in1Pin, LOW);
  digitalWrite(mot->in2Pin, LOW);

  mot->direction = 0;
  mot->vitesse = 0;

  Serial.print("Moteur cree, ");
  Serial.print(mot->enablePin);
  Serial.print(", ");
  Serial.print(mot->in1Pin);
  Serial.print(", ");
  Serial.println(mot->in2Pin);
}

void Moteur_setVitesse(Moteur* mot, uint8_t vitesse, uint8_t dir) {

  if (dir == 0) {
    digitalWrite(mot->in1Pin, LOW);
    digitalWrite(mot->in2Pin, HIGH);
  } else if (dir == 1) {
    digitalWrite(mot->in1Pin, HIGH);
    digitalWrite(mot->in2Pin, LOW);
  } else {
    digitalWrite(mot->in1Pin, LOW);
    digitalWrite(mot->in2Pin, LOW);
  }

  analogWrite(mot->enablePin, vitesse);

  if (vitesse < 20) {
    digitalWrite(mot->in1Pin, LOW);
    digitalWrite(mot->in2Pin, LOW);
    digitalWrite(mot->enablePin, LOW);
  }
}

RF24 radio(CE_PIN, CSN_PIN);

void setup() {

  role = radioNumber;

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  Serial.print("Commence.");

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }

  Moteur_init(&moteurs[0], 0, ENA, IN1, IN2);
  Moteur_init(&moteurs[1], 1, ENB, IN3, IN4);

  payload.vitesse1 = 0;
}

long int lastUpdate = 0;

void updatePayload() {
  if (!role) {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payloadBuffer, bytes);       // fetch payload from FIFO
      /*Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      //Serial.println(payload);  // print the payload's value
      Serial.println(payload.vitesse1);  // print payload sent
      Serial.println(payload.dir1);  // print payload sent
      //Serial.println(payload.bouton);  // print payload sent*/

      uint8_t checksum = payloadBuffer.vitesse1 + payloadBuffer.vitesse2 + payloadBuffer.dir1 + payloadBuffer.dir2;

      if (checksum == payloadBuffer.checksum) {
        // Paquet valide
        payload.vitesse1 = payloadBuffer.vitesse1;
        payload.vitesse2 = payloadBuffer.vitesse2;
        payload.dir1 = payloadBuffer.dir1;
        payload.dir2 = payloadBuffer.dir2;
        lastUpdate = millis();
      } else {
        Serial.print("Checksum invalide : attendu ");
        Serial.print(payloadBuffer.checksum);
        Serial.print(" etait ");
        Serial.println(checksum);
      }
    }

    if (millis() > lastUpdate + 1000) {
      payload.vitesse1 = 0;
      payload.vitesse2 = 0;
      payload.dir1 = 2;
      payload.dir2 = 2;
    }
  }  // role
}

void loop() {
  updatePayload();

  Moteur_setVitesse(&moteurs[0], payload.vitesse1, payload.dir1);
  Moteur_setVitesse(&moteurs[1], payload.vitesse2, payload.dir2);
}
