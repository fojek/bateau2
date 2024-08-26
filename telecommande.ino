#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role
//bool role - true;

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

struct {
  uint8_t dir1;
  uint8_t vitesse1;
  uint8_t dir2;
  uint8_t vitesse2;
  uint8_t checksum;
} payload;

uint8_t direction;

RF24 radio(CE_PIN, CSN_PIN);

bool ledstate = false;

void calcVitesses(uint8_t vitesse, uint8_t direction) {

  //uint8_t facteur = 100;
  uint8_t max = 255;

  if (vitesse >= 137) {
    // Avance
    Serial.println("Direction : avance.");
    payload.vitesse1 = map(vitesse, 127, 255, 0, max);
    payload.vitesse2 = payload.vitesse1;
    payload.dir1 = 1;
    payload.dir2 = 1;
  } else if (vitesse <= 117) {
    // Recule
    Serial.println("Direction : reculons.");
    payload.vitesse1 = map(vitesse, 0, 127, max, 0);
    payload.vitesse2 = payload.vitesse1;
    payload.dir1 = 0;
    payload.dir2 = 0;
  } else {
    Serial.println("Direction : Arret.");
    payload.vitesse1 = 0;
    payload.vitesse2 = 0;
    payload.dir1 = 2;
    payload.dir2 = 2;
  }

  if (direction > 137) {
    // moteur droit = arret
    payload.dir2 = 2;
    payload.vitesse2 = 0;
  } else if (direction < 117) {
    // moteur gauche = arret
    payload.dir1 = 2;
    payload.vitesse1 = 0;
  } else {
    // Tout droit
  }
}

void setup() {

  role = radioNumber;

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

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

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  pinMode(A0, OUTPUT);

  pinMode(A5, INPUT_PULLUP);
}

void loop() {

  direction = analogRead(A6) / 4;
  calcVitesses(analogRead(A7) / 4, direction);
  payload.checksum = payload.vitesse1 + payload.vitesse2 + payload.dir1 + payload.dir2;

  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                  // start the timer
    bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
    unsigned long end_timer = micros();                    // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.println(F(" us. Sent: "));
      Serial.println(payload.vitesse1);  // print payload sent
      //Serial.println(payload.vitesse2);    // print payload sent
      Serial.println(payload.dir1);  // print payload sent
      //Serial.println(payload.dir2);    // print payload sent

      //Serial.println(payload.direction);  // print payload sent
      //Serial.println(payload.bouton);  // print payload sent

      if (ledstate) {
        digitalWrite(A0, HIGH);
        ledstate = 0;
      } else {
        digitalWrite(A0, LOW);
        ledstate = 1;
      }
      //payload += 0.01;          // increment float payload
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    // to make this example readable in the serial monitor
    delay(75);  // slow transmissions down by 1 second

  } /* else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);  // print the payload's value
    }
  }  // role*/
}
