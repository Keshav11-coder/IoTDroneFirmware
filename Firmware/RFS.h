#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10

class RFRX {
  private:
    RF24 radio(CE_PIN, CSN_PIN);
    byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
    bool newData = false;
  public:
    char dataReceived[10]; // this must match dataToSend in the TX

    void configure(byte SlaveAddress[5]) {
      thisSlaveAddress = SlaveAddress;
      Serial.println("SimpleRx Starting");
      radio.begin();
      radio.setDataRate( RF24_250KBPS );
      radio.openReadingPipe(1, thisSlaveAddress);
      radio.startListening();
    }

    void StartLineData() {
      if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
      }
    }

    void ShowLineData() {
      if (newData == true) {
        Serial.print("Data received ");
        Serial.println(dataReceived);
        newData = false;
      }
    }
};

class RFTX {
  private:
    RF24 radio(CE_PIN, CSN_PIN);
    byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

    char dataToSend[10] = "Message 0";
    char txNum = '0';
  public:
    void configure(byte SlaveAddress[5]) {
      slaveAddress = SlaveAddress;
      Serial.println("SimpleTx Starting");
      radio.begin();
      radio.setDataRate( RF24_250KBPS );
      radio.setRetries(3, 5); // delay, count
      radio.openWritingPipe(slaveAddress);
    }

    void send() {
      bool rslt;
      rslt = radio.write( &dataToSend, sizeof(dataToSend) );
      // Always use sizeof() as it gives the size as the number of bytes.
      // For example if dataToSend was an int sizeof() would correctly return 2

      Serial.print("Data Sent ");
      Serial.print(dataToSend);
      if (rslt) {
        Serial.println("  Acknowledge received");
        updateMessage();
      }
      else {
        Serial.println("  Tx failed");
      }
    }

    void updateMessage() {
      // so you can see that new data is being sent
      txNum += 1;
      if (txNum > '9') {
        txNum = '0';
      }
      dataToSend[8] = txNum;
    }
};

RFTX mytx;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second

byte slaveAddr[5] = {'R', 'x', 'A', 'A', 'A'};

void setup() {
  Serial.begin(9600);
  mytx.configure(slaveAddr);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    mytx.send();
    prevMillis = millis();
  }
}
