#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Motor control pins
#define in1 15
#define in2 2
#define in3 4
#define in4 16
#define rev_LED 17
// BLE server name
#define DEVICE_NAME "ESP_CAR"

// Service UUID
#define SERVICE_UUID "000000ff-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ff01-0000-1000-8000-00805f9b34fb"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
void Stop();
void Forward();
void Left();
void Right();
void Reverse(); 
void Reverse_LED_ON();
void Reverse_LED_OFF();
// Callback class for handling server events
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Stop();
    // Restart advertising when disconnected
    BLEDevice::startAdvertising();
  }
};

// Callback class for handling characteristic events
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0)
    {
      if (value == "MOVE_FORWARD")
      {
        Forward();
        Serial.println("Forward");
      }
      else if (value == "STOP")
      {
        Stop();
        Serial.println("Stop");
      }
      else if (value == "MOVE_LEFT")
      {
        Left();
        Serial.println("Left");
      }
      else if (value == "MOVE_RIGHT")
      {
        Right();
        Serial.println("Right");
      }
      else if (value == "MOVE_REVERSE")
      {
        Reverse();
        Serial.println("Reverse");
      }
      else if(value == "REVERSE_LED_ON"){
        Reverse_LED_ON();
      }
      else if(value == "REVERSE_LED_OFF"){
        Reverse_LED_OFF();
      }
    }
  }
};

void Reverse()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Forward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Right()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(250);
  Stop();
}

void Left()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(250);
  Stop();
}

void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void Reverse_LED_ON(){
  digitalWrite(rev_LED, HIGH);
}
void Reverse_LED_OFF(){
  digitalWrite(rev_LED, LOW);
}

void setup()
{
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(rev_LED, OUTPUT);
  Stop();
  delay(2000);
  Left();
  delay(2000);
  Right();
  delay(2000);
  Forward();
  delay(2000);
  Stop();
  delay(2000);
  Reverse_LED_ON();
  delay(2000);
  Reverse();
  delay(2000);
  Reverse_LED_OFF();
  Stop();


  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);
  Serial.println("Device name: ");
  Serial.println(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println("BLE device ready for connections!");
}

void loop()
{
  // Add any continuous monitoring or control logic here
  delay(2000);
}