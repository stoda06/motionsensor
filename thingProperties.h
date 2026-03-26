#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

const char DEVICE_LOGIN_NAME[] = "57954265-4629-4461-9129-ae5a89bddd85";
const char DEVICE_KEY[]        = SECRET_DEVICE_KEY;

// Read-only cloud variable — updated by sketch, not controllable from cloud
bool cloudRelayState;

void initProperties() {
  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

  ArduinoCloud.addProperty(cloudRelayState, READ, ON_CHANGE, NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_PASS);
