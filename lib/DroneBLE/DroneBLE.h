#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"

using namespace std;

class DroneBLE
{
private:
    BLEServer *bleServer;
    BLEService *bleService;
    BLECharacteristic *bleCharacteristic;
    BLEAdvertising *bleAdvertising;
public:
    DroneBLE() {};
    ~DroneBLE() {};

    void setup();

    string getValue();
};
