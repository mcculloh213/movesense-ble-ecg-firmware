#include "PowerModesClient.h"
#include "movesense.h"

MOVESENSE_APPLICATION_STACKSIZE(1024)

MOVESENSE_PROVIDERS_BEGIN(1)

MOVESENSE_PROVIDER_DEF(PowerModesClient)

MOVESENSE_PROVIDERS_END(1)

MOVESENSE_FEATURES_BEGIN()
SERIAL_COMMUNICATION(false) // Turn off Serial communication since it uses 1.6mA of power
BLE_COMMUNICATION(true)
MOVESENSE_FEATURES_END()
