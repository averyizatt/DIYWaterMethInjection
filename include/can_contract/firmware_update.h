#pragma once

// Firmware update over CAN is part of the shared contract in
// DIYComfortControlModule/shared/can_contract (see can_protocol.h here for the layout).
#if __has_include("../../../../shared/can_contract/include/can_contract/firmware_update.h")
#include "../../../../shared/can_contract/include/can_contract/firmware_update.h"
#elif __has_include("../../../DIYComfortControlModule/shared/can_contract/include/can_contract/firmware_update.h")
#include "../../../DIYComfortControlModule/shared/can_contract/include/can_contract/firmware_update.h"
#else
#error "Shared CAN contract not found: build from DIYComfortControlModule/modules/water-meth or clone DIYComfortControlModule next to this repo."
#endif
