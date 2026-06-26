#pragma once

// Keep the water-meth firmware on the repository-wide CAN contract. Headers in
// this module can resolve quoted includes relative to modules/water-meth/include
// before PlatformIO's shared include path, so this local wrapper prevents an old
// duplicate contract from drifting out of sync.
#include "../../../../shared/can_contract/include/can_contract/can_protocol.h"
