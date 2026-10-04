#pragma once

// The CAN contract lives in DIYComfortControlModule/shared/can_contract.
// Resolve it whether this repo is checked out as modules/water-meth inside
// DIYComfortControlModule or standalone next to it.
#if __has_include("../../../../shared/can_contract/include/can_contract/can_protocol.h")
#include "../../../../shared/can_contract/include/can_contract/can_protocol.h"
#elif __has_include("../../../DIYComfortControlModule/shared/can_contract/include/can_contract/can_protocol.h")
#include "../../../DIYComfortControlModule/shared/can_contract/include/can_contract/can_protocol.h"
#else
#error "Shared CAN contract not found: build from DIYComfortControlModule/modules/water-meth or clone DIYComfortControlModule next to this repo."
#endif
