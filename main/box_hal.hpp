#pragma once

#if CONFIG_HARDWARE_BOX
#include "box.hpp"
#elif CONFIG_HARDWARE_BOX_3
#include "box_3.hpp"
#else
#error "Invalid module selection"
#endif
