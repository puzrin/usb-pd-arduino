//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

// Code that needs to be included in a single .cpp file as it is not just a header file
// with declarations but provides actual implementation.

#if USB_PD_PHY == USB_PD_PHY_UCPD1 || USB_PD_PHY == USB_PD_PHY_UCPD2 || USB_PD_PHY == USB_PD_PHY_UCPD1_2

#include "phy/STM32UCPD/STM32UCPDInterruptHandler.h"

#elif USB_PD_PHY == USB_PD_PHY_FUSB302

// nothing further to include

#else
    #error "Invalid or missing value for USB_PD_PHY. Please define USB_PD_PHY before including this file."
#endif
