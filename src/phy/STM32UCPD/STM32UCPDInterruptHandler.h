//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32G4xx)

extern "C" void UCPD1_IRQHandler() {
    PDPhySTM32UCPD::instances[0]->handleInterrupt();
}

#endif

#if defined(STM32G0xx)

extern "C" void UCPD1_2_IRQHandler() {
    #if defined(USB_PD_PHY_UCPD1) || defined(USB_PD_PHY_UCPD1_2)
        PDPhySTM32UCPD* instance1 = PDPhySTM32UCPD::instances[0];
        if (instance1 != nullptr)
            instance1->handleInterrupt();
    #endif

    #if defined(USB_PD_PHY_UCPD2) || defined(USB_PD_PHY_UCPD1_2)
        PDPhySTM32UCPD* instance2 = PDPhySTM32UCPD::instances[1];
        if (instance2 != nullptr)
            instance2->handleInterrupt();
    #endif
}

#endif
