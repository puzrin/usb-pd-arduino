//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32G0xx) || defined(STM32G4xx)

#include <Arduino.h>
#include "stm32yyxx_ll_bus.h"
#include "stm32yyxx_ll_dma.h"
#include "stm32yyxx_ll_pwr.h"
#include "stm32yyxx_ll_ucpd.h"
#include "stm32yyxx_ll_system.h"
#include "PDController.h"
#include "PDPhySTM32UCPD.h"


#if defined(STM32G4xx)
    // STM32G4 family: CC1 -> PB6, CC2 -> PB4
    #define GPIO_CC1 GPIOB
    #define PIN_CC1 LL_GPIO_PIN_6
    #define ARDUINO_PIN_CC1 PB6
    #define GPIO_CC2 GPIOB
    #define PIN_CC2 LL_GPIO_PIN_4
    #define ARDUINO_PIN_CC2 PB4
    #define DMA_RX DMA1
    #define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
    #define DMA_TX DMA1
    #define DMA_CHANNEL_TX LL_DMA_CHANNEL_2
    #define UCPD_IRQ UCPD1_IRQn

#elif defined(STM32G0xx)

const PDPhySTM32UCPD::Peripheral PDPhySTM32UCPD::peripherals[] = {
    // STM32G0 family / UCPD1: CC1 -> PA8, CC2 -> PB15
    {
        .ucpd = UCPD1,
        .gpioCC1 = GPIOA,
        .gpioCC2 = GPIOB,
        .pinCC1 = LL_GPIO_PIN_8,
        .pinCC2 = LL_GPIO_PIN_15,
        .arduinoPinCC1 = PA8,
        .arduinoPinCC2 = PB15,
        .dmaRx = DMA1,
        .dmaTx = DMA1,
        .dmaChannelRx = LL_DMA_CHANNEL_1,
        .dmaRequestRx = LL_DMAMUX_REQ_UCPD1_RX,
        .dmaChannelTx = LL_DMA_CHANNEL_2,
        .dmaRequestTx = LL_DMAMUX_REQ_UCPD1_TX,
        .ucpdIrq = UCPD1_2_IRQn,
        .dbatt = LL_SYSCFG_UCPD1_STROBE,
    },
    // STM32G0 family / UCPD2: CC1 -> PD0, CC2 -> PD2
    {
        .ucpd = UCPD2,
        .gpioCC1 = GPIOD,
        .gpioCC2 = GPIOD,
        .pinCC1 = LL_GPIO_PIN_0,
        .pinCC2 = LL_GPIO_PIN_2,
        .arduinoPinCC1 = PD0,
        .arduinoPinCC2 = PD2,
        .dmaRx = DMA1,
        .dmaTx = DMA1,
        .dmaChannelRx = LL_DMA_CHANNEL_3,
        .dmaRequestRx = LL_DMAMUX_REQ_UCPD2_RX,
        .dmaChannelTx = LL_DMA_CHANNEL_4,
        .dmaRequestTx = LL_DMAMUX_REQ_UCPD2_TX,
        .ucpdIrq = UCPD1_2_IRQn,
        .dbatt = LL_SYSCFG_UCPD2_STROBE,
    }
};

PDPhySTM32UCPD* PDPhySTM32UCPD::instances[] = {0};

    // #define GPIO_CC1 GPIOA
    // #define PIN_CC1 LL_GPIO_PIN_8
    // #define ARDUINO_PIN_CC1 PA8
    // #define GPIO_CC2 GPIOB
    // #define PIN_CC2 LL_GPIO_PIN_15
    // #define ARDUINO_PIN_CC2 PB15
    // #define DMA_RX DMA1
    // #define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
    // #define DMA_TX DMA1
    // #define DMA_CHANNEL_TX LL_DMA_CHANNEL_2
    // #define UCPD_IRQ UCPD1_2_IRQn

    // #define GPIO_CC1 GPIOD
    // #define PIN_CC1 LL_GPIO_PIN_0
    // #define ARDUINO_PIN_CC1 PD0
    // #define GPIO_CC2 GPIOD
    // #define PIN_CC2 LL_GPIO_PIN_2
    // #define ARDUINO_PIN_CC2 PD2
    // #define DMA_RX DMA1
    // #define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
    // #define DMA_TX DMA1
    // #define DMA_CHANNEL_TX LL_DMA_CHANNEL_2
    // #define UCPD_IRQ UCPD1_2_IRQn

#endif


PDPhySTM32UCPD::PDPhySTM32UCPD() : controller(nullptr), rxMessage(nullptr), ccActive(0), instance(0) {}

void PDPhySTM32UCPD::startSink(PDController<PDPhySTM32UCPD>* controller) {
    this->controller = controller;
    init(false);
}

void PDPhySTM32UCPD::init(bool isMonitor) {
    instances[instance] = this;
    peripheral = &peripherals[instance];

    // clocks
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    #if defined(STM32G4xx)
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    #else
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    #endif

    // Use Arduino function for basic pin configuration so the Arduino library is aware
    // if the most important settings (such as GPIO clock initialization).
    pinMode(peripheral->arduinoPinCC1, INPUT_ANALOG);
    pinMode(peripheral->arduinoPinCC2, INPUT_ANALOG);

    // initialize UCPD peripheral
    LL_UCPD_InitTypeDef ucpdInit = {};
    LL_UCPD_StructInit(&ucpdInit);
    LL_UCPD_Init(peripheral->ucpd, &ucpdInit);

    LL_GPIO_InitTypeDef pinCc1Init = {
        .Pin = peripheral->pinCC1,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(peripheral->gpioCC1, &pinCc1Init);

    LL_GPIO_InitTypeDef pinCc2Init = {
        .Pin = peripheral->pinCC2,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(peripheral->gpioCC2, &pinCc2Init);

    // configure DMA for USB PD RX
    LL_DMA_InitTypeDef rxDmaInit = {
        .PeriphOrM2MSrcAddress = (uint32_t)&peripheral->ucpd->RXDR,
        .MemoryOrM2MDstAddress = 0,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = 0,
        .PeriphRequest = peripheral->dmaRequestRx,
        .Priority = LL_DMA_PRIORITY_LOW,
    };
    LL_DMA_Init(peripheral->dmaRx, peripheral->dmaChannelRx, &rxDmaInit);

    if (!isMonitor) {
        // configure DMA for USB PD TX
        LL_DMA_InitTypeDef txDmaInit = {
            .PeriphOrM2MSrcAddress = (uint32_t)&peripheral->ucpd->TXDR,
            .MemoryOrM2MDstAddress = 0,
            .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
            .Mode = LL_DMA_MODE_NORMAL,
            .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
            .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .NbData = 0,
            .PeriphRequest = peripheral->dmaRequestTx,
            .Priority = LL_DMA_PRIORITY_LOW
        };
        LL_DMA_Init(peripheral->dmaTx, peripheral->dmaChannelTx, &txDmaInit);
    }

    #if defined(STM32G4xx)
        // turn off dead battery detection
        LL_PWR_DisableUCPDDeadBattery();
    #endif

    // configure ordered sets
    LL_UCPD_SetRxOrderSet(peripheral->ucpd, LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_SOP1 | LL_UCPD_ORDERSET_SOP2 |
                                     LL_UCPD_ORDERSET_CABLERST | LL_UCPD_ORDERSET_HARDRST);

    // enable interrupts
    LL_UCPD_EnableIT_TypeCEventCC1(peripheral->ucpd);
    LL_UCPD_EnableIT_TypeCEventCC2(peripheral->ucpd);

    // enable
    LL_UCPD_Enable(peripheral->ucpd);

    // configure as sink (when enabled)
    LL_UCPD_SetSNKRole(peripheral->ucpd);
    LL_UCPD_SetRpResistor(peripheral->ucpd, isMonitor ? LL_UCPD_RESISTOR_DEFAULT : LL_UCPD_RESISTOR_NONE);
    LL_UCPD_SetccEnable(peripheral->ucpd, LL_UCPD_CCENABLE_CC1CC2);
    #if defined(STM32G0xx)
        if (!isMonitor) {
            LL_SYSCFG_DisableDBATT(peripheral->dbatt);
        }
    #endif

    // enable DMA
    LL_UCPD_RxDMAEnable(peripheral->ucpd);
    if (!isMonitor)
        LL_UCPD_TxDMAEnable(peripheral->ucpd);

    // same interrupt priority as timer 7 so they don't interrupt each other (code is not re-entrant)
    NVIC_SetPriority((IRQn_Type)peripheral->ucpdIrq, NVIC_GetPriority(TIM7_IRQn));
    // enable interrupt handler
    NVIC_EnableIRQ((IRQn_Type)peripheral->ucpdIrq);
}

void PDPhySTM32UCPD::prepareRead(PDMessage* msg) {
    rxMessage = msg;
    if (ccActive != 0)
        enableRead();
}

void PDPhySTM32UCPD::enableRead() {
    if (rxMessage == nullptr)
        return;
        
    // enable RX DMA
    LL_DMA_SetMemoryAddress(peripheral->dmaRx, peripheral->dmaChannelRx, reinterpret_cast<uint32_t>(&rxMessage->header));
    LL_DMA_SetDataLength(peripheral->dmaRx, peripheral->dmaChannelRx, 30);
    LL_UCPD_RxEnable(peripheral->ucpd);
    LL_DMA_EnableChannel(peripheral->dmaRx, peripheral->dmaChannelRx);
}

bool PDPhySTM32UCPD::transmitMessage(const PDMessage* msg) {
    // configure DMA request
    LL_DMA_SetMemoryAddress(peripheral->dmaTx, peripheral->dmaChannelTx, reinterpret_cast<uint32_t>(&msg->header));
    LL_DMA_SetDataLength(peripheral->dmaTx, peripheral->dmaChannelTx, msg->payloadSize());
    LL_DMA_EnableChannel(peripheral->dmaTx, peripheral->dmaChannelTx);

    // start transmitting
    LL_UCPD_WriteTxOrderSet(peripheral->ucpd, LL_UCPD_ORDERED_SET_SOP);
    LL_UCPD_WriteTxPaySize(peripheral->ucpd, msg->payloadSize());
    LL_UCPD_SendMessage(peripheral->ucpd);

    return true;
}

void PDPhySTM32UCPD::enableCommunication(int cc) {
    // set pin for communication
    LL_UCPD_SetCCPin(peripheral->ucpd, cc == 1 ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);

    // enable interrupts
    LL_UCPD_EnableIT_RxHRST(peripheral->ucpd);
    LL_UCPD_EnableIT_RxMsgEnd(peripheral->ucpd);
    LL_UCPD_EnableIT_TxMSGSENT(peripheral->ucpd);
    LL_UCPD_EnableIT_TxMSGDISC(peripheral->ucpd);
    LL_UCPD_EnableIT_TxMSGABT(peripheral->ucpd);

    enableRead();
}

void PDPhySTM32UCPD::disableCommunication() {

    // cancel read
    LL_DMA_DisableChannel(peripheral->dmaRx, peripheral->dmaChannelRx);
    LL_UCPD_RxDisable(peripheral->ucpd);

    // cancel transmit
    LL_DMA_DisableChannel(peripheral->dmaTx, peripheral->dmaChannelTx);

    // disable interrupts
    LL_UCPD_DisableIT_RxMsgEnd(peripheral->ucpd);
    LL_UCPD_DisableIT_RxHRST(peripheral->ucpd);
    LL_UCPD_DisableIT_TxMSGSENT(peripheral->ucpd);
    LL_UCPD_DisableIT_TxMSGDISC(peripheral->ucpd);
    LL_UCPD_DisableIT_TxMSGABT(peripheral->ucpd);
}

// interrupt handler

void PDPhySTM32UCPD::handleInterrupt() {

    UCPD_TypeDef* ucpd = peripheral->ucpd;
    uint32_t status = ucpd->SR;
    if (status == 0)
        return;

    // voltage changed on CC1 or CC2 pin
    if ((status & (UCPD_SR_TYPECEVT1 | UCPD_SR_TYPECEVT2)) != 0) {
        LL_UCPD_ClearFlag_TypeCEventCC1(ucpd);
        LL_UCPD_ClearFlag_TypeCEventCC2(ucpd);

        int cc = 0;
        if (LL_UCPD_GetTypeCVstateCC1(ucpd) != 0) {
            cc = 1;
        } else if (LL_UCPD_GetTypeCVstateCC2(ucpd) != 0) {
            cc = 2;
        }
        if (cc != ccActive) {
            ccActive = cc;
            if (cc != 0)
                enableCommunication(cc);
            else
                disableCommunication();

            controller->onVoltageChanged(cc);
        }
    }

    // hard reset received
    if ((status & UCPD_SR_RXHRSTDET) != 0) {
        LL_UCPD_ClearFlag_RxHRST(ucpd);
        controller->onReset(PDSOPSequence::hardReset);
    }

    // message received
    if ((status & UCPD_SR_RXMSGEND) != 0) {
        LL_UCPD_ClearFlag_RxMsgEnd(ucpd);
        LL_DMA_DisableChannel(peripheral->dmaRx, peripheral->dmaChannelRx);
        if ((status & UCPD_SR_RXERR) == 0) {
            uint32_t orderedSet = LL_UCPD_ReadRxOrderSet(ucpd);
            rxMessage->sopSequence = mapSOPSequence(orderedSet);
            rxMessage->cc = ccActive;
            controller->onMessageReceived(rxMessage);

        } else {
            controller->onError();
        }
    }

    // message sent
    if ((status & UCPD_SR_TXMSGSENT) != 0) {
        LL_UCPD_ClearFlag_TxMSGSENT(ucpd);
        LL_DMA_DisableChannel(peripheral->dmaTx, peripheral->dmaChannelTx);
        controller->onMessageTransmitted(true);
    }

    // message aborted
    if ((status & UCPD_SR_TXMSGABT) != 0) {
        LL_UCPD_ClearFlag_TxMSGABT(ucpd);
        LL_DMA_DisableChannel(peripheral->dmaTx, peripheral->dmaChannelTx);
        controller->onMessageTransmitted(false);
    }

    // message discarded
    if ((status & UCPD_SR_TXMSGDISC) != 0) {
        LL_UCPD_ClearFlag_TxMSGDISC(ucpd);
        LL_DMA_DisableChannel(peripheral->dmaTx, peripheral->dmaChannelTx);
        controller->onMessageTransmitted(false);
    }
}

PDSOPSequence PDPhySTM32UCPD::mapSOPSequence(uint32_t orderedSet) {
    switch (orderedSet) {
    case LL_UCPD_RXORDSET_SOP:
        return PDSOPSequence::sop;
    case LL_UCPD_RXORDSET_SOP1:
        return PDSOPSequence::sop1;
    case LL_UCPD_RXORDSET_SOP2:
        return PDSOPSequence::sop2;
    case LL_UCPD_RXORDSET_SOP1_DEBUG:
        return PDSOPSequence::sop1Debug;
    case LL_UCPD_RXORDSET_SOP2_DEBUG:
        return PDSOPSequence::sop2Debug;
    case LL_UCPD_RXORDSET_CABLE_RESET:
        return PDSOPSequence::cableReset;
    default:
        return PDSOPSequence::invalid;
    }
}

#endif
