/*** GPIO ***/

use core::ptr::{self, read_volatile, write_volatile};

// gpio ports
pub const GPIO_PORT_SIZE: u32 = 0x400;
pub const GPIO_BASE: u32 = 0x4800_0000;
pub const GPIOA_BASE: u32 = GPIO_BASE;
pub const GPIOB_BASE: u32 = GPIO_BASE + (GPIO_PORT_SIZE);
pub const GPIOC_BASE: u32 = GPIO_BASE + (2 * GPIO_PORT_SIZE);
pub const GPIOD_BASE: u32 = GPIO_BASE + (3 * GPIO_PORT_SIZE);
pub const GPIOE_BASE: u32 = GPIO_BASE + (4 * GPIO_PORT_SIZE);
pub const GPIOF_BASE: u32 = GPIO_BASE + (5 * GPIO_PORT_SIZE);
pub const GPIOG_BASE: u32 = GPIO_BASE + (6 * GPIO_PORT_SIZE);
pub const GPIOH_BASE: u32 = GPIO_BASE + (7 * GPIO_PORT_SIZE);

pub const GPIOX_MODER_OFFSET: u32 = 0x00;
pub const GPIO_PORT_OUTPUT_TYPE_OFFSET: u32 = 0x04;
pub const GPIO_BSRR_OFFSET: u32 = 0x18;
pub const GPIO_ODR_OFFSET: u32 = 0x14;

// rcc
pub const RCC_BASE: u32 = 0x4002_1000;
pub const RCC_AHBENR_OFFSET: u32 = 0x14;

// got from fastbitlab.com
#[allow(non_camel_case_types, dead_code, clippy::upper_case_acronyms)]
pub enum IRQn {
    WWDG = 0,                // Window WatchDog Interrupt
    PVD = 1,                 // PVD through EXTI Line detection Interrupt
    TAMP_STAMP = 2,          // Tamper and TimeStamp interrupts through EXTI line 19
    RTC_WKUP = 3,            // RTC Wakeup interrupt through EXTI line 20
    FLASH = 4,               // FLASH global Interrupt
    RCC = 5,                 // RCC global Interrupt
    EXTI0 = 6,               // EXTI Line0 Interrupt
    EXTI1 = 7,               // EXTI Line1 Interrupt
    EXTI2_TSC = 8,           // EXTI Line2 Interrupt and Touch Sense Controller
    EXTI3 = 9,               // EXTI Line3 Interrupt
    EXTI4 = 10,              // EXTI Line4 Interrupt
    DMA1_Channel1 = 11,      // DMA1 Channel 1 Interrupt
    DMA1_Channel2 = 12,      // DMA1 Channel 2 Interrupt
    DMA1_Channel3 = 13,      // DMA1 Channel 3 Interrupt
    DMA1_Channel4 = 14,      // DMA1 Channel 4 Interrupt
    DMA1_Channel5 = 15,      // DMA1 Channel 5 Interrupt
    DMA1_Channel6 = 16,      // DMA1 Channel 6 Interrupt
    DMA1_Channel7 = 17,      // DMA1 Channel 7 Interrupt
    ADC1_2 = 18,             // ADC1 & ADC2 Interrupts
    USB_HP_CAN_TX = 19,      // USB Device High Priority or CAN TX Interrupts
    USB_LP_CAN_RX0 = 20,     // USB Device Low Priority or CAN RX0 Interrupts
    CAN_RX1 = 21,            // CAN RX1 Interrupt
    CAN_SCE = 22,            // CAN SCE Interrupt
    EXTI9_5 = 23,            // External Line[9:5] Interrupts
    TIM1_BRK_TIM15 = 24,     // TIM1 Break and TIM15 Interrupts
    TIM1_UP_TIM16 = 25,      // TIM1 Update and TIM16 Interrupts
    TIM1_TRG_COM_TIM17 = 26, // TIM1 Trigger and TIM17 Interrupt
    TIM1_CC = 27,            // TIM1 Capture Compare Interrupt
    TIM2 = 28,               // TIM2 global Interrupt
    TIM3 = 29,               // TIM3 global Interrupt
    TIM4 = 30,               // TIM4 global Interrupt
    I2C1_EV = 31,            // I2C1 Event Interrupt & EXTI Line23 Interrupt
    I2C1_ER = 32,            // I2C1 Error Interrupt
    I2C2_EV = 33,            // I2C2 Event Interrupt & EXTI Line24 Interrupt
    I2C2_ER = 34,            // I2C2 Error Interrupt
    SPI1 = 35,               // SPI1 global Interrupt
    SPI2 = 36,               // SPI2 global Interrupt
    USART1 = 37,             // USART1 global Interrupt & EXTI Line25
    USART2 = 38,             // USART2 global Interrupt & EXTI Line26
    USART3 = 39,             // USART3 global Interrupt & EXTI Line28
    EXTI15_10 = 40,          // External Line[15:10] Interrupts
    RTC_Alarm = 41,          // RTC Alarm (A and B) through EXTI Line 17
    USBWakeUp = 42,          // USB Wakeup Interrupt
    TIM8_BRK = 43,           // TIM8 Break Interrupt
    TIM8_UP = 44,            // TIM8 Update Interrupt
    TIM8_TRG_COM = 45,       // TIM8 Trigger and Commutation Interrupt
    TIM8_CC = 46,            // TIM8 Capture Compare Interrupt
    ADC3 = 47,               // ADC3 global Interrupt
    SPI3 = 51,               // SPI3 global Interrupt
    UART4 = 52,              // UART4 global Interrupt & EXTI Line34
    UART5 = 53,              // UART5 global Interrupt & EXTI Line35
    TIM6_DAC = 54,           // TIM6 global and DAC underrun error Interrupt
    TIM7 = 55,               // TIM7 global Interrupt
    DMA2_Channel1 = 56,      // DMA2 Channel 1 global Interrupt
    DMA2_Channel2 = 57,      // DMA2 Channel 2 global Interrupt
    DMA2_Channel3 = 58,      // DMA2 Channel 3 global Interrupt
    DMA2_Channel4 = 59,      // DMA2 Channel 4 global Interrupt
    DMA2_Channel5 = 60,      // DMA2 Channel 5 global Interrupt
    ADC4 = 61,               // ADC4 global Interrupt
    COMP1_2_3 = 64,          // COMP1, COMP2, COMP3 Interrupts via EXTI Lines
    COMP4_5_6 = 65,          // COMP4, COMP5, COMP6 Interrupts via EXTI Lines
    COMP7 = 66,              // COMP7 global Interrupt via EXTI Line33
    USB_HP = 74,             // USB High Priority global Interrupt
    USB_LP = 75,             // USB Low Priority global Interrupt
    USBWakeUp_RMP = 76,      // USB Wakeup Interrupt remap
    FPU = 81,                // Floating point Interrupt
}

impl IRQn {
    pub fn from_pin(pin: u32) -> Option<u32> {
        match pin {
            0 => Some(IRQn::EXTI0 as u32),
            1 => Some(IRQn::EXTI1 as u32),
            2 => Some(IRQn::EXTI2_TSC as u32),
            3 => Some(IRQn::EXTI3 as u32),
            4 => Some(IRQn::EXTI4 as u32),
            5..=9 => Some(IRQn::EXTI9_5 as u32),
            10..15 => Some(IRQn::EXTI15_10 as u32),
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
pub enum GPIOPort {
    PortH,
    PortA,
    PortB,
    PortC,
    PortD,
    PortE,
    PortF,
    PortG,
}
impl From<GPIOPort> for u32 {
    fn from(value: GPIOPort) -> Self {
        match value {
            GPIOPort::PortH => GPIOH_BASE,
            GPIOPort::PortA => GPIOA_BASE,
            GPIOPort::PortB => GPIOB_BASE,
            GPIOPort::PortC => GPIOC_BASE,
            GPIOPort::PortD => GPIOD_BASE,
            GPIOPort::PortE => GPIOE_BASE,
            GPIOPort::PortF => GPIOF_BASE,
            GPIOPort::PortG => GPIOG_BASE,
        }
    }
}

pub enum GPIOMode {
    InputMode,
    OutputMode,
    AltFuncMode,
    AnalogMode,
}
impl From<GPIOMode> for u32 {
    fn from(value: GPIOMode) -> Self {
        match value {
            GPIOMode::InputMode => 0b00,
            GPIOMode::OutputMode => 0b01,
            GPIOMode::AltFuncMode => 0b10,
            GPIOMode::AnalogMode => 0b11,
        }
    }
}

pub enum GPIOOutputType {
    PushPull,
    OpenDrain,
}
impl From<GPIOOutputType> for u32 {
    fn from(value: GPIOOutputType) -> Self {
        match value {
            GPIOOutputType::PushPull => 0b0,
            GPIOOutputType::OpenDrain => 0b1,
        }
    }
}

#[derive(PartialEq)]
pub enum PinState {
    GPIOPinHigh,
    GPIOPinLow,
    GPIOPinToggle,
}

// GPIO E

/*** GPIO Helper functions ***/

/// will enable the gpio clock on a given port
pub fn enable_gpio_clock(port: GPIOPort) {
    let rcc_ahbenr_reg: *mut u32 = (RCC_BASE + RCC_AHBENR_OFFSET) as *mut u32;
    let rcc_set_mask: u32 = match port {
        GPIOPort::PortH => 1 << 16,
        GPIOPort::PortA => 1 << 17,
        GPIOPort::PortB => 1 << 18,
        GPIOPort::PortC => 1 << 19,
        GPIOPort::PortD => 1 << 20,
        GPIOPort::PortE => 1 << 21,
        GPIOPort::PortF => 1 << 22,
        GPIOPort::PortG => 1 << 23,
    };
    // set the value
    unsafe {
        let mut rcc_ahbenr_value = read_volatile(rcc_ahbenr_reg);
        rcc_ahbenr_value |= rcc_set_mask;
        write_volatile(rcc_ahbenr_reg, rcc_ahbenr_value);
    }
}

/// set_pin_state will set the pin provided's state to the state
pub fn set_pin_state(port: u32, pin: u32, state: PinState) {
    unsafe {
        let bssr: *mut u32 = (GPIOE_BASE + GPIO_BSRR_OFFSET) as *mut u32;
        let bssr_write_val: u32 = match state {
            PinState::GPIOPinHigh => 1 << pin,
            PinState::GPIOPinLow => 1 << (pin + 16),
            PinState::GPIOPinToggle => {
                let podr: *mut u32 = (GPIOE_BASE + GPIO_ODR_OFFSET) as *mut u32;
                let podr_val = ptr::read_volatile(podr);
                if ((podr_val >> pin) & 0x1) != 0 {
                    1 << (pin + 16)
                } else {
                    1 << pin
                }
            }
        };
        write_volatile(bssr, bssr_write_val);
    }
}

/// Configure gpio pin mode
pub fn set_gpio_port_mode(port: GPIOPort, pin: u32, mode: GPIOMode) {
    match port {
        GPIOPort::PortF => {
            if pin == 10 || pin == 11 {
                // GPIOF pin 10 and 11 must be kept in reset state for STM32F303xB/xC
                return;
            }
        }
        _ => {}
    }
    let port_addr: u32 = u32::from(port);

    // set gpio port mode (GPIOx_MODER)
    let gpio_moder_addr: *mut u32 = (port_addr + GPIOX_MODER_OFFSET) as *mut u32;
    let mut gpio_moder_val: u32 = unsafe { read_volatile(gpio_moder_addr) };

    // set gpio port mode
    let moder_pin_postion: u32 = pin * 2;
    let mode_mask: u32 = 0b11 << moder_pin_postion;
    let mode_value_to_config: u32 = u32::from(mode) << moder_pin_postion;

    unsafe {
        gpio_moder_val &= !(mode_mask); // clear the bit postion
        gpio_moder_val |= mode_value_to_config; // set the pit postion
        write_volatile(gpio_moder_addr, gpio_moder_val);
    }
}
