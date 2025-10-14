/*** GPIO ***/

use core::ptr::{self, read_volatile, write_volatile};

// gpio ports
pub const GPIO_PORT_SIZE: u32 = 0x400;
pub const GPIO_BASE: u32 = 0x4800_0000;
pub const GPIOA_BASE: u32 = GPIO_BASE;
pub const GPIOB_BASE: u32 = GPIO_BASE + (1 * GPIO_PORT_SIZE);
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
        let mut bssr_write_val: u32 = match state {
            PinState::GPIOPinHigh => 1 << pin,
            PinState::GPIOPinLow => 1 << (pin + 16),
            PinState::GPIOPinToggle => {
                let mut podr: *mut u32 = (GPIOE_BASE + GPIO_ODR_OFFSET) as *mut u32;
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
