use core::{
    pin,
    ptr::{read_volatile, write_volatile},
};

use crate::{
    exti::{self, ExtiLine, gpio::EdgeTrigger},
    mcu::{
        self, GPIO_PORT_OUTPUT_TYPE_OFFSET, GPIOA_BASE, GPIOE_BASE, GPIOMode, GPIOOutputType,
        GPIOPort, PinState, enable_gpio_clock, set_gpio_port_mode, set_pin_state,
    },
    proc,
};

pub const LD3_LED_PIN: u32 = 9; // I/0 PE9
pub const LD3_LED_PORT: u32 = GPIOE_BASE;

// Buttons
pub const USER_BUTTON_PIN: u32 = 0;
pub const USER_BUTTON_PORT_ADDR: u32 = GPIOA_BASE;
pub const USER_BUTTON_PORT: GPIOPort = GPIOPort::PortA;

/// Button State
pub enum ButtonStatus {
    Pressed,
    Released,
}

/// Pin mode
pub enum InputMode {
    Input,
    Interrupt(EdgeTrigger),
}

/// led_init initializes the pin on the port provided to be and gpio output with output type as
/// pushpull
pub fn led_init(port: u32, pin: u32) {
    // todod replace with port mode
    // set the gpio to pin mode = output mode
    let gpio_port_mode_reg_addr: *mut u32 = (port + mcu::GPIOX_MODER_OFFSET) as *mut u32;
    let mut gpio_mode_reg_value: u32 = unsafe { read_volatile(gpio_port_mode_reg_addr) };

    let pin_postion: u32 = pin * 2;
    let mode_mask: u32 = 0b11 << pin_postion;
    let mut mode_value: u32 = u32::from(GPIOMode::OutputMode) << pin_postion;
    if pin == 5 {
        // pin 5 must be kept as reset state for xB/xC variants
        mode_value = 0b00 << pin_postion;
    }

    unsafe {
        gpio_mode_reg_value &= !(mode_mask); // clear the bit postion
        gpio_mode_reg_value |= mode_value; // set the pit postion
        write_volatile(gpio_port_mode_reg_addr, gpio_mode_reg_value);
    }

    // set output type to pushpull
    let gpio_output_type_reg_addr: *mut u32 = (port + GPIO_PORT_OUTPUT_TYPE_OFFSET) as *mut u32;
    unsafe {
        let mut gpio_output_type_val = read_volatile(gpio_port_mode_reg_addr);
        gpio_output_type_val |= u32::from(GPIOOutputType::PushPull) << pin; // set the output type to
        // PushPull
        write_volatile(gpio_output_type_reg_addr, gpio_output_type_val);
    }
}

/// turns the led on
pub fn let_on(port: u32, pin: u32) {
    set_pin_state(port, pin, PinState::GPIOPinHigh);
}

/// turns the provided led off
pub fn led_off(port: u32, pin: u32) {
    set_pin_state(port, pin, PinState::GPIOPinLow);
}

/// toggles the led on or off
pub fn led_toggle(port: u32, pin: u32) {
    set_pin_state(port, pin, PinState::GPIOPinToggle);
}

pub fn button_init(port: GPIOPort, pin: u32, mode: InputMode) {
    enable_gpio_clock(port);
    set_gpio_port_mode(port, pin, mcu::GPIOMode::InputMode);

    match mode {
        InputMode::Interrupt(trigger) => match trigger {
            EdgeTrigger::RisingEdge => {
                exti::gpio::set_edge_trigger(pin, EdgeTrigger::RisingEdge);
            }
            EdgeTrigger::FallingEdge => {
                exti::gpio::set_edge_trigger(pin, EdgeTrigger::FallingEdge);
            }
        },

        InputMode::Input => {
            // allready set the pin mode to input.
        }
    }
    // enable the iterrupt
    if let Some(exti_line) = exti::ExtiLine::from_pin(pin) {
        exti::enable_interrupt(exti_line);
    }
    // enable itq
    if let Some(irq_num) = mcu::IRQn::from_pin(pin) {
        proc::enable_irq(irq_num);
    }
    // set the cfgreg
    exti::gpio::configure_syscfg_external_iterrupt(port, pin);
}

// pub fn button_configure_interrupt() {}
//
// pub fn button_read_status(port: u32, pin: u32) -> ButtonStatus {
//     todo!()
// }

pub fn button_clear_iterrupt(pin: u32) {
    if let Some(exti_line) = ExtiLine::from_pin(pin) {
        exti::clear_pending_interrupt(exti_line);
    }
}
