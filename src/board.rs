use core::ptr::{read_volatile, write_volatile};

use crate::mcu::{
    self, GPIO_PORT_OUTPUT_TYPE_OFFSET, GPIOE_BASE, GPIOMode, GPIOOutputType, GPIOPort, PinState,
    set_pin_state,
};

pub const LD3_LED_PIN: u32 = 9; // I/0 PE9
pub const LD3_LED_PORT: u32 = GPIOE_BASE;

pub enum ButtonStatus {
    Pressed,
    Released,
}

/// led_init initializes the pin on the port provided to be and gpio output with output type as
/// pushpull
pub fn led_init(port: u32, pin: u32) {
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

pub fn button_init(port: u32, pin: u32) {
    todo!()
}

pub fn button_configure_interrupt() {}

pub fn button_read_status(port: u32, pin: u32) -> ButtonStatus {
    todo!()
}
