#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]

mod board;
mod mcu;

use core::{
    panic::PanicInfo,
    ptr::{self, read_volatile, write_volatile},
    sync::atomic,
};
use cortex_m_rt::entry;

use crate::{
    board::{
        ButtonStatus, LD3_LED_PIN, LD3_LED_PORT, Mode, Trigger, USER_BUTTON_PORT, button_init,
        led_init, let_on,
    },
    mcu::{
        GPIO_BSRR_OFFSET, GPIO_ODR_OFFSET, GPIO_PORT_OUTPUT_TYPE_OFFSET, GPIOE_BASE, GPIOMode,
        GPIOOutputType, GPIOPort, PinState, enable_gpio_clock, set_pin_state,
    },
};

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(atomic::Ordering::SeqCst);
    }
}

/*** Helpers ***/

/*** Interrupt Handler ***/
const GPIOE_MODER: u32 = GPIOE_BASE + 0x00;
const GPIOE_OTYPER: u32 = GPIOE_BASE + 0x04;
const GPIOE_OSPEEDR: u32 = GPIOE_BASE + 0x08;
const GPIOE_PUPDR: u32 = GPIOE_BASE + 0x0C;
const GPIOE_BSRR: u32 = GPIOE_BASE + 0x18;

/*** Main ***/
#[entry]
fn main() -> ! {
    //enable_gpio_clock(GPIOPor_sdsdinit(LD3_LED_PORT, LD3_LED_PIN);
    //set_pin_state(LD3_LED_PORT, LD3_LED_PIN, PinState::GPIOPinHigh);

    // Led init
    enable_gpio_clock(GPIOPort::PortE);
    led_init(LD3_LED_PORT, LD3_LED_PIN);
    let_on(LD3_LED_PORT, LD3_LED_PIN);

    // Button init
    button_init(
        USER_BUTTON_PORT,
        USER_BUTTON_PORT,
        Mode::Interrupt(Trigger::FallingEdge),
    );

    loop {
        // Spin forever
    }
}
