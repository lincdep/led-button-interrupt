#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]

mod board;
mod exti;
mod mcu;
mod proc;

use core::{
    panic::PanicInfo,
    ptr::{self, read_volatile, write_volatile},
    sync::atomic,
};
use cortex_m_rt::entry;

use crate::{
    board::{
        ButtonStatus, InputMode, LD3_LED_PIN, LD3_LED_PORT, USER_BUTTON_PIN, USER_BUTTON_PORT,
        USER_BUTTON_PORT_ADDR, button_clear_iterrupt, button_init, led_init, led_toggle, let_on,
    },
    exti::gpio::EdgeTrigger,
    mcu::{
        GPIO_BSRR_OFFSET, GPIO_ODR_OFFSET, GPIO_PORT_OUTPUT_TYPE_OFFSET, GPIOE_BASE, GPIOMode,
        GPIOOutputType, GPIOPort, PinState, enable_gpio_clock, set_pin_state,
    },
};

/*** Panic Handler ***/
#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(atomic::Ordering::SeqCst);
    }
}

/*** Interrupt Handler ***/
#[allow(non_snake_case, dead_code)]
fn EXTI9_Handler() {
    led_toggle(LD3_LED_PORT, LD3_LED_PIN);
    button_clear_iterrupt(USER_BUTTON_PIN);
}

/*** Main ***/
#[entry]
fn main() -> ! {
    // Led init
    enable_gpio_clock(GPIOPort::PortE);
    led_init(LD3_LED_PORT, LD3_LED_PIN);
    //let_on(LD3_LED_PORT, LD3_LED_PIN);

    // Button init
    button_init(
        USER_BUTTON_PORT,
        USER_BUTTON_PIN,
        InputMode::Interrupt(EdgeTrigger::FallingEdge),
    );

    loop {
        // Spin forever
    }
}
