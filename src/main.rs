#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]

mod board;
mod exti;
mod mcu;
mod proc;

use crate::{
    board::{
        ButtonStatus, InputMode, LD3_LED_PIN, LD3_LED_PORT, USER_BUTTON_PIN, USER_BUTTON_PORT,
        USER_BUTTON_PORT_ADDR, button_clear_iterrupt, button_init, led_init, led_off, led_on,
        led_toggle,
    },
    exti::gpio::EdgeTrigger,
    mcu::{
        GPIO_BSRR_OFFSET, GPIO_ODR_OFFSET, GPIO_PORT_OUTPUT_TYPE_OFFSET, GPIOE_BASE, GPIOMode,
        GPIOOutputType, GPIOPort, PinState, enable_gpio_clock, set_pin_state,
    },
};
use core::{
    hint::black_box,
    panic::PanicInfo,
    ptr::{self, read_volatile, write_volatile},
    sync::atomic,
};
use cortex_m_rt::entry;
use stm32f3::stm32f303::interrupt;

/*** Panic Handler ***/
#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(atomic::Ordering::SeqCst);
    }
}

/*** Interrupt Handler ***/
#[interrupt]
fn EXTI0() {
    button_clear_iterrupt(USER_BUTTON_PIN);
    led_toggle(LD3_LED_PORT, LD3_LED_PIN);
}

/*** Main ***/
#[entry]
fn main() -> ! {
    // Led init
    enable_gpio_clock(GPIOPort::PortE);
    led_init(LD3_LED_PORT, LD3_LED_PIN);

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
