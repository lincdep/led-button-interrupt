// File: proc.rs
// Author: Lincoln D'Epagnier
// Created on: 10/25/2025
//
// Discription: processor specific code

// Addresses of nvic registers

use core::ptr::{read_volatile, write_volatile};

pub const NVIC_BASE: u32 = 0xE000_E100;
// Interrupt Set-Enable Registers
pub const NVIC_ISER: u32 = NVIC_BASE;
// Interrupt Clear-Enable Registers
pub const NVIC_ICER: u32 = NVIC_BASE + 0x80;

/// Enables the IRQ for the given IRQ number.
pub fn enable_irq(irq_number: u32) {
    let register_offset = (irq_number / 32) * 4;
    let bit_pos = irq_number % 32;
    let iser_addr = (NVIC_ISER + register_offset) as *mut u32;
    unsafe {
        let iser_val = read_volatile(iser_addr);
        write_volatile(iser_addr, iser_val | (1 << bit_pos));
    }
}

pub fn disable_irq(irq_number: u32) {
    let register_offset = (irq_number / 32) * 4;
    let bit_pos = irq_number % 32;
    let icer_addr = (NVIC_ICER + register_offset) as *mut u32;
    unsafe {
        let iser_val = read_volatile(icer_addr);
        write_volatile(icer_addr, iser_val & (!(1 << bit_pos)));
    }
}
