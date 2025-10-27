// File: exti.rs
// Author: Lincoln D'Epagnier
// Created on: 10/24/2025
//
// Discription: All code related to the Extended Interrupts and Events Controller EXTI.

use core::ptr::{read_volatile, write_volatile};

pub const EXTI_BASE: u32 = 0x4001_0400;
/// Rising trigger selection registers offset
pub const EXTI_RTSR1_OFFSET: u32 = 0x08;
pub const EXTI_RTSR2_OFFSET: u32 = 0x28;
/// Falling trigger selection registers offset
pub const EXTI_FTSR1_OFFSET: u32 = 0x0C;
pub const EXTI_FTSR2_OFFSET: u32 = 0x2C;
// Exti Interrupt mask registers
pub const EXTI_IMR1_OFFSET: u32 = 0x00;
pub const EXTI_IMR2_OFFSET: u32 = 0x20;
// SYSCFG external interrupt configuration registers
pub const SYSCFG_BASE: u32 = 0x4001_0000;
pub const SYSCFG_EXTICR1_OFFSET: u32 = 0x08;
// pending register
pub const EXTI_PR1_OFFSET: u32 = 0x14;
pub const EXTI_PR2_OFFSET: u32 = 0x34;

pub mod gpio {
    use core::ptr::{read_volatile, write_volatile};

    use crate::{
        exti::{
            EXTI_BASE, EXTI_FTSR1_OFFSET, EXTI_FTSR2_OFFSET, EXTI_RTSR1_OFFSET, EXTI_RTSR2_OFFSET,
            SYSCFG_BASE,
        },
        mcu::GPIOPort,
    };

    /// Interrupt trigger type
    pub enum EdgeTrigger {
        FallingEdge,
        RisingEdge,
    }

    pub fn set_edge_trigger(pin: u32, edge: EdgeTrigger) {
        let (exti_reg_1, bit_pos) = match pin {
            0..=31 if !(23..=28).contains(&pin) => (true, pin),
            32..=33 => (false, pin - 31),
            _ => return, // not a setable exti pin
        };
        let (rtsr_off, ftsr_off) = if exti_reg_1 {
            (EXTI_RTSR1_OFFSET, EXTI_FTSR1_OFFSET)
        } else {
            (EXTI_RTSR2_OFFSET, EXTI_FTSR2_OFFSET)
        };

        let rtsr = (EXTI_BASE + rtsr_off) as *mut u32;
        let ftsr = (EXTI_BASE + ftsr_off) as *mut u32;

        unsafe {
            match edge {
                EdgeTrigger::RisingEdge => {
                    write_volatile(ftsr, read_volatile(ftsr) & !(1 << bit_pos));
                    write_volatile(rtsr, read_volatile(rtsr) | (1 << bit_pos));
                }
                EdgeTrigger::FallingEdge => {
                    write_volatile(rtsr, read_volatile(rtsr) & !(1 << bit_pos));
                    write_volatile(ftsr, read_volatile(ftsr) | (1 << bit_pos));
                }
            }
        }
    }

    pub fn configure_syscfg_external_iterrupt(port: GPIOPort, pin: u32) {
        let port_pos = pin / 4;
        let syscfg_reg_num: u32 = port_pos * 4;
        let syscfg_reg_addr = (SYSCFG_BASE + 0x08 + syscfg_reg_num) as *mut u32;

        let cnfg_val: u32 = match port {
            GPIOPort::PortA => 0b0000,
            GPIOPort::PortB => 0b0001,
            GPIOPort::PortC => 0b0010,
            GPIOPort::PortD => 0b0011,
            GPIOPort::PortE => 0b0100,
            GPIOPort::PortF => 0b0101,
            GPIOPort::PortG => 0b0110,
            GPIOPort::PortH => 0b0111,
        };
        unsafe {
            let mut syscfg_reg_val = read_volatile(syscfg_reg_addr);
            let shift: u32 = (pin % 4) * 4;
            syscfg_reg_val &= !(0xF << shift);
            syscfg_reg_val |= cnfg_val << shift;
            write_volatile(syscfg_reg_addr, syscfg_reg_val);
        }
    }
}

pub enum ExtiLine {
    // Gpio EXTI Lines 0-15
    EXTI0,
    EXTI1,
    EXTI2,
    EXTI3,
    EXTI4,
    EXTI5,
    EXTI6,
    EXTI7,
    EXTI8,
    EXTI9,
    EXTI10,
    EXTI11,
    EXTI12,
    EXTI13,
    EXTI14,
    EXTI15,
    // Add other EXTI lines here
}
impl ExtiLine {
    pub fn from_pin(pin: u32) -> Option<ExtiLine> {
        match pin {
            0 => Some(ExtiLine::EXTI0),
            1 => Some(ExtiLine::EXTI1),
            2 => Some(ExtiLine::EXTI2),
            3 => Some(ExtiLine::EXTI3),
            4 => Some(ExtiLine::EXTI4),
            5 => Some(ExtiLine::EXTI5),
            6 => Some(ExtiLine::EXTI6),
            7 => Some(ExtiLine::EXTI7),
            8 => Some(ExtiLine::EXTI0),
            9 => Some(ExtiLine::EXTI1),
            10 => Some(ExtiLine::EXTI2),
            11 => Some(ExtiLine::EXTI3),
            12 => Some(ExtiLine::EXTI4),
            13 => Some(ExtiLine::EXTI5),
            14 => Some(ExtiLine::EXTI6),
            15 => Some(ExtiLine::EXTI7),
            // add more here
            _ => None,
        }
    }
}

pub fn enable_interrupt(exti_line: ExtiLine) {
    let exti_imr1 = (EXTI_BASE + EXTI_IMR1_OFFSET) as *mut u32;
    let exti_imr2 = (EXTI_BASE + EXTI_IMR2_OFFSET) as *mut u32;
    let exti_line_num: u32 = exti_line as u32;
    let (exti_imr, bit_pos) = match exti_line_num {
        0..=31 => (exti_imr1, exti_line_num),
        32..=35 => (exti_imr2, exti_line_num - 32),
        _ => return, // out of range of valid exti line number
    };
    unsafe {
        let mut exti_imr_val = read_volatile(exti_imr);
        exti_imr_val |= 1 << bit_pos;
        write_volatile(exti_imr, exti_imr_val);
    }
}

pub fn disable_interrrupt(exti_line: ExtiLine) {
    let exti_imr1 = (EXTI_BASE + EXTI_IMR1_OFFSET) as *mut u32;
    let exti_imr2 = (EXTI_BASE + EXTI_IMR2_OFFSET) as *mut u32;
    let exti_line_num: u32 = exti_line as u32;
    let (exti_imr, bit_pos) = match exti_line_num {
        0..=31 => (exti_imr1, exti_line_num),
        32..=35 => (exti_imr2, exti_line_num - 32),
        _ => return, // out of range of valid exti line number
    };
    unsafe {
        let mut exti_imr_val = read_volatile(exti_imr);
        exti_imr_val &= !(1 << bit_pos);
        write_volatile(exti_imr, exti_imr_val);
    }
}

pub fn clear_pending_interrupt(exti_line: ExtiLine) {
    let exti_pr1 = (EXTI_BASE + EXTI_PR1_OFFSET) as *mut u32;
    let exti_pr2 = (EXTI_BASE + EXTI_PR2_OFFSET) as *mut u32;
    let exti_line_num: u32 = exti_line as u32;
    let (exti_imr, bit_pos) = match exti_line_num {
        0..=31 if !(23..=28).contains(&exti_line_num) => (exti_pr1, exti_line_num),

        32..=35 => (exti_pr2, exti_line_num - 32),
        _ => return, // out of range of valid exti line number
    };
    unsafe {
        let mut exti_imr_val = read_volatile(exti_imr);
        exti_imr_val &= !(1 << bit_pos);
        write_volatile(exti_imr, exti_imr_val);
    }
}
