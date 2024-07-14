//! Fish
//!
//! Input: move detection -> d7 (interrupt on change)
//! Output: d2..=d6, d8..=d12, 10 MOSFETs in total
//! Interrupt: on change of d7

/*
* Timer0: 5,6
* Timer1: 9,10
* Timer2: 3,11
*
* PWM pins: 3, 5, 6, 9, 10, 11
*
* Port A: a0..a5
* Port B: d8..d13
* Port C: d0..d7
*/

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::{
    mem::MaybeUninit,
    sync::atomic::{self, AtomicBool, AtomicU8, Ordering}
};

use arduino_hal::{
    delay_ms, 
    Peripherals,
    port::{Pin, mode::Output},
};
use avr_device::{atmega328p::TC2, interrupt};
use panic_halt as _;

static mut FLAG: u8 = 0;
static MOVED: AtomicBool = AtomicBool::new(true);
static CHECKING: AtomicBool = AtomicBool::new(false);

static mut PWM_PINS: MaybeUninit<[PwmPin; 10]> = MaybeUninit::uninit();

struct PwmPin {
    pin: Pin<Output>,
    cur_val: u8,
    check_val: u8,
    next_check_val: AtomicU8,
}

impl PwmPin {
    fn set_duty(&mut self, duty: u8) {
        self.next_check_val.store(duty, Ordering::Relaxed);
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    rig_tc2(&dp.TC2);

    // Adrduino will be connected to move detection chip via d7 pin.
    // Any move detection causes an interrupt.
    // 
    dp.EXINT.pcicr.write(|w|
        // enable PCINT2 (Port D) pin change interrupt
        unsafe { w.bits(0b100) }
    );
    dp.EXINT.pcmsk2.write(|w|
        // enable pin change interrupt on PCINT23 (d7)
        w.bits(1 << 7)
    );

    let pwm_pins = [
        pins.d2.into_output().downgrade(),
        pins.d3.into_output().downgrade(),
        pins.d4.into_output().downgrade(),
        pins.d5.into_output().downgrade(),
        pins.d6.into_output().downgrade(),
        pins.d8.into_output().downgrade(),
        pins.d9.into_output().downgrade(),
        pins.d10.into_output().downgrade(),
        pins.d11.into_output().downgrade(),
        pins.d12.into_output().downgrade(),
    ].map(|pin| PwmPin {
        pin,
        cur_val: 0,
        check_val: 0,
        next_check_val: AtomicU8::new(u8::MAX),
    });

    // SAFETY: interrupt is not enabled at this point.
    unsafe { PWM_PINS = MaybeUninit::new(pwm_pins) }
    atomic::compiler_fence(Ordering::SeqCst);

    // Enable interrupts globally.
    // SAFETY: we are not inside a critical section.
    unsafe { interrupt::enable() };

    loop {
        if MOVED.load(Ordering::Acquire) {
            MOVED.store(false, Ordering::Release);

            // SAFETY: we _know_ that interrupts will only be enabled after all pins were
            // initialized so this interrupt will never run when PWM_PINS is uninitialized.
            let pwm_pins = unsafe { &mut *PWM_PINS.as_mut_ptr() };

            // patterns::one_by_one(pwm_pins);
            patterns::wave(pwm_pins);

            // wait for _10_ secs before next iteration
            delay_ms(10_000);
            CHECKING.store(true, Ordering::SeqCst);
        }
    }
}

#[allow(dead_code)]
mod patterns {
    use super::*;

    use core::iter;

    pub fn wave(pwm_pins: &mut [PwmPin]) {
        // fade down
        for duty in (0..=255).rev().step_by(16).chain(iter::once(0)) {
            for p in pwm_pins.iter_mut() {
                p.set_duty(duty);
            }
            delay_ms(20);
        }

        delay_ms(1_000);

        // Start waving.
        // Schema: 
        //     0 1 | 2 3 4 5 6 7 8 9 10 11 | 12 13 14 <-- iterator
        //           0 1 2 3 4 5 6 7 8  9             <-- pin id
        //             . . . ^ . . .                  <-- window
        //             0 9 1 2 1 9 0                  |
        //               0 8 5 8 0                    |-- duty
        //                 0 5 0                      |
        for _ in 0..3 {
            for iter_id in 0..=14 {
                match iter_id {
                    0 => pwm_pins[0].set_duty(90),
                    1 => {
                        pwm_pins[0].set_duty(180);
                        pwm_pins[1].set_duty(90);
                    }
                    12 => {
                        pwm_pins[9].set_duty(180);
                        pwm_pins[8].set_duty(90);
                        pwm_pins[7].set_duty(0);
                    }
                    13 => {
                        pwm_pins[9].set_duty(90);
                        pwm_pins[8].set_duty(0);
                    }
                    14 => pwm_pins[9].set_duty(0),
                    win_id => {
                        if win_id > 4 {
                            pwm_pins[win_id - 5].set_duty(0);
                        }
                        if win_id > 3 {
                            pwm_pins[win_id - 4].set_duty(90);
                        }
                        if win_id > 2 {
                            pwm_pins[win_id - 3].set_duty(180);
                        }
                        if (2..=11).contains(&win_id) {
                            pwm_pins[win_id - 2].set_duty(255);
                        }
                        if win_id < 11 {
                            pwm_pins[win_id - 1].set_duty(180);
                        }
                        if win_id < 10 {
                            pwm_pins[win_id].set_duty(90);
                        }
                    }
                }
                // this delay controls wave _speed_ 
                delay_ms(100);
            }
        }

        delay_ms(1_000);

        // fade up
        for duty in (0..=255).step_by(16).chain(iter::once(255)) {
            for p in pwm_pins.iter_mut() {
                p.set_duty(duty);
            }
            delay_ms(20);
        }
    }

    pub fn one_by_one(pwm_pins: &mut [PwmPin]) {
        // fade down
        for duty in (0..=255).rev().step_by(32).chain(iter::once(0)) {
            for p in pwm_pins.iter_mut() {
                p.set_duty(duty);
            }
            delay_ms(20);
        }

        // fade up and down each pin one-by-one
        for p in pwm_pins.iter_mut() {
            for duty in (0..=255).step_by(16)
                .chain((0..=255).rev().step_by(16))
                .chain(iter::once(0)) 
            {
                p.set_duty(duty);
                delay_ms(20);
            }
        }

        // fade up
        for duty in (0..=255).step_by(16) {
            for p in pwm_pins.iter_mut() {
                p.set_duty(duty);
            }
            delay_ms(20);
        }
    }
}


#[interrupt(atmega328p)]
fn TIMER2_COMPA() {
    // SAFETY: we _know_ that interrupts will only be enabled after all pins were
    // initialized so this interrupt will never run when PWM_PINS is uninitialized.
    let pwm_pins = unsafe { &mut *PWM_PINS.as_mut_ptr() };

    for p in pwm_pins.iter_mut() {
        if p.cur_val == 0 {
            p.check_val = p.next_check_val.load(Ordering::SeqCst);
        }

        if p.cur_val <= p.check_val {
            p.pin.set_high();
        } else {
            p.pin.set_low();
        }

        p.cur_val = p.cur_val.wrapping_add(1);
    }
}

#[interrupt(atmega328p)]
fn PCINT2() {
    if CHECKING.load(Ordering::SeqCst) {
        // Prevent from MOVE is being set twice.
        // SAFETY: access to FLAG happens only inside the current function.
        unsafe {
            FLAG = 1 - FLAG;
            if FLAG == 0 {
                return;
            }
        }

        MOVED.store(true, Ordering::SeqCst);
    }
}

fn rig_tc2(tc2: &TC2) {
    tc2.tccr2a.write(|w|
        // mode 2 (CTC): set WGM21 and WGM20 bits
        w.wgm2().ctc()
    );
    tc2.tccr2b.write(|w|
        w.cs2().prescale_8()
        // mode 2 (CTC): set WGM22
        .wgm22().clear_bit()
    );
    tc2.ocr2a.write(|w|
        // ticks value to achive 60 Hz
        w.bits(130)
    );
    tc2.timsk2.write(|w|
        // enable Output Compare Match interrupt
        w.ocie2a().set_bit()
    );
}
