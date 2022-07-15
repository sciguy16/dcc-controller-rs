// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! blinky timer using interrupts on TIM2, adapted from blinky_timer_irq.rs example from
//! stm32f1xx-hal
//!
//! This assumes that a LED is connected to pa5 (sck/d13) as is the case on most nucleo board.

// Pinout:
// POT_LEFT: PB0
// POT_RIGHT: PB1
// DIPSW_LEFT: PB15, PA8, PA9, PA10, PA11, PA12, PA15
// DIPSW_RIGHT: PB3, PB4, PB5, PA6, PA5, PB8, PB9
// SCL: PB6
// SDA: PB7
// LED_LEFT: PA1
// LED_RIGHT: PA0
// LED_PROG: PB12
// LED_ALARM: PB13
// BTN_STOP: PB14
// BTN_PROG: PA4
// SW_PROG_LEFT: PB10
// SW_PROG_RIGHT: PB11
// OUT_EN: PA2
// OUT_DIR: PA3
// CURRENT_SENSE: PA7

#![no_main]
#![no_std]

use crate::hal::{
    adc,
    gpio::{gpioa, gpiob, Output, PushPull},
    i2c::{BlockingI2c, Mode},
    pac::{interrupt, Interrupt, Peripherals, TIM2},
    prelude::*,
    timer::{CounterUs, Event},
};
use core::cell::RefCell;
use core::fmt::{self, Display, Formatter, Write};
use core::panic::PanicInfo;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use dcc_rs::{packets::*, DccInterruptHandler};
use defmt::{debug, info};
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{ascii, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, Ssd1306};
use stm32f1xx_hal as hal;

mod address_switches;

use address_switches::AddressSwitches;

const CLOCK_MHZ: u32 = 48;

#[derive(Default)]
struct ChannelProps {
    address: u8,
    speed: u8,
    direction: Direction,
}

impl ChannelProps {
    pub fn with_address(address: u8) -> Self {
        Self {
            address,
            ..Default::default()
        }
    }

    pub fn direction_arrow(&self) -> char {
        match (self.speed, self.direction) {
            (0, _) => ' ',
            (_, Direction::Forward) => '>',
            (_, Direction::Backward) => '<',
        }
    }
}

#[derive(Copy, Clone)]
enum Channel {
    Left,
    Right,
}

impl Channel {
    pub fn swap(self) -> Self {
        match self {
            Channel::Left => Channel::Right,
            Channel::Right => Channel::Left,
        }
    }
}

impl Display for Channel {
    fn fmt(&self, fmt: &mut Formatter) -> fmt::Result {
        write!(
            fmt,
            "{}",
            match self {
                Channel::Left => "LEFT",
                Channel::Right => "RIGHT",
            }
        )
    }
}

enum RunState {
    /// Run state tracks the channel and alternates between them
    Run(Channel),
    Prog(Channel),
    EmergencyStop,
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    // Turn off outputs
    cortex_m::interrupt::free(|cs| {
        if let Some(out_en) = LED_ALARM.borrow(cs).borrow_mut().as_mut() {
            out_en.set_low();
        }
    });

    // grab alarm LED and turn it on
    let mut led = cortex_m::interrupt::free(|cs| {
        LED_ALARM.borrow(cs).borrow_mut().take()
    });
    if let Some(led) = &mut led {
        led.set_low();
    }
    loop {
        // Counting nops here doesn't give a precise timing because the
        // DCC timer is still running
        for _ in 0..CLOCK_MHZ * 10_000 {
            unsafe {
                core::arch::asm!("nop");
            }
        }
        if let Some(led) = &mut led {
            led.toggle();
        }
    }
}

// A type definition for the GPIO pin to be used for our LED
type DccDirPin = gpioa::PA3<Output<PushPull>>;

// Make DCC thingy globally available
static G_DCC: Mutex<RefCell<Option<DccInterruptHandler<DccDirPin>>>> =
    Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> =
    Mutex::new(RefCell::new(None));

// place for sending packets
static TX_BUFFER: Mutex<RefCell<Option<(SerialiseBuffer, usize)>>> =
    Mutex::new(RefCell::new(None));

type LedAlarm = gpiob::PB13<Output<PushPull>>;
static LED_ALARM: Mutex<RefCell<Option<LedAlarm>>> =
    Mutex::new(RefCell::new(None));

type EnablePin = gpioa::PA2<Output<PushPull>>;
static ENABLE_PIN: Mutex<RefCell<Option<EnablePin>>> =
    Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM2() {
    static mut DCC: Option<DccInterruptHandler<DccDirPin>> = None;
    static mut TIM: Option<CounterUs<TIM2>> = None;

    let dcc = DCC.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_DCC.borrow(cs).replace(None).unwrap()
        })
    });

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });

    if let Some((new_data, len)) =
        cortex_m::interrupt::free(|cs| TX_BUFFER.borrow(cs).replace(None))
    {
        dcc.write(&new_data[..len]).unwrap();
    }

    if let Ok(new_delay) = dcc.tick() {
        tim.start(new_delay.micros()).unwrap();
    }

    let _ = tim.wait();
}

#[entry]
fn main() -> ! {
    info!("Start boot");

    let dp = Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // Prepare the alternate function I/O registers
    let mut afio = dp.AFIO.constrain();
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(CLOCK_MHZ.MHz())
        //.pclk1(8.MHz())
        .freeze(&mut flash.acr);
    // info!("adc freq: {}", clocks.adcclk());

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    // store the alarm led pin so that it is available for the panic handler
    let mut led_alarm = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    led_alarm.set_high();
    cortex_m::interrupt::free(|cs| {
        *LED_ALARM.borrow(cs).borrow_mut() = Some(led_alarm)
    });

    info!("Initialise DCC interrupt handler");
    let dcc_pin = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let dcc = DccInterruptHandler::new(dcc_pin);
    cortex_m::interrupt::free(|cs| *G_DCC.borrow(cs).borrow_mut() = Some(dcc));

    info!("Initialise timer");
    let mut timer = dp.TIM2.counter_us(&clocks);
    // Generate an interrupt when the timer expires
    timer.start(50000.micros()).unwrap();
    timer.listen(Event::Update);
    cortex_m::interrupt::free(|cs| {
        *G_TIM.borrow(cs).borrow_mut() = Some(timer)
    });

    info!("Init I2C");
    cp.DWT.enable_cycle_counter();
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Standard {
            frequency: 400.kHz(),
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    info!("Init display");
    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    //enable TIM2 interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    // make a delay thing to send packets
    let mut delay = cp.SYST.delay(&clocks);

    // LED to show when power is on
    let mut led1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    let mut led2 = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
    let mut led_prog = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    led1.set_high();
    led2.set_high();
    led_prog.set_high();

    let mut out_en = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    out_en.set_high();
    cortex_m::interrupt::free(|cs| {
        *ENABLE_PIN.borrow(cs).borrow_mut() = Some(out_en)
    });

    // remove pins from the jtag peripheral and set up pins for address
    // selectors
    info!("Set up address pins");
    let (pa15, pb3, pb4) =
        afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    let mut addr_left = AddressSwitches::new(
        gpiob.pb15.into_pull_up_input(&mut gpiob.crh),
        gpioa.pa8.into_pull_up_input(&mut gpioa.crh),
        gpioa.pa9.into_pull_up_input(&mut gpioa.crh),
        gpioa.pa10.into_pull_up_input(&mut gpioa.crh),
        gpioa.pa11.into_pull_up_input(&mut gpioa.crh),
        gpioa.pa12.into_pull_up_input(&mut gpioa.crh),
        pa15.into_pull_up_input(&mut gpioa.crh),
    );
    let mut addr_right = AddressSwitches::new(
        pb3.into_pull_up_input(&mut gpiob.crl),
        pb4.into_pull_up_input(&mut gpiob.crl),
        gpiob.pb5.into_pull_up_input(&mut gpiob.crl),
        gpioa.pa6.into_pull_up_input(&mut gpioa.crl),
        gpioa.pa5.into_pull_up_input(&mut gpioa.crl),
        gpiob.pb8.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb9.into_pull_up_input(&mut gpiob.crh),
    );
    info!(
        "addresses: left={}, right={}",
        addr_left.value(),
        addr_right.value()
    );

    info!("Configure button pins");
    let btn_stop = gpiob.pb14.into_pull_up_input(&mut gpiob.crh);
    let btn_prog = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);
    let sw_prog_left = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);
    let sw_prog_right = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);

    // set up ADC
    info!("Configure ADC");
    let mut adc1 = adc::Adc::adc1(dp.ADC1, clocks);
    let mut ch0 = gpiob.pb1.into_analog(&mut gpiob.crl);
    let mut ch1 = gpiob.pb0.into_analog(&mut gpiob.crl);

    let mut adc2 = adc::Adc::adc2(dp.ADC2, clocks);
    let mut current_pin = gpioa.pa7.into_analog(&mut gpioa.crl);

    let mut chan_left = ChannelProps::with_address(addr_left.value());
    let mut chan_right = ChannelProps::with_address(addr_right.value());

    let mut row1 = FmtBuf::new();
    let mut row2 = FmtBuf::new();

    let mut run_state = RunState::Run(Channel::Left);

    info!("Enter main loop");
    loop {
        // read the current
        let current: u16 = adc2.read(&mut current_pin).unwrap();

        // update address
        addr_left.update();
        addr_right.update();

        run_state = match run_state {
            RunState::Run(chan) => {
                use Channel::*;

                // read the control
                let control: u16 = match chan {
                    Left => adc1.read(&mut ch0).unwrap(),
                    Right => adc1.read(&mut ch1).unwrap(),
                };

                // it's a 12-bit ADC, range is 0..=4095
                // speed is -16..=16
                // use the following ranges:
                // 0..1600: -16..0
                // 2495..=4095: 0..=16
                // 1601..2495: 0
                let (speed, direction) = match control {
                    0..=1600 => (28 - control / 57, Direction::Backward),
                    2495..=4095 => ((control - 2494) / 57, Direction::Forward),
                    _ => (0, Direction::Forward),
                };
                // info!(
                //     "control {} set to: {}, speed is {} {}",
                //     channel_select as u8, control, speed, direction
                // );
                let speed = (speed & 0x1f) as u8;
                match (speed, chan) {
                    (0, Left) => led1.set_high(),
                    (0, Right) => led2.set_high(),
                    (_, Left) => led1.set_low(),
                    (_, Right) => led2.set_low(),
                };

                let loco_addr = match chan {
                    Left => {
                        chan_left.speed = speed;
                        chan_left.direction = direction;
                        addr_left.value()
                    }
                    Right => {
                        chan_right.speed = speed;
                        chan_right.direction = direction;
                        addr_right.value()
                    }
                };

                //info!("tx, addr = {}", addr);
                // pop a new chunk of data into the buffer
                let pkt = SpeedAndDirection::builder()
                    .address(loco_addr)
                    .unwrap()
                    .speed(speed)
                    .unwrap()
                    .direction(direction)
                    .build();
                let mut buffer = SerialiseBuffer::default();
                let len = pkt.serialise(&mut buffer).unwrap();
                cortex_m::interrupt::free(|cs| {
                    *TX_BUFFER.borrow(cs).borrow_mut() = Some((buffer, len))
                });

                debug!(
                    "Speed {}: {}, speed {}: {}, current: {}",
                    chan_left.address,
                    chan_left.speed,
                    chan_right.address,
                    chan_right.speed,
                    current
                );

                row1.reset();
                write!(
                    &mut row1,
                    "{:03} TRAIN {:03}",
                    addr_left.value(),
                    addr_right.value()
                )
                .unwrap();

                row2.reset();
                {
                    let a = chan_left.direction_arrow();
                    let b = chan_right.direction_arrow();
                    write!(
                        &mut row2,
                        "{a}{speed1:02}{a}     {b}{speed2:02}{b}",
                        speed1 = chan_left.speed,
                        speed2 = chan_right.speed,
                    )
                }
                .unwrap();

                // Display is 128 x 64
                // Font is 10x20
                //
                // 000  loco  000
                //
                // 05 >      > 04
                //

                delay.delay_ms(10u16);

                // decide the next state
                if btn_stop.is_low() {
                    info!("Emergency stop!");
                    RunState::EmergencyStop
                } else if sw_prog_left.is_low() {
                    info!("Program mode LEFT");
                    RunState::Prog(Channel::Left)
                } else if sw_prog_right.is_low() {
                    info!("Program mode RIGHT");
                    RunState::Prog(Channel::Right)
                } else {
                    RunState::Run(chan.swap())
                }
            }
            RunState::Prog(chan) => {
                // Display programming mode on screen with address and
                // channel selection
                led_prog.set_low();

                row1.reset();
                row2.reset();
                write!(
                    &mut row1,
                    "{:03} TRAIN {:03}",
                    addr_left.value(),
                    addr_right.value()
                )
                .unwrap();
                write!(&mut row2, "PROG: {}", chan).unwrap();

                // handle pressing the "program" button
                if btn_prog.is_low() {
                    led_prog.set_high();
                    //TODO do program sequence
                }

                // decide the next state
                let left = sw_prog_left.is_high();
                let right = sw_prog_right.is_high();

                match (left, right) {
                    (true, true) => {
                        info!("RUN mode");
                        led_prog.set_high();
                        RunState::Run(Channel::Left)
                    }
                    (true, false) => RunState::Prog(Channel::Right),
                    (false, true) => RunState::Prog(Channel::Left),
                    (false, false) => {
                        // In theory it's impossible for the switch to
                        // be in both positions at the same time...
                        RunState::EmergencyStop
                    }
                }
            }
            RunState::EmergencyStop => {
                // disable outputs, draw "e-stop" on display and panic

                cortex_m::interrupt::free(|cs| {
                    if let Some(out_en) =
                        LED_ALARM.borrow(cs).borrow_mut().as_mut()
                    {
                        out_en.set_low();
                    }
                });

                row1.reset();
                row2.reset();
                write!(&mut row1, "STOP",).unwrap();

                // Empty the display:
                display.clear();

                // Draw 2 lines of text:
                Text::with_baseline(
                    row1.as_str(),
                    Point::new(0, 25),
                    text_style,
                    Baseline::Alphabetic,
                )
                .draw(&mut display)
                .unwrap();

                Text::with_baseline(
                    row2.as_str(),
                    Point::new(0, 50),
                    text_style,
                    Baseline::Alphabetic,
                )
                .draw(&mut display)
                .unwrap();

                display.flush().unwrap();

                panic!();
            }
        };

        // Empty the display:
        display.clear();

        // Draw 2 lines of text:
        Text::with_baseline(
            row1.as_str(),
            Point::new(0, 25),
            text_style,
            Baseline::Alphabetic,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            row2.as_str(),
            Point::new(0, 50),
            text_style,
            Baseline::Alphabetic,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
    }
}

/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)]
            .copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
