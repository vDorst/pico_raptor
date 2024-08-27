// #![deny(clippy::pedantic)]
#![no_std]
#![no_main]
#![allow(unused_imports)]

use cortex_m::peripheral;
use embassy_rp::dma::{AnyChannel, Channel as DMA_Channel};
use embassy_rp::pac::pio;
use embassy_rp::usb::Out;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver};

use ::pio::{Assembler, SetDestination, SideSet};
use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::config::Config;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2C_Config, InterruptHandler as I2C_IH};
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::{I2C0, PIO1};
use embassy_rp::pio::{
    Common, Config as PIO_Config, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::spi::{Config as SPI_Config, Spi};
use embassy_rp::{bind_interrupts, clocks, into_ref, Peripheral, PeripheralRef};
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use ui::KeyAction;
use {defmt_rtt as _, panic_probe as _};

use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2C_IH<I2C0>;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;

});

const LED_DEFAULT: u32 = 0x0101_01FF;
const LED_NUM: usize = 16;

const BUF_NUM: usize = LED_NUM + 2;
const BUF_BYTE_SIZE: usize = BUF_NUM * 4;

const EYE_LEFT: [u8; 5] = [0, 1, 7, 8, 9];
const EYE_RIGHT: [u8; 5] = [2, 3, 4, 5, 6];
const TONG: [u8; 10] = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19];

const KEY_EYELEFT: u8 = 4;
const KEY_EYERIGHT: u8 = 0;
const KEY_TONG: u8 = 8;
const KEY_ALL: u8 = 12;

const KEY_BLUE: u8 = 6;
const KEY_RED: u8 = 2;
const KEY_GREEN: u8 = 10;
const KEY_OFF: u8 = 13;
const KEY_YELLOW: u8 = 3;
const KEY_CYAN: u8 = 11;
const KEY_PINK: u8 = 7;
const KEY_WHITE: u8 = 1;
const KEY_HUE: u8 = 9;
const KEY_LOOP: u8 = 5;
const KEY_MIN: u8 = 14;
const KEY_SCARE: u8 = 15;

pub struct Ws2812<'d, P: Instance, const S: usize, const N: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

static KEY_CHAN: StaticCell<Channel<NoopRawMutex, ui::KeyData, 16>> = StaticCell::new();

use fixed::types::U24F8;
use fixed_macro::fixed;
use smart_leds::RGB8;

mod ui;

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
enum LedCol {
    Red,
    Green,
    Blue,
    Cyan,
    Yellow,
    Pink,
    White,
    Minimun,
    Off,
}

impl From<LedCol> for RGB8 {
    fn from(value: LedCol) -> Self {
        match value {
            LedCol::Off => RGB8::new(0x00, 0x00, 0x00),
            LedCol::Red => RGB8::new(0xFF, 0x00, 0x00),
            LedCol::Green => RGB8::new(0x00, 0xFF, 0x00),
            LedCol::Blue => RGB8::new(0x00, 0x00, 0xFF),
            LedCol::Cyan => RGB8::new(0xFF, 0xFF, 0x00),
            LedCol::Yellow => RGB8::new(0x00, 0xFF, 0xFF),
            LedCol::Pink => RGB8::new(0xFF, 0x00, 0xFF),
            LedCol::Minimun => RGB8::new(0x00, 0x01, 0x00),
            LedCol::White => RGB8::new(0xFF, 0xFF, 0xFF),
        }
    }
}

const LVL: u8 = 0x01;
impl From<LedCol> for u32 {
    fn from(value: LedCol) -> Self {
        match value {
            LedCol::Off => u32::from_le_bytes([0xFF, 0x00, 0x00, 0x00]),
            LedCol::Blue => u32::from_le_bytes([0xFF, LVL, 0x00, 0x00]),
            LedCol::Green => u32::from_le_bytes([0xFF, 0x00, LVL, 0x00]),
            LedCol::Red => u32::from_le_bytes([0xFF, 0x00, 0x00, LVL]),
            LedCol::Yellow => u32::from_le_bytes([0xFF, LVL, LVL, 0x00]),
            LedCol::Cyan => u32::from_le_bytes([0xFF, 0x00, LVL, LVL]),
            LedCol::Pink => u32::from_le_bytes([0xFF, LVL, 0x00, LVL]),
            LedCol::Minimun => u32::from_le_bytes([0xFF, 0x01, 0x01, 0x01]),
            LedCol::White => u32::from_le_bytes([0xFF, LVL, LVL, LVL]),
        }
    }
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
enum LedMode {
    Color(LedCol),
    Hue,
    Loop,
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
enum LedGrp {
    EyeLeft,
    EyeRight,
    Tong,
    All,
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
enum Menu {
    Idle,
    WaitForInput(LedGrp),
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    let _bl = Output::new(p.PIN_25, Level::Low);

    // Jaw state
    let jaw_half = Input::new(p.PIN_21, Pull::Up);
    let jaw_full = Input::new(p.PIN_20, Pull::Up);
    let mut jaw = Jaw::new(jaw_half, jaw_full);

    // I2C pins
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let key_int = Input::new(p.PIN_3, Pull::None);

    // SPI pins
    let spi_mosi = p.PIN_19;
    let spi_clk = p.PIN_18;
    let _spi_cs = Output::new(p.PIN_17, Level::Low);

    info!("set up spi");
    let mut spi = Spi::new_txonly(p.SPI0, spi_clk, spi_mosi, p.DMA_CH1, SPI_Config::default());

    // LED buffer with start and end frame
    let mut led_buff: [u32; BUF_NUM] = [
        // First u32 = start
        0x0000_0000, // Led data = 0b111 + 5-bit bright, 3x 8-bit B G R
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        // Last u32 = end
        0xFFFF_FFFF,
    ];

    let buf: [u8; BUF_BYTE_SIZE] = unsafe { core::mem::transmute(led_buff) };
    spi.write(&buf).await.unwrap();

    let keys = KEY_CHAN.init(Channel::new());

    info!("set up i2c keyboard");
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, I2C_Config::default());
    unwrap!(spawner.spawn(ui::ui(i2c, key_int, keys.sender())));

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    const NUM_LEDS: usize = 10 + 3 + 3 + 4;
    let mut data = [LedCol::Off.into(); NUM_LEDS];

    let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_22);

    ws2812.write(&data).await;

    let mut tick = Timer::after_millis(10);

    let mut hue = 0_u8;

    let mut led_modes: [LedMode; 4] = [
        LedMode::Color(LedCol::Off),
        LedMode::Color(LedCol::Off),
        LedMode::Color(LedCol::Off),
        LedMode::Color(LedCol::Off),
    ];

    let mut menu = Menu::Idle;
    let menu_led = LedCol::Off;

    let mut is_scare: bool = false;

    loop {
        match select(&mut tick, keys.receive()).await {
            Either::First(()) => {
                tick = Timer::after_millis(50);

                // Render Keyboard
                match menu {
                    Menu::Idle => {
                        for key_idx in 0..16 {
                            led_buff[1 + key_idx] = menu_led.into();
                        }
                    }
                    Menu::WaitForInput(idx) => {
                        for key_idx in 0..16 {
                            led_buff[1 + key_idx] = menu_led.into();
                        }

                        match idx {
                            LedGrp::EyeLeft => {
                                led_buff[1 + usize::from(KEY_EYELEFT)] = LedCol::White.into()
                            }
                            LedGrp::EyeRight => {
                                led_buff[1 + usize::from(KEY_EYERIGHT)] = LedCol::White.into()
                            }
                            LedGrp::Tong => {
                                led_buff[1 + usize::from(KEY_TONG)] = LedCol::White.into()
                            }
                            LedGrp::All => {
                                led_buff[1 + usize::from(KEY_ALL)] = LedCol::White.into()
                            }
                        }
                        led_buff[1 + usize::from(KEY_RED)] = LedCol::Red.into();
                        led_buff[1 + usize::from(KEY_GREEN)] = LedCol::Green.into();
                        led_buff[1 + usize::from(KEY_BLUE)] = LedCol::Blue.into();
                        led_buff[1 + usize::from(KEY_OFF)] = LedCol::Off.into();
                        led_buff[1 + usize::from(KEY_YELLOW)] = LedCol::Yellow.into();
                        led_buff[1 + usize::from(KEY_CYAN)] = LedCol::Cyan.into();
                        led_buff[1 + usize::from(KEY_PINK)] = LedCol::Pink.into();
                        led_buff[1 + usize::from(KEY_WHITE)] = LedCol::White.into();
                        led_buff[1 + usize::from(KEY_MIN)] = LedCol::Minimun.into();
                        led_buff[1 + usize::from(KEY_OFF)] = LedCol::Minimun.into();
                        led_buff[1 + usize::from(KEY_LOOP)] = LedCol::Red.into();
                    }
                }

                let (md_left, md_right, md_tong) = if JawState::Full == jaw.get_state() || is_scare
                {
                    // println!("Jaw: {}", js);
                    // let col = match js {
                    //     JawState::Closed => RGB8::new(0x00, 0x00, 0xFF),
                    //     JawState::Half => RGB8::new(0x00, 0xFF, 0x00),
                    //     JawState::Full => RGB8::new(0x00, 0x00, 0x00),
                    // };
                    (
                        LedMode::Color(LedCol::Red),
                        LedMode::Color(LedCol::Red),
                        LedMode::Loop,
                    )
                } else {
                    (led_modes[0], led_modes[1], led_modes[2])
                };

                for (idx, led) in (0_u8..).zip(EYE_LEFT) {
                    let col = match md_left {
                        LedMode::Color(col) => col.into(),
                        LedMode::Hue => {
                            let val = hue.wrapping_add(idx);
                            wheel(val)
                        }
                        LedMode::Loop => {
                            if hue % 5 == idx {
                                LedCol::Green.into()
                            } else {
                                LedCol::Off.into()
                            }
                        }
                    };

                    let led_idx = usize::from(led);
                    data[led_idx] = col;
                }

                for (idx, led) in (0_u8..).zip(EYE_RIGHT) {
                    let col = match md_right {
                        LedMode::Color(col) => col.into(),
                        LedMode::Hue => {
                            let val = hue.wrapping_add(idx);
                            wheel(val)
                        }
                        LedMode::Loop => {
                            if hue % 5 == idx {
                                LedCol::Green.into()
                            } else {
                                LedCol::Off.into()
                            }
                        }
                    };

                    let led_idx = usize::from(led);
                    data[led_idx] = col;
                }

                for (idx, led) in (0_u8..).zip(&TONG[0..5]) {
                    let col = match md_tong {
                        LedMode::Color(LedCol::Minimun) => RGB8::new(0x01, 0x00, 0x00),
                        LedMode::Color(col) => col.into(),
                        LedMode::Hue => {
                            let val = hue.wrapping_add(idx);
                            wheel(val)
                        }
                        LedMode::Loop => {
                            if hue % 5 == idx {
                                LedCol::Red.into()
                            } else {
                                LedCol::Off.into()
                            }
                        }
                    };

                    let led_idx = usize::from(*led);
                    data[led_idx] = col;
                }

                for (idx, led) in (0_u8..).zip(TONG[5..10].iter().rev()) {
                    let col = match md_tong {
                        LedMode::Color(LedCol::Minimun) => RGB8::new(0x01, 0x00, 0x00),
                        LedMode::Color(col) => col.into(),
                        LedMode::Hue => {
                            let val = hue.wrapping_add(idx);
                            wheel(val)
                        }
                        LedMode::Loop => {
                            if hue % 5 == idx {
                                LedCol::Red.into()
                            } else {
                                LedCol::Off.into()
                            }
                        }
                    };

                    let led_idx = usize::from(*led);
                    data[led_idx] = col;
                }

                hue = hue.wrapping_add(1);

                ws2812.write(&data).await;
            }
            Either::Second(key) => {
                match key {
                    KeyAction::Pressed(KEY_SCARE) => is_scare = true,
                    KeyAction::Released(KEY_SCARE) => is_scare = false,
                    _ => (),
                }

                if let KeyAction::Released(key_idx) = key {
                    let key_grp = is_key_group(key_idx);
                    let key_col = is_key_color(key_idx);

                    match menu {
                        Menu::Idle => {
                            if let Some(grp) = key_grp {
                                menu = Menu::WaitForInput(grp);
                            };
                        }
                        Menu::WaitForInput(grp) => {
                            if Some(grp) == key_grp {
                                println!("Menu: Reset");
                                menu = Menu::Idle;
                            } else if let Some(col) = key_col {
                                println!("Selected col {}", col);
                                menu = Menu::Idle;
                                match grp {
                                    LedGrp::EyeLeft => {
                                        println!("Left Eye: {}", col);
                                        led_modes[0] = col;
                                    }
                                    LedGrp::EyeRight => led_modes[1] = col,
                                    LedGrp::Tong => led_modes[2] = col,

                                    LedGrp::All => {
                                        for led in &mut led_modes {
                                            *led = col;
                                        }
                                    }
                                }
                            } else {
                                println!("Menu: Invalid select");
                            }
                        }
                    }
                }
            }
        }

        let buf: [u8; BUF_BYTE_SIZE] = unsafe { core::mem::transmute(led_buff) };
        spi.write(&buf).await.unwrap();
    }
}

fn is_key_group(key: u8) -> Option<LedGrp> {
    match key {
        KEY_EYELEFT => Some(LedGrp::EyeLeft),
        KEY_EYERIGHT => Some(LedGrp::EyeRight),
        KEY_TONG => Some(LedGrp::Tong),
        KEY_ALL => Some(LedGrp::All),
        _ => None,
    }
}

fn is_key_color(key: u8) -> Option<LedMode> {
    match key {
        KEY_BLUE => Some(LedMode::Color(LedCol::Blue)),
        KEY_RED => Some(LedMode::Color(LedCol::Red)),
        KEY_GREEN => Some(LedMode::Color(LedCol::Green)),
        KEY_OFF => Some(LedMode::Color(LedCol::Off)),
        KEY_YELLOW => Some(LedMode::Color(LedCol::Yellow)),
        KEY_CYAN => Some(LedMode::Color(LedCol::Cyan)),
        KEY_PINK => Some(LedMode::Color(LedCol::Pink)),
        KEY_MIN => Some(LedMode::Color(LedCol::Minimun)),
        KEY_LOOP => Some(LedMode::Loop),
        KEY_WHITE => Some(LedMode::Color(LedCol::White)),
        KEY_HUE => Some(LedMode::Hue),
        _ => None,
    }
}

#[derive(Debug, Default, PartialEq, Clone, Copy, defmt::Format)]
pub enum JawState {
    #[default]
    Closed,
    Half,
    Full,
}

struct Jaw {
    state: JawState,
    half: Input<'static>,
    full: Input<'static>,
}

impl Jaw {
    pub fn new(half: Input<'static>, full: Input<'static>) -> Self {
        Self {
            state: Default::default(),
            half,
            full,
        }
    }

    pub fn get_state_change(&mut self) -> Option<JawState> {
        let old_state = self.state;
        let new_state = self.get_state();

        if new_state != old_state {
            return Some(new_state);
        }
        None
    }

    pub fn get_state(&mut self) -> JawState {
        let new_state = match (self.full.get_level(), self.half.get_level()) {
            (Level::Low, _) => JawState::Full,
            (_, Level::Low) => JawState::Half,
            (_, _) => JawState::Closed,
        };
        self.state = new_state;
        self.state
    }
}

impl<'d, P: Instance, const S: usize, const N: usize> Ws2812<'d, P, S, N> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl DMA_Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        into_ref!(dma);

        // Setup sm0

        // prepare the PIO program
        let side_set = SideSet::new(false, 1, false);
        let mut a: Assembler<32> = Assembler::new_with_side_set(side_set);

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(::pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(::pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(::pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = PIO_Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[RGB8; N]) {
        // Precompute the word bytes from the colors
        let mut words = [0u32; N];
        for i in 0..N {
            let word = (u32::from(colors[i].g) << 24)
                | (u32::from(colors[i].r) << 16)
                | (u32::from(colors[i].b) << 8);
            words[i] = word;
        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;

        Timer::after_micros(55).await;
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}
