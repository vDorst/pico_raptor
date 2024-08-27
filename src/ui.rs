use defmt::{error, info, println};
use embassy_executor::task;
use embassy_rp::{
    gpio::Input,
    i2c::{Async, I2c},
    peripherals::I2C0,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Sender};
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as _;

pub(crate) mod tca9555 {
    pub const ADDR: u8 = 0x20;

    #[allow(unused)]
    #[repr(u8)]
    pub enum Reg {
        InputP0 = 0x00,
        InputP1,
        OutputP0,
        OutputP1,
        PolInvP0,
        PolInvP1,
        CfgP0,
        CfgP1,
    }
}

#[derive(Debug, defmt::Format, PartialEq)]
pub enum KeyAction {
    Released(u8),
    Pressed(u8),
}

pub type KeyData = KeyAction;

#[task]
pub async fn ui(
    mut i2c: I2c<'static, I2C0, Async>,
    mut key_int: Input<'static>,
    keys: Sender<'static, NoopRawMutex, KeyData, 16>,
) {
    let mut i2c_input: [u8; 2] = [0xFF, 0xFF];
    let mut old: u16 = 0xFFFF;

    info!("init tca9555 config for keyboard");
    // // All pins as input
    i2c.write(tca9555::ADDR, &[tca9555::Reg::CfgP0 as u8, 0xFF, 0xFF])
        .await
        .unwrap();

    loop {
        key_int.wait_for_low().await;
        // Debounce
        Timer::after_millis(10).await;

        // Read key_status
        let _ = i2c
            .write_read_async(
                u16::from(tca9555::ADDR),
                [tca9555::Reg::InputP0 as u8],
                &mut i2c_input,
            )
            .await;

        let inp: u16 = u16::from_ne_bytes(i2c_input);

        let diff = inp ^ old;
        for bit in 0..16_u16 {
            if diff & (1 << bit) != 0x00 {
                let data = u8::try_from(bit & 0xFF).unwrap();
                // | if inp & (1 << bit) == 0x00 { 0x80 } else { 0x00 };

                let is_pressed = inp & (1 << bit) == 0x00;
                let key_data = if is_pressed {
                    KeyAction::Pressed(data)
                } else {
                    KeyAction::Released(data)
                };
                println!("Key {} press {}", data & 0xF, is_pressed);
                if keys.try_send(key_data).is_err() {
                    error!("Error Send");
                };
            }
        }

        old = inp;
    }
}
