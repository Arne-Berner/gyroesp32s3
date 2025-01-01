use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver, I2cSlaveConfig, I2cSlaveDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let i2c_master = i2c_master_init(
        peripherals.i2c0,
        // peripherals.pins.gpio22.into(), // scl
        // peripherals.pins.gpio21.into(), // sda
        peripherals.pins.gpio14.into(), // scl
        peripherals.pins.gpio9.into(),  // sda
        // peripherals.pins.gpio47.into(), // scl
        // peripherals.pins.gpio48.into(), // sda
        100.kHz().into(),
    )
    .unwrap();

    log::info!("before spawn!");
    let mut sensor = Mpu6050::new(i2c_master, Address::default()).unwrap();
    log::info!("spawned!");
    let mut delay = Delay::new_default();

    sensor.initialize_dmp(&mut delay).unwrap();
    log::info!("initialized!");

    loop {
        match sensor.get_fifo_count() {
            Ok(len) => {
                if len >= 28 {
                    let mut buf = [0; 28];
                    let buf = sensor.read_fifo(&mut buf).unwrap();
                    let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
                    let ypr = YawPitchRoll::from(quat);
                    log::info!("{:.5?}; {:.5?}; {:.5?};", ypr.yaw, ypr.pitch, ypr.roll);
                    //led.toggle();
                }
            }
            Err(_) => sensor.initialize_dmp(&mut delay).unwrap(),
        }
    }
}

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> anyhow::Result<I2cDriver<'d>> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}
