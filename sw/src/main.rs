//! A part of Nordic switch - Bluetooth controlled cord light switch project
//! This firmware and hardware is a work in progress - use at your own risk.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::unwrap;
use embassy_executor::Spawner;
use git_version::git_version;
use heapless::String;

use embassy_nrf::{
    gpio::{self, Pin},
    interrupt, Peripherals,
};
use nrf_softdevice::{
    ble::{self, peripheral as blep},
    generate_adv_data, raw as nrf_defines, Softdevice,
};

use defmt_rtt as _;
use panic_probe as _;

// A structure representing triac output, i.e inverted GPIO pin capable of driving the lamp
struct Triac<'a> {
    out: gpio::Output<'a, gpio::AnyPin>,
}

impl Triac<'_> {
    fn control(&mut self, on: bool) {
        if on {
            self.out.set_low();
        } else {
            self.out.set_high();
        }
    }

    fn new(pin: gpio::AnyPin) -> Self {
        Self {
            out: gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard),
        }
    }
}

// Gatt configuration. A single 'custom' control service containing everything needed.
#[nrf_softdevice::gatt_service(uuid = "c831c2f2-817f-11ee-b962-0242ac120002")]
pub struct ControlService {
    #[characteristic(uuid = "c831c2f2-817f-11ee-b962-0242ac130002", read, write)]
    lamp_control: bool,

    #[characteristic(uuid = "c831c2f2-817f-11ee-b962-0242ac140002", read)]
    git_version: String<32>,
}

#[nrf_softdevice::gatt_server]
pub struct GattServer {
    control: ControlService,
}

// Runs advertisement cycle. Returns connection that we can feed to the gatt_server.
async fn advertise(softdevice: &Softdevice) -> Result<ble::Connection, blep::AdvertiseError> {
    let packet = blep::ConnectableAdvertisement::ScannableUndirected {
        adv_data: generate_adv_data! {
            flags: (GeneralDiscovery, LE_Only),
            short_name: "Nordic Switch"
        },

        scan_data: generate_adv_data! {},
    };

    blep::advertise_connectable(softdevice, packet, &blep::Config::default()).await
}

// Initializes SoftDevice. Does not return errors, only panic
fn init_softdevice() -> &'static mut Softdevice {
    let config = nrf_softdevice::Config {
        // The board is lacking LF crystal because of the space restrictions. Use internal RC network.
        clock: Some(nrf_defines::nrf_clock_lf_cfg_t {
            source: nrf_defines::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: nrf_defines::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        ..Default::default()
    };

    return Softdevice::enable(&config);
}

#[embassy_executor::task]
async fn softdevice_run(softdevice: &'static Softdevice) -> ! {
    softdevice.run().await
}

// Initializes embassy for NRF52.
fn init_embassy() -> Peripherals {
    let mut config = embassy_nrf::config::Config::default();

    // Softdevice implicitly utilizes the highest-level interrupt priority
    // We have to move all other interrupts to lower priority, unless
    // random issues and asserts from the Softdevice may (and will) occur

    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;

    return embassy_nrf::init(config);
}

// Main task
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init_embassy();
    let mut triac = Triac::new(p.P0_11.degrade());

    let softdevice = init_softdevice();
    let server = unwrap!(GattServer::new(softdevice));

    unwrap!(server.control.git_version_set(&git_version!().into()));
    unwrap!(spawner.spawn(softdevice_run(softdevice)));

    loop {
        // Advertisement errors should not occur in the runtime, so it's probably ok to use unwrap
        let connection = unwrap!(advertise(softdevice).await);

        ble::gatt_server::run(&connection, &server, |e| match e {
            GattServerEvent::Control(event) => match event {
                ControlServiceEvent::LampControlWrite(status) => triac.control(status),
            },
        })
        .await;
    }
}
