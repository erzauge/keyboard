#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] =rp2040_boot2::BOOT_LOADER_W25Q080 ;

use cortex_m::prelude::_embedded_hal_serial_Read;
use rp2040_hal::entry;


use embedded_hal::digital::v2::OutputPin;
// use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Microseconds;



use keyberon::debounce::Debouncer;
use keyberon::matrix::Matrix;

use layout::LAYERS;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp2040_hal::gpio::bank0::*;


// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp2040_hal::pac;
use rp2040_hal::{usb::UsbBus,gpio::dynpin::DynPin};


// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp2040_hal;
use rp2040_hal::pac::interrupt;
use rp2040_hal::timer::{self, Alarm};
use usb_device::device::{UsbDevice, UsbDeviceState};

// extern crate cortex_m_rt;

use core::cell::RefCell;
use cortex_m::interrupt::{Mutex as mutex_CS};

use rtt_target::{rtt_init_print, rprintln};

use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;
use heapless::Vec;
use keyberon;
// mod mutex;
mod layout;
mod events;
/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    rp2040_hal::gpio::Pin<Gpio12, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>,
    rp2040_hal::gpio::Pin<Gpio13, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = rp2040_hal::uart::UartPeripheral<rp2040_hal::uart::Enabled, pac::UART0
, UartPins>;


static mut USB_BUS:Option<UsbBusAllocator<UsbBus>>  = None;
static USB_DEV: mutex_CS<RefCell<Option<UsbDevice<UsbBus>>>> =  mutex_CS::new(RefCell::new(None));
static USB_CLASS: mutex_CS<RefCell<Option<keyberon::Class<UsbBus,()>>>> =  mutex_CS::new(RefCell::new(None));
static ALARM0: mutex_CS<RefCell<Option<timer::Alarm0>>> =  mutex_CS::new(RefCell::new(None));
static ALARM1: mutex_CS<RefCell<Option<timer::Alarm1>>> =  mutex_CS::new(RefCell::new(None));
static MATRIX: mutex_CS<RefCell<Option<Matrix<DynPin,DynPin,5 , 4>>>> = mutex_CS::new(RefCell::new(None));
static DEBOUNC: mutex_CS<RefCell<Option<keyberon::debounce::Debouncer<[[bool;5];4]>>>> = mutex_CS::new(RefCell::new(None));
static EVENTS: mutex_CS<RefCell<Vec<keyberon::layout::Event,72>>> = mutex_CS::new(RefCell::new(Vec::new()));
static LAYOUT: mutex_CS<RefCell<Option<keyberon::layout::Layout<10,4,2,()>>>> = mutex_CS::new(RefCell::new(None));
// static LED : mutex_CS<RefCell<Option<DynPin>>> = mutex_CS::new(RefCell::new(None));
static UART : mutex_CS<RefCell<Option<Uart>>> = mutex_CS::new(RefCell::new(None));
static BUF :mutex_CS<RefCell<[u8;4]>>=mutex_CS::new(RefCell::new([0;4]));
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // rtt_init_print!();
    // rprintln!("1");
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // rprintln!("2");

    let usb_bus = UsbBusAllocator::new( rp2040_hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // rprintln!("3");

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = rp2040_hal::Sio::new(pac.SIO);

    unsafe {
        USB_BUS = Some(usb_bus);
    }
    
    let usb_class = keyberon::new_class(unsafe{USB_BUS.as_ref().unwrap()}, ());
    let usb_dev = keyberon::new_device(unsafe{USB_BUS.as_ref().unwrap()});
    
    cortex_m::interrupt::free(|cs|{
        USB_DEV.borrow(cs).replace(Some(usb_dev));
        USB_CLASS.borrow(cs).replace(Some(usb_class));
    });

    // rprintln!("4");
    
    let mut timer =timer::Timer::new(pac.TIMER,&mut pac.RESETS);
    
    let mut alarm0 = timer.alarm_0().unwrap();
    let mut alarm1 = timer.alarm_1().unwrap();
    alarm0.enable_interrupt();
    alarm0.schedule(Microseconds::new(500)).unwrap();
    alarm1.enable_interrupt();
    alarm1.schedule(Microseconds::new(1000)).unwrap();
    // rprintln!("5");
    cortex_m::interrupt::free(|cs|{
        ALARM0.borrow(cs).replace(Some(alarm0));
        ALARM1.borrow(cs).replace(Some(alarm1));
    });
    
    // Set the pins up according to their function on this particular board
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Set the LED to be an output
    let mut led_pin = pins.gpio19.into_push_pull_output();
    led_pin.set_high().unwrap();
    // rprintln!("6");

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio12.into_mode::<rp2040_hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio13.into_mode::<rp2040_hal::gpio::FunctionUart>(),
    );
    led_pin.set_high().unwrap();
    let mut uart =rp2040_hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS).enable(rp2040_hal::uart::common_configs::_19200_8_N_1, clocks.peripheral_clock.into()).unwrap();
    uart.enable_rx_interrupt();

    // rprintln!("6");
    
    // #[cfg(feature = "left")]
    let matrix : Matrix<DynPin,DynPin,5, 4> = keyberon::matrix::Matrix::new(
        [
            pins.gpio0.into_pull_up_input().into(),
            pins.gpio1.into_pull_up_input().into(),
            pins.gpio2.into_pull_up_input().into(),
            pins.gpio3.into_pull_up_input().into(),
            pins.gpio4.into_pull_up_input().into(),
            ],
        [
            pins.gpio29.into_push_pull_output().into(),
            pins.gpio28.into_push_pull_output().into(),
            pins.gpio27.into_push_pull_output().into(),
            pins.gpio26.into_push_pull_output().into(),
            ],
    ).unwrap();
    
    // #[cfg(feature = "right")]
    // let matrix : Matrix<DynPin,DynPin,5, 4> = keyberon::matrix::Matrix::new(
    //     [
    //         pins.gpio0.into_pull_up_input().into(),
    //         pins.gpio1.into_pull_up_input().into(),
    //         pins.gpio2.into_pull_up_input().into(),
    //         pins.gpio3.into_pull_up_input().into(),
    //         pins.gpio6.into_pull_up_input().into(),
    //         ],
    //     [
    //         pins.gpio29.into_push_pull_output().into(),
    //         pins.gpio28.into_push_pull_output().into(),
    //         pins.gpio27.into_push_pull_output().into(),
    //         pins.gpio26.into_push_pull_output().into(),
    //         ],
    // ).unwrap();
    // rprintln!("7");
    uart.write_full_blocking(b"ein kleinre test was auc immer");
    let debounce = Debouncer::new([[false;5];4],[[false;5];4],30);
    cortex_m::interrupt::free(|cs|{
        DEBOUNC.borrow(cs).replace(Some(debounce));
        MATRIX.borrow(cs).replace(Some(matrix));
        LAYOUT.borrow(cs).replace(Some(keyberon::layout::Layout::new(&LAYERS)));
        // LED.borrow(cs).replace(Some(led_pin.into()));
        UART.borrow(cs).replace(Some(uart));
    });
    // rprintln!("8");

    unsafe {
        pac::NVIC::unmask(rp2040_hal::pac::Interrupt::USBCTRL_IRQ);
        // rprintln!("9a");
        pac::NVIC::unmask(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
        // rprintln!("9b");
        pac::NVIC::unmask(rp2040_hal::pac::Interrupt::TIMER_IRQ_1);
        // rprintln!("9c");
        pac::NVIC::unmask(rp2040_hal::pac::Interrupt::UART0_IRQ);
        // rprintln!("9d");
    };

    // rprintln!("Hello, world!");
    loop { 
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs|{
        // rprintln!("usb interupt");
        if let Some(usb_dev) =USB_DEV.borrow(cs).borrow_mut().as_mut()  {
            if let Some(usb_class) = USB_CLASS.borrow(cs).borrow_mut().as_mut() {
                if usb_dev.poll(&mut [usb_class]) {
                    usb_class.poll();
                }
            }
        }
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs|{
        if let Some(alarm) =ALARM0.borrow(cs).borrow_mut().as_mut()  {
            alarm.clear_interrupt();
            alarm.schedule(Microseconds::new(1000)).unwrap();
        }
        // rprintln!("timer0 interupt");
        let mut m= MATRIX.borrow(cs).borrow_mut();
        if let Some(m)=m.as_mut() {
            let a=m.get();
            if let Some(d) =DEBOUNC.borrow(cs).borrow_mut().as_mut()  {
                if let Some(uart) =UART.borrow(cs).borrow_mut().as_mut() {
                    let mut events =EVENTS.borrow(cs).borrow_mut();
                    for e in d.events(a.unwrap()){
                        let e= events::tans(e);

                        if let Err(_)=events.push(e){
                            events.clear()
                        }
                        // rprintln!("send {:?}",e);
                        uart.write_full_blocking(&events::ser(e));
                    }
                }
            }
        }
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    cortex_m::interrupt::free(|cs|{
        if let Some(alarm) =ALARM1.borrow(cs).borrow_mut().as_mut()  {
            alarm.clear_interrupt();
            alarm.schedule(Microseconds::new(9000)).unwrap();
        }
        // rprintln!("timer1 interupt");
        if let Some(usb_dev) =USB_DEV.borrow(cs).borrow_mut().as_mut() {
            if usb_dev.state() == UsbDeviceState::Configured {
                if let Some(layout) = LAYOUT.borrow(cs).borrow_mut().as_mut(){
                    let mut events =EVENTS.borrow(cs).borrow_mut();
                    while let Some(e)= events.pop() {
                        layout.event(e);
                    }
                    
                    layout.tick();
                    let report: keyberon::key_code::KbHidReport = layout.keycodes().collect();
                    if let Some(class) =USB_CLASS.borrow(cs).borrow_mut().as_mut()  
                    {
                        if class.device_mut().set_keyboard_report(report.clone())
                        {
                            while let Ok(0) = class.write(report.as_bytes()) {}
                        }
                    }
                }
            }
        }

    });
}

#[interrupt]
fn UART0_IRQ(){
    cortex_m::interrupt::free(|cs|{
        if let Some(uart) =UART.borrow(cs).borrow_mut().as_mut() {
            let mut events =EVENTS.borrow(cs).borrow_mut();
            let mut buf = BUF.borrow(cs).borrow_mut();
            // rprintln!("uart interupt");
            while let Ok(b)= uart.read() {
                // rprintln!("resivet: {}",b);
                buf.rotate_left(1);
                buf[3]=b;
                if buf[3] == b'\n'{
                    if let Ok(e)=events::de(buf.as_ref()){
                        if let Err(_)=events.push(e){
                            // rprintln!("recied {:?}",e);
                            events.clear()
                        }
                    }
                }
            }
        }
    });
}