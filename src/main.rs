//! # Pico Blinky Example tast
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]


// use cortex_m::interrupt;
// use alloc::vec::Vec;
// The macro for our start-up function
use cortex_m_rt::entry;

use defmt::info;
use embedded_hal::digital::v2::InputPin;
// GPIO traits
use embedded_hal::digital::v2::OutputPin;

use embedded_time::duration::Microseconds;

// Time handling traits
use embedded_time::rate::*;

use keyberon::debounce;
use keyberon::debounce::Debouncer;
use keyberon::layout::Event;
use keyberon::matrix::Matrix;

use layout::LAYERS;
use mutex_trait::Mutex;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::bank0::Gpio25;
// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;
use rp_pico::hal::{usb::UsbBus,gpio::dynpin::DynPin};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::pac::interrupt;
use hal::timer;
use usb_device::class;
use usb_device::device::UsbDevice;

use core::borrow::Borrow;
use core::borrow::BorrowMut;
use core::cell::RefCell;
use core::pin::Pin;
use cortex_m::interrupt::Mutex as mutex_CS;


use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
use heapless::Vec;
use keyberon;
mod mutex;
mod layout;

static mut USB_BUS:Option<UsbBusAllocator<UsbBus>>  = None;
// static USB_BUS:Mutex<RefCell<Option<UsbBusAllocator<UsbBus>>>> = Mutex::new(RefCell::new(None));
static USB_DEV: mutex_CS<RefCell<Option<UsbDevice<UsbBus>>>> =  mutex_CS::new(RefCell::new(None));
static USB_CLASS: mutex_CS<RefCell<Option<keyberon::Class<UsbBus,()>>>> =  mutex_CS::new(RefCell::new(None));
static ALARM0: mutex_CS<RefCell<Option<timer::Alarm0>>> =  mutex_CS::new(RefCell::new(None));
static ALARM1: mutex_CS<RefCell<Option<timer::Alarm1>>> =  mutex_CS::new(RefCell::new(None));
static MATRIX: mutex_CS<RefCell<Option<Matrix<DynPin,DynPin,5 , 4>>>> = mutex_CS::new(RefCell::new(None));
static DEBOUNC: mutex_CS<RefCell<Option<keyberon::debounce::Debouncer<[[bool;5];4]>>>> = mutex_CS::new(RefCell::new(None));
static EVENTS: mutex_CS<RefCell<Vec<keyberon::layout::Event,72>>> = mutex_CS::new(RefCell::new(Vec::new()));
static LAYOUT: mutex_CS<RefCell<Option<keyberon::layout::Layout<10,4,1,()>>>> = mutex_CS::new(RefCell::new(None));
static LED : mutex_CS<RefCell<Option<DynPin>>> = mutex_CS::new(RefCell::new(None));
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new( rp_pico::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    unsafe {
        USB_BUS = Some(usb_bus);
    }
    
    let usb_class = keyberon::new_class(unsafe{USB_BUS.as_ref().unwrap()}, ());
    let usb_dev = keyberon::new_device(unsafe{USB_BUS.as_ref().unwrap()});
    
    cortex_m::interrupt::free(|cs|{
        USB_DEV.borrow(cs).replace(Some(usb_dev));
        USB_CLASS.borrow(cs).replace(Some(usb_class));
    });

    
    let mut timer =timer::Timer::new(pac.TIMER,&mut pac.RESETS);
    
    let mut alarm0 = timer.alarm_0().unwrap();
    let mut alarm1 = timer.alarm_1().unwrap();
    alarm0.enable_interrupt();
    alarm0.schedule(Microseconds::new(500)).unwrap();
    alarm1.enable_interrupt();
    alarm1.schedule(Microseconds::new(1000)).unwrap();
    
    cortex_m::interrupt::free(|cs|{
        ALARM0.borrow(cs).replace(Some(alarm0));
        ALARM1.borrow(cs).replace(Some(alarm1));
    });
    
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        pac::NVIC::unmask(hal::pac::Interrupt::TIMER_IRQ_0);
        pac::NVIC::unmask(hal::pac::Interrupt::TIMER_IRQ_1);
    };
    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Set the LED to be an output
    let led_pin = pins.led.into_push_pull_output();
    // let mut key_out = pins.gpio3.into_push_pull_output();
    // let key_in = pins.gpio8.into_floating_input();
    // Blink the LED at 1 Hz
    // key_out.set_high().unwrap();

    let matrix : Matrix<DynPin,DynPin,5, 4> = keyberon::matrix::Matrix::new(
        [
            pins.gpio12.into_pull_up_input().into(),
            pins.gpio11.into_pull_up_input().into(),
            pins.gpio10.into_pull_up_input().into(),
            pins.gpio9.into_pull_up_input().into(),
            pins.gpio8.into_pull_up_input().into(),
            ],
        [
            pins.gpio0.into_push_pull_output().into(),
            pins.gpio1.into_push_pull_output().into(),
            pins.gpio2.into_push_pull_output().into(),
            pins.gpio3.into_push_pull_output().into()
            ],
        ).unwrap();

    let debounce = Debouncer::new([[false;5];4],[[false;5];4],30);
    cortex_m::interrupt::free(|cs|{
        DEBOUNC.borrow(cs).replace(Some(debounce));
        MATRIX.borrow(cs).replace(Some(matrix));
        LAYOUT.borrow(cs).replace(Some(keyberon::layout::Layout::new(&LAYERS)));
        LED.borrow(cs).replace(Some(led_pin.into()));
    });
    loop { 
        delay.delay_ms(20);
        // info!("test");
        cortex_m::interrupt::free(|cs|{
            if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
                led.set_high().unwrap();  
            } 
        });

        delay.delay_ms(20);
        cortex_m::interrupt::free(|cs|{
            if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
                led.set_low().unwrap();  
            } 
        });
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs|{
        if let Some(usb_dev) =USB_DEV.borrow(cs).borrow_mut().as_mut()  {
            if let Some(usb_class) = USB_CLASS.borrow(cs).borrow_mut().as_mut() {
                if usb_dev.poll(&mut [usb_class]) {
                    usb_class.poll();
                }
                // let mut led = LED.borrow(cs).borrow_mut();
                // if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
                //     led.set_high();  
                // } 
            }
        }
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs|{
        // if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
        //     led.set_high();  
        // } 
        if let Some(alarm) =ALARM0.borrow(cs).borrow_mut().as_mut()  {
            alarm.clear_interrupt();
            alarm.schedule(Microseconds::new(500)).unwrap();
        }
        let mut m= MATRIX.borrow(cs).borrow_mut();
        if let Some(m)=m.as_mut() {
            let a=m.get();
            if let Some(d) =DEBOUNC.borrow(cs).borrow_mut().as_mut()  {
                let mut events =EVENTS.borrow(cs).borrow_mut();
                for e in d.events(a.unwrap()){
                    events.push(e).unwrap();
                    // if let Err(_) = events.push(e) {
                    //     if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
                    //         led.set_high().unwrap();  
                    //     } 
                    // }
                }
            }
        }
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    cortex_m::interrupt::free(|cs|{
        // if let Some(led) =LED.borrow(cs).borrow_mut().as_mut(){
        //     led.set_high();  
        // } 
        if let Some(alarm) =ALARM1.borrow(cs).borrow_mut().as_mut()  {
            alarm.clear_interrupt();
            alarm.schedule(Microseconds::new(1000)).unwrap();
        }

        let mut events =EVENTS.borrow(cs).borrow_mut();
        if let Some(layout) = LAYOUT.borrow(cs).borrow_mut().as_mut(){
            while let Some(e)= events.pop() {
                layout.event(e);
            }
            
            layout.tick();
            let report: keyberon::key_code::KbHidReport = layout.keycodes().collect();
            if let Some(class) =USB_CLASS.borrow(cs).borrow_mut().as_mut()  
            {
                if class.device_mut().set_keyboard_report(report.clone())
                {
                    // return;
                    while let Ok(0) = class.write(report.as_bytes()) {}
                }
            }
        }

    });
}
