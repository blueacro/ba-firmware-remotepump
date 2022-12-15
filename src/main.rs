#![no_std]
#![no_main]

use core::cell::RefCell;
use core::convert::Infallible;
use core::ops::DerefMut;

use cortex_m::interrupt::free as int_free;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32f4xx_hal::gpio::{Alternate, Pin, PushPull};
use stm32f4xx_hal::hal::digital::v2::OutputPin;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::pac::{interrupt, Interrupt};
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::time::Hertz;
use stm32f4xx_hal::{i2c::I2c, pac, prelude::*};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use sh1106::{prelude::*, Builder};

use usb_device::prelude::*;

// These lines are part of our setup for debug printing.
use defmt_rtt as _;
use panic_probe as _;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static TIM2CC: Mutex<RefCell<Option<Tim2CC>>> = Mutex::new(RefCell::new(None));

enum StepperDriver {
    Driver1,
    Driver2,
}
struct StepperControl<Dir, NSleepPin, M1Pin, M0Pin, En0Pin, En1Pin> {
    dir_pin: Dir,
    nsleep_pin: NSleepPin,
    m1_pin: M1Pin,
    m0_pin: M0Pin,

    en0_pin: En0Pin,
    en1_pin: En1Pin,
    tim2_cc: &'static Mutex<RefCell<Option<Tim2CC>>>,
}

impl<Dir, NSleep, M1, M0, En0, En1> StepperControl<Dir, NSleep, M1, M0, En0, En1>
where
    Dir: OutputPin<Error = Infallible>,
    NSleep: OutputPin<Error = Infallible>,
    M1: OutputPin<Error = Infallible>,
    M0: OutputPin<Error = Infallible>,
    En0: OutputPin<Error = Infallible>,
    En1: OutputPin<Error = Infallible>,
{
    fn init(&mut self) -> Result<(), Infallible> {
        self.dir_pin.set_low()?;
        self.en0_pin.set_low()?;
        self.en1_pin.set_low()?;
        self.nsleep_pin.set_high()?;
        // 1/16th microsteps
        self.m0_pin.set_high()?;
        self.m1_pin.set_low()?;
        Ok(())
    }

    fn enable_driver(&mut self, driver: StepperDriver) -> Result<(), core::convert::Infallible> {
        self.nsleep_pin.set_high()?;

        match driver {
            StepperDriver::Driver1 => {
                self.en1_pin.set_low()?;
                self.dir_pin.set_low()?;
                self.en0_pin.set_high()?;
            }
            StepperDriver::Driver2 => {
                self.en0_pin.set_low()?;
                self.dir_pin.set_high()?;
                self.en1_pin.set_high()?;
            }
        }
        Ok(())
    }

    fn set_speed(&mut self, speed: Hertz) {
        int_free(|cs| {
            if let Some(ref mut tim2) = self.tim2_cc.borrow(cs).borrow_mut().deref_mut() {
                tim2.enable(speed);
            }
        });
    }

    fn disable(&mut self) -> Result<(), Infallible> {
        self.nsleep_pin.set_low()?;
        self.en0_pin.set_low()?;
        self.en1_pin.set_low()
    }
}

struct Tim2CC {
    timer: stm32f4xx_hal::pac::TIM2,
    rate: Hertz,
    current_speed: Hertz,
    target_speed: Hertz,
}

impl Tim2CC {
    fn new(timer: stm32f4xx_hal::pac::TIM2, clocks: &Clocks) -> Self {
        use stm32f4xx_hal::rcc::BusTimerClock;
        unsafe {
            use stm32f4xx_hal::rcc::Enable;
            use stm32f4xx_hal::rcc::Reset;
            let rcc = &(*stm32f4xx_hal::pac::RCC::ptr());
            stm32f4xx_hal::pac::TIM2::enable(rcc);
            stm32f4xx_hal::pac::TIM2::reset(rcc);
        }
        timer.psc.modify(|_, w| w.psc().bits(0_u16));
        timer.ccmr1_output().write(|w| w.oc1m().toggle());
        timer.ccer.modify(|_, w| w.cc1e().set_bit());

        let rate = stm32f4xx_hal::pac::TIM2::timer_clock(clocks);
        Self {
            timer,
            rate,
            target_speed: 0_u32.Hz(),
            current_speed: 0_u32.Hz(),
        }
    }

    fn enable(&mut self, rate: Hertz) {
        self.target_speed = rate;
        // Enable interrupt
        self.timer.dier.modify(|_, w| w.cc1ie().set_bit());
        self.handle_int();
        self.timer.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn handle_int(&mut self) {
        let adjust: Hertz = 20.Hz();
        if self.current_speed < self.target_speed {
            self.current_speed += 20.Hz();
        } else if self.current_speed > self.target_speed {
            self.current_speed = self
                .current_speed
                .checked_sub(adjust)
                .unwrap_or_else(|| 0.Hz());
        } else {
            // If we've reached the target speed, disable the interrupt
            self.timer.dier.modify(|_, w| w.cc1ie().clear_bit());
        }
        // Set a minimum time to allow ramping to not be very slow
        if self.target_speed.to_Hz() != 0 && self.current_speed.to_Hz() < 10 {
            self.current_speed = 100.Hz();
        }
        if self.current_speed.to_Hz() == 0 {
            self.timer.cr1.modify(|_, w| w.cen().clear_bit());
            self.timer.sr.modify(|_, w| w.cc1if().clear_bit());
            return;
        }
        let freq_timer: u32 = self.rate.raw();
        let ticks = (freq_timer / self.current_speed.raw());
        self.timer.arr.write(|w| w.arr().bits(ticks));
        self.timer.ccr1().write(|w| w.ccr().bits(ticks));
        self.timer.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}

#[interrupt]
fn TIM2() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
    int_free(|cs| {
        if let Some(ref mut tim2) = TIM2CC.borrow(cs).borrow_mut().deref_mut() {
            tim2.handle_int();
        }
    });
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(12.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let mut led1 = gpiob.pb14.into_push_pull_output();
    let mut led2 = gpiob.pb15.into_push_pull_output();

    led1.set_high();
    led2.set_low();

    // Stepper control
    let dir_pin = gpioa.pa4.into_push_pull_output();
    let _step_pin: Pin<'A', 5, Alternate<1, PushPull>> = gpioa.pa5.into_alternate();
    let en0_pin = gpioa.pa6.into_push_pull_output();
    let en1_pin = gpioa.pa7.into_push_pull_output();
    let nsleep_pin = gpioa.pa8.into_push_pull_output();
    let m1_pin = gpiob.pb0.into_push_pull_output();
    let m0_pin = gpiob.pb1.into_push_pull_output();

    let mut tim2 = Tim2CC::new(dp.TIM2, &clocks);
    int_free(|cs| TIM2CC.borrow(cs).replace(Some(tim2)));
    let mut stepper = StepperControl {
        dir_pin,
        nsleep_pin,
        m1_pin,
        m0_pin,
        en0_pin,
        en1_pin,
        tim2_cc: &TIM2CC,
    };

    stepper.init().unwrap();

    let scl = gpiob
        .pb8
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    let sda = gpiob
        .pb7
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

    let mut display: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

    display.init().unwrap();
    display.clear();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("blueAcro")
        .product("RemotePump")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut delay = cp.SYST.delay(&clocks);
    let mut flip = false;

    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    stepper.enable_driver(StepperDriver::Driver2).unwrap();

    loop {
        stepper.set_speed(400.Hz());
        delay.delay_ms(1000_u32);
        stepper.set_speed(800.Hz());
        delay.delay_ms(1000_u32);
        stepper.set_speed(400.Hz());
        delay.delay_ms(1000_u32);

        stepper.set_speed(100.Hz());
        delay.delay_ms(1000_u32);
        stepper.set_speed(0.Hz());
        delay.delay_ms(1000_u32);
        if flip {
            stepper.enable_driver(StepperDriver::Driver2).unwrap();
            flip = false;
        } else {
            stepper.enable_driver(StepperDriver::Driver1).unwrap();
            flip = true;
        }
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                defmt::println!("{}", buf);
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}

#[interrupt]
fn OTG_FS() {}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
