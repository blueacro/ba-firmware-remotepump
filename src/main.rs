#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

pub mod display;
pub mod sm;
pub mod stepper;
pub mod usb;

use sh1106::interface::DisplayInterface;
use stepper::Tim2CC;
use stepper::TimerControl;

use core::cell::RefCell;
use core::fmt::Binary;
use core::ops::DerefMut;

use cortex_m::interrupt::free as int_free;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32f4xx_hal::gpio::{Alternate, Pin, PushPull};

use stm32f4xx_hal::otg_fs::{UsbBus, USB};

use stm32f4xx_hal::pac::{interrupt, Interrupt};

use stm32f4xx_hal::{i2c::I2c, pac, prelude::*};


use sh1106::{prelude::*, Builder};

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*};
struct WrappedDisplay<D: DisplayInterface> {
    display: GraphicsMode<D>
}

impl<D: DisplayInterface> display::FBDisplay for WrappedDisplay<D> {
    fn fbclear(&mut self) {
        self.display.clear()
    }

    fn flush(&mut self) {
        let _ = self.display.flush();
    }
}

impl<D: DisplayInterface> DrawTarget for WrappedDisplay<D> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>> {
        self.display.draw_iter(pixels)
    }
}

impl<D: DisplayInterface> OriginDimensions for WrappedDisplay<D> {
    fn size(&self) -> Size {
        self.display.size()
    }
}



// These lines are part of our setup for debug printing.
use defmt_rtt as _;
use panic_probe as _;

static TIM2CC: Mutex<RefCell<Option<Tim2CC>>> = Mutex::new(RefCell::new(None));

use core::alloc::Layout;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024 * 8;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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
    let mut stepper = TimerControl {
        dir_pin,
        nsleep_pin,
        m1_pin,
        m0_pin,
        en0_pin,
        en1_pin,
        tim2_cc: &TIM2CC,
        driver_enabled: None,
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

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut usb::EP_MEMORY });
    let mut usb_ser = usb::UsbSerialTask::new(&usb_bus);

    let mut dis: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
    dis.init().unwrap();

    let fbdis = WrappedDisplay { display: dis };
    let mut display_stack = display::Stack::new(fbdis);
    display_stack.init();


    let mut delay = cp.SYST.delay(&clocks);

    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    let mut app = sm::App::new(stepper);

    loop {
        app.poll();
        usb_ser.usb_task();
        delay.delay_us(10);
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

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
