use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::time::Hertz;

use core::cell::RefCell;

use core::ops::Deref;
use core::ops::DerefMut;

use cortex_m::interrupt::free as int_free;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::hal::digital::v2::OutputPin;

use stm32f4xx_hal::prelude::*;

use core::cmp::Ordering;
use core::convert::Infallible;

#[derive(Clone, Debug)]
pub enum StepperDriver {
    Driver1,
    Driver2,
}
pub struct StepperControl<Dir, NSleepPin, M1Pin, M0Pin, En0Pin, En1Pin> {
    pub dir_pin: Dir,
    pub nsleep_pin: NSleepPin,
    pub m1_pin: M1Pin,
    pub m0_pin: M0Pin,

    pub en0_pin: En0Pin,
    pub en1_pin: En1Pin,
    pub tim2_cc: &'static Mutex<RefCell<Option<Tim2CC>>>,

    pub driver_enabled: Option<StepperDriver>,
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
    pub fn init(&mut self) -> Result<(), Infallible> {
        self.dir_pin.set_low()?;
        self.en0_pin.set_low()?;
        self.en1_pin.set_low()?;
        self.nsleep_pin.set_high()?;
        // 1/16th microsteps
        self.m0_pin.set_high()?;
        self.m1_pin.set_low()?;
        Ok(())
    }

    pub fn enable_driver(
        &mut self,
        driver: &StepperDriver,
    ) -> Result<(), core::convert::Infallible> {
        self.driver_enabled = Some(driver.clone());

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

    pub fn set_speed(&mut self, speed: Hertz) {
        int_free(|cs| {
            if let Some(ref mut tim2) = self.tim2_cc.borrow(cs).borrow_mut().deref_mut() {
                tim2.enable(speed);
            }
        });
    }

    pub fn set_speed_steps(&mut self, speed: Hertz, steps: u64) {
        int_free(|cs| {
            if let Some(ref mut tim2) = self.tim2_cc.borrow(cs).borrow_mut().deref_mut() {
                tim2.enable_steps(speed, steps);
            }
        });
    }

    pub fn is_running(&self) -> bool {
        let mut _run = false;
        int_free(|cs| {
            if let Some(tim2) = self.tim2_cc.borrow(cs).borrow().deref() {
                _run = tim2.running();
            }
        });
        _run
    }

    pub fn disable(&mut self) -> Result<(), Infallible> {
        self.driver_enabled = None;
        self.nsleep_pin.set_low()?;
        self.en0_pin.set_low()?;
        self.en1_pin.set_low()
    }
}

pub struct Tim2CC {
    timer: stm32f4xx_hal::pac::TIM2,
    rate: Hertz,
    current_speed: Hertz,
    target_speed: Hertz,
    steps: Option<u64>,
}

impl Tim2CC {
    pub fn new(timer: stm32f4xx_hal::pac::TIM2, clocks: &Clocks) -> Self {
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
        defmt::info!("tim2 rate is {:?}", rate.raw());
        Self {
            timer,
            rate,
            target_speed: 0_u32.Hz(),
            current_speed: 0_u32.Hz(),
            steps: None,
        }
    }

    pub fn enable(&mut self, rate: Hertz) {
        self.target_speed = rate;
        // Enable interrupt
        self.timer.dier.modify(|_, w| w.cc1ie().set_bit());
        self.handle_int();
        self.timer.cr1.modify(|_, w| w.cen().set_bit());
    }

    pub fn enable_steps(&mut self, rate: Hertz, steps: u64) {
        self.steps = Some(steps);
        self.enable(rate);
    }

    pub fn steps_left(&self) -> Option<u64> {
        self.steps
    }

    pub fn running(&self) -> bool {
        self.timer.cr1.read().cen().is_enabled()
    }

    pub fn handle_int(&mut self) {
        let adjust: Hertz = 20.Hz();
        defmt::trace!(
            "current {:?} target {:?} steps {:?}",
            self.current_speed.raw(),
            self.target_speed.raw(),
            self.steps
        );
        match (self.steps, self.current_speed.cmp(&self.target_speed)) {
            (_, Ordering::Less) => {
                self.current_speed += 20.Hz();
            }
            (_, Ordering::Greater) => {
                self.current_speed = self
                    .current_speed
                    .checked_sub(adjust)
                    .unwrap_or_else(|| 0.Hz());
            }
            (None, _) => {
                // Don't disable the interrupt to account for tracking rotations
                defmt::info!("disabling tim2 interrupt");
                // If we've reached the target speed, disable the interrupt
                self.timer.dier.modify(|_, w| w.cc1ie().clear_bit());
            }
            _ => {}
        }

        //
        // Set a minimum time to allow ramping to not be very slow
        if self.target_speed.to_Hz() != 0 && self.current_speed.to_Hz() < 10 {
            self.current_speed = 100.Hz();
        }

        // Bail if we are actually at 0
        if self.current_speed.to_Hz() == 0 {
            self.timer.cr1.modify(|_, w| w.cen().clear_bit());
            self.timer.sr.modify(|_, w| w.cc1if().clear_bit());
            return;
        }

        let freq_timer: u32 = self.rate.raw();
        let ticks = freq_timer / self.current_speed.raw();
        defmt::trace!("ticks this cycle {}", ticks);

        if let Some(steps_left) = self.steps {
            if steps_left <= 1 {
                self.target_speed = 0.Hz();
                self.steps = None;
            } else {
                self.steps = Some(steps_left - 1)
            }
        }

        self.timer.arr.write(|w| w.arr().bits(ticks));
        self.timer.ccr1().write(|w| w.ccr().bits(ticks));
        self.timer.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}
