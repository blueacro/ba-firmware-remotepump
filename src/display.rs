use core::ops::{Deref, DerefMut};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

/// Some displays need to implement an internal frame-buffer. This trait wraps
/// an internal frame buffer display which can be cleared and flushed/blitted.
pub trait FBDisplay {
    fn fbclear(&mut self);
    fn flush(&mut self);
}

pub struct Stack<F: FBDisplay + DrawTarget<Color = BinaryColor, Error = core::convert::Infallible>> {
    display: F,
}

impl<F: FBDisplay + DrawTarget<Color = BinaryColor, Error = core::convert::Infallible>> Stack<F> {
    pub fn new(f: F) -> Self {
        Self { display: f }
    }

    pub fn init(&mut self) {
        self.display.fbclear();
        self.display.flush();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush();
    }
}
