use core::{
    convert::Infallible,
    fmt::{Binary, Debug},
};

use alloc::boxed::Box;
use alloc::vec::Vec;

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

pub trait Scene {
    fn render<DT: DrawTarget<Color = BinaryColor, Error = Infallible>>(&self, display: &mut DT);
}

pub struct MessageScene {
    pub message: &'static str,
}

impl Scene for MessageScene {
    fn render<DT: DrawTarget<Color = BinaryColor, Error = Infallible>>(&self, display: &mut DT) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(self.message, Point::zero(), text_style, Baseline::Top)
            .draw(display)
            .unwrap();
    }
}

pub struct Stack<F>
where
    F: FBDisplay + DrawTarget,
{
    display: F,
    stack: Vec<Box<dyn Fn(&mut F)>>,
}

impl<F> Stack<F>
where
    F: FBDisplay + DrawTarget<Color = BinaryColor, Error = Infallible>,
{
    pub fn new(f: F) -> Self {
        Self {
            display: f,
            stack: Vec::with_capacity(5),
        }
    }

    pub fn init(&mut self) {
        self.display.fbclear();
        self.display.flush();
    }

    pub fn push<S: Scene + 'static>(&mut self, scene: S) {
        self.stack
            .push(Box::new(move |display: &mut F| scene.render(display)));
    }

    pub fn draw_all(&mut self) {
        self.display.fbclear();
        for scene in self.stack.iter() {
            (scene)(&mut self.display);
        }
        self.display.flush();
    }
}
