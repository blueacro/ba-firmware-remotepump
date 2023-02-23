use crate::stepper::Control;

pub enum State {
    Idle,
    SmallWc,
}

pub struct App<C: Control> {
    state: State,
    control: C,
}

impl<C: Control> App<C> {
    pub fn new(control: C) -> Self {
        App {
            state: State::Idle,
            control,
        }
    }

    pub fn poll(&mut self) {}
}
