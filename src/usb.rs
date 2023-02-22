pub static mut EP_MEMORY: [u32; 1024] = [0; 1024];

use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{i2c::I2c, pac, prelude::*};

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

pub struct UsbSerialTask<'a> {
    serial: SerialPort<'a, UsbBus<USB>>,
    usb_dev: UsbDevice<'a, UsbBus<USB>>,
}

impl<'a> UsbSerialTask<'a> {
    pub fn new(usb_bus: &'a UsbBusAllocator<UsbBus<USB>>) -> Self {
        let serial = usbd_serial::SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("blueAcro")
            .product("RemotePump")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        Self { serial, usb_dev }
    }
    pub fn usb_task(&mut self) {
        if !self.usb_dev.poll(&mut [&mut self.serial]) {
            return;
        }

        let mut buf = [0u8; 64];

        match self.serial.read(&mut buf) {
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
                    match self.serial.write(&buf[write_offset..count]) {
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
