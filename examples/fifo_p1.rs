#![no_std]
#![no_main]

// use core::fmt::Write;
use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;

use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    peripherals,
    peripherals::{Peripherals, USB_DEVICE, Interrupt},
    gpio::IO,
    i2c::I2C, 
    prelude::*, 
    timer::TimerGroup, 
    Rtc,
    Delay,
    UsbSerialJtag,
    systimer::SystemTimer,
    //peripherals::{self, Peripherals, USB_DEVICE},
    Cpu,
};


use esp_backtrace as _;
use icm42670::{prelude::*, Address, Icm42670, FifoDataP1, FifoDataSiP1, FifoPacketType};
use shtcx;
use shtcx::PowerMode;

use dcmimu::DCMIMU;

use core::f32::consts::PI;

static USB_SERIAL: Mutex<RefCell<Option<UsbSerialJtag<USB_DEVICE>>>> =
    Mutex::new(RefCell::new(None));

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;
    
    let mut sysT = SystemTimer::new(peripherals.SYSTIMER);

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    
    let mut dcmimu = DCMIMU::new();
    
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        1000u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    //.unwrap();
    
    let bus = shared_bus::BusManagerSimple::new(i2c);

    //let mut icm = Icm42670::new(bus.acquire_i2c(), Address::Primary).unwrap();
    let mut icm = Icm42670::new_fifo(bus.acquire_i2c(), Address::Primary, 
                                     FifoPacketType::Packet1, &mut delay).unwrap();
    let fifo_config1 = icm.readreg(0);
    
    let arange = icm.accel_range().unwrap();
    let ascal = arange.scale_factor();
    
    let mut sht = shtcx::shtc3(bus.acquire_i2c());
    let device_id = sht.device_identifier().unwrap();
    let raw_id = sht.raw_id_register().unwrap();
    
    
    let temperature = sht.measure_temperature(PowerMode::NormalMode, &mut delay).unwrap();
    let humidity = sht.measure_humidity(PowerMode::NormalMode, &mut delay).unwrap();
    let combined = sht.measure(PowerMode::NormalMode, &mut delay).unwrap();
    
    let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
    usb_serial.listen_rx_packet_recv_interrupt();
    critical_section::with(|cs| USB_SERIAL.borrow_ref_mut(cs).replace(usb_serial));
    
    critical_section::with(|cs| { 
        writeln!(
                USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                "T: {:+.04}°C RH: {:+.04}% -- Combined: {} °C / {} %RH",
                temperature.as_degrees_celsius(), 
                humidity.as_percent(),
                combined.temperature.as_degrees_celsius(),
                combined.humidity.as_percent()
        ).ok();
    });
    
    let mut count:u32 = 0;
    
    
    
    interrupt::enable(
        peripherals::Interrupt::USB_SERIAL_JTAG,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::InterruptKind::Edge,
    );
    
    // unsafe {
    //     riscv::interrupt::enable();
    // }
    
    let mut last_now = SystemTimer::now();

    loop {
        let t1 = SystemTimer::now();
        let accel_norm = icm.accel_norm().unwrap();
        let t2 = SystemTimer::now();
        let now = SystemTimer::now();
        let t3 = now;
        let deltaTime = ((now - last_now) as f32)/SystemTimer::TICKS_PER_SECOND as f32;
        last_now = now;
        
        let fifo_cnt = icm.read_fifo_cnt();
        let ts = icm.read_tmst();
        
        let t4 = SystemTimer::now();
        let mut fifodata:[u8;16] = [0;16];
        // icm.read_fifo(0x3F as u8, &mut fifodata);
        icm.read_fifo(&mut fifodata);
        // only continue, if fifo read contains valied data
        // if no data, do nothing
        if (fifodata[0] & 0x80) == 0x80 {
            
        } else { // else evaluate the data and present them every second
            let fdata = FifoDataP1::to_fifodata_raw(&mut fifodata);
            let fdatasi = FifoDataSiP1::to_fifodata_si(&mut fifodata, ascal);
            
            let t5 = SystemTimer::now();
            
            count = count + 1;
            if count == 100 {
                count = 0;
                
                critical_section::with(|cs| {
                    writeln!(
                        USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                        "fifo_cnt:{:?}  T:{:?} ts {:?}  {:?} {:?}",
                        fifo_cnt.unwrap(),
                        icm.temperature().unwrap(),
                        ts.unwrap(),
                        fdata,
                        fdatasi,
                    )
                    .ok();
                    writeln!(
                        USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                        "{:?} t2-t1:{:?} t3-t2:{:?} t4-t3:{:?} t5-t4:{:?}", 
                        deltaTime, // SystemTimer::now()
                        (t2-t1) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                        (t3-t2)  as f32/SystemTimer::TICKS_PER_SECOND as f32,
                        (t4-t3) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                        (t5-t4) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                    )
                    .ok();
                });
            }

        // delay.delay_ms(1u32);
        }
    }
}


#[interrupt]
fn USB_SERIAL_JTAG() {
    critical_section::with(|cs| {
        let mut usb_serial = USB_SERIAL.borrow_ref_mut(cs);
        let usb_serial = usb_serial.as_mut().unwrap();
        writeln!(usb_serial, "USB serial interrupt").unwrap();
        while let nb::Result::Ok(c) = usb_serial.read_byte() {
            writeln!(usb_serial, "Read byte: {:02x}", c).unwrap();
        }
        usb_serial.reset_rx_packet_recv_interrupt();
    });
}
