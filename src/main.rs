#![no_std]
#![no_main]

// use core::fmt::Write;
use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;

use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac,
    pac::{Peripherals, USB_DEVICE, Interrupt},
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
use icm42670::{prelude::*, Address, Icm42670, FifoDataP3, FifoDataSiP3, FifoPacketType};
use shtcx;
use shtcx::PowerMode;

use dcmimu::DCMIMU;

use core::f32::consts::PI;

static USB_SERIAL: Mutex<RefCell<Option<UsbSerialJtag<USB_DEVICE>>>> =
    Mutex::new(RefCell::new(None));

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
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
                                     FifoPacketType::Packet3, &mut delay).unwrap();
    //icm.set_fifo_mode(&mut delay);
    let fifo_config1 = icm.readreg(0);
    
    let arange = icm.accel_range().unwrap();
    let ascal = arange.scale_factor();
    let grange = icm.gyro_range().unwrap();
    let gscal = grange.scale_factor();
    
    icm.ped_ena(true);
    
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
        pac::Interrupt::USB_SERIAL_JTAG,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::InterruptKind::Edge,
    );
    
    unsafe {
        riscv::interrupt::enable();
    }
    
    let mut last_now = SystemTimer::now();

    loop {
        let t1 = SystemTimer::now();
        let accel_norm = icm.accel_norm().unwrap();
        let t2 = SystemTimer::now();
        let gyro_norm = icm.gyro_norm().unwrap();
        let t2_2 = SystemTimer::now();
        //let gyro_rap = icm.gyro_raw().unwrap();
        //let fifo_config5 = icm.read_mreg1(&mut delay).unwrap();
        let now = SystemTimer::now();
        let t3 = now;
        let deltaTime = ((now - last_now) as f32)/SystemTimer::TICKS_PER_SECOND as f32;
        //let t4 = SystemTimer::now();
        last_now = now;
        
        let fifo_cnt = icm.read_fifo_cnt();
        let ped_cnt = icm.read_ped_cnt();
        let ts = icm.read_tmst();
        
        let t4 = SystemTimer::now();
        let mut fifodata:[u8;16] = [0;16];
        icm.read_fifo(0x3F as u8, &mut fifodata);
        let fdata = FifoDataP3::to_fifodata_raw(&mut fifodata);
        let fdatasi = FifoDataSiP3::to_fifodata_si(&mut fifodata, ascal, gscal);
        
        //for i in fifodata.iter_mut() {
        //    *i = icm.readreg(0x3F as u8).unwrap();
        //    //fifodata[i] = reg;
        //}
        
        
        //let (dcm, _gyro_biases) = dcmimu.update(((gyro_norm.x*PI/180.0), (gyro_norm.y*PI/180.0), (gyro_norm.z*PI/180.0)),
        //                                    (accel_norm.x, accel_norm.y, accel_norm.z),
        //                                    deltaTime as f32);
        let (dcm, _gyro_biases) = dcmimu.update(((fdatasi.gx), (fdatasi.gy), (fdatasi.gz)),
                                            (fdatasi.ax, fdatasi.ay, fdatasi.az),
                                            deltaTime as f32);
        let t5 = SystemTimer::now();
        //critical_section::with(|cs| {
        //    writeln!(
        //        UsbSerialJtag,
        //        "ACCEL  =  X: {:+.04} Y: {:+.04} Z: {:+.04}\t\tGYRO  =  X: {:+.04} Y: {:+.04} Z: {:+.04}",
        //        accel_norm.x, accel_norm.y, accel_norm.z, gyro_norm.x, gyro_norm.y, gyro_norm.z
        //    ).ok();
        //});
        
        count = count + 1;
        if count == 1 {
            count = 0;
            //writeln!(
            //    USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                //"Roll: {}; yaw: {}; pitch: {}  {:?} == {}, {}, {}", 
            //    "{:?}",
                //dcm.roll, dcm.yaw, dcm.pitch,
            //    dcmimu.all(), 
                //dcmimu.roll(), dcmimu.yaw(), dcmimu.pitch()
                //"ACCEL  =  X: {:+.04} Y: {:+.04} Z: {:+.04}\t\tGYRO  =  X: {:+.04} Y: {:+.04} Z: {:+.04}",
                //accel_norm.x, accel_norm.y, accel_norm.z, gyro_norm.x, gyro_norm.y, gyro_norm.z
            //).ok();
            
            critical_section::with(|cs| {
                writeln!(
                    USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                    //"Hello world!"
                    //"T(icm):{:?}°C {:?}",
                    "fifo_cnt:{:?} ped_cnt: {:?} {:?} ts {:?} accel_norm:{:?} {:?} {:?} {:?}",
                    fifo_cnt.unwrap(),
                    ped_cnt.unwrap(),
                    //fifo_config1,
                    //fifo_config5,
                    icm.temperature().unwrap(),
                    //dcmimu.all()
                    ts.unwrap(),
                    accel_norm,
                    gyro_norm,
                   fdata, // fifodata,
                   fdatasi,
                )
                .ok();
                writeln!(
                    USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                    //"Hello world!"
                    "{:?} t2-t1:{:?} t2_2-t2:{:?} t3-t2_2:{:?} t4-t3:{:?} t5-t4:{:?}", 
                    deltaTime, // SystemTimer::now()
                    (t2-t1) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                    (t2_2-t2)  as f32/SystemTimer::TICKS_PER_SECOND as f32,
                    (t3-t2_2) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                    (t4-t3) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                    (t5-t4) as f32/SystemTimer::TICKS_PER_SECOND as f32,
                )
                .ok();
            });
        }

        // delay.delay_ms(1u32);
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
