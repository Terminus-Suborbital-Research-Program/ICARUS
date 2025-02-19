#![no_std]
#![no_main]

// Our Modules
pub mod actuators;
pub mod communications;
pub mod phases;
pub mod utilities;
pub mod sensors;

use panic_halt as _;

// HAL Access
use rp235x_hal as hal;

// Monotonics
use rtic_monotonics::rp235x::prelude::*;
rp235x_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

#[rtic::app(
    device = hal::pac,
    dispatchers = [PIO2_IRQ_0, PIO2_IRQ_1, DMA_IRQ_0],
    peripherals = true
)]
mod app {
    use super::*;
    use crate::{
        actuators::{
            PWM2a,
            motor::{MotorXPWM, Motor},
            servo::{EjectionServoMosfet, EjectorServo, Servo},
        },
        communications::{
            hc12::{UART1Bus, GPIO10},
            link_layer::{Device, LinkPacket},
        },
        phases::EjectorStateMachine,
    };

    use bin_packets::{packets::ApplicationPacket, phases::EjectorPhase};

    use communications::{
        link_layer::{LinkLayerDevice, LinkLayerPayload},
        serial_handler::HeaplessString,
        *,
    };

    use icarus::{
        print, println, DelayTimer, I2CMainBus
    };

    use fugit::{Duration, RateExtU32};
    use hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput},
        sio::Sio,
        
    };
    use rp235x_hal::{
        clocks::{init_clocks_and_plls},
        pwm::Slices,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock, Watchdog,
        I2C,
        pac::I2C1,
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::{embedded_io::Write, SerialPort};

    use hc12::{BaudRate, HC12};

    use rtic_sync::{
        arbiter::{i2c::ArbiterDevice, Arbiter},
        channel::{Receiver, Sender}, make_channel
    };
    use serial_handler::{HEAPLESS_STRING_ALLOC_LENGTH, MAX_USB_LINES};

    pub type UART0Bus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    use core::{
        cmp::max,
        mem::MaybeUninit,
        result::Result
    };
    use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode};
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::I2c;
    use embedded_hal::{digital::{OutputPin, StatefulOutputPin}};

    #[shared]
    struct Shared {
        ejector_servo: EjectorServo,
        radio_link: LinkLayerDevice<HC12<UART1Bus, GPIO10>>,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        serial_console_writer: serial_handler::SerialWriter,
        clock_freq_hz: u32,
        state_machine: EjectorStateMachine,
        blink_status_delay_millis: u64,
        motor_x: MotorXPWM,
        // delay_timer: DelayTimer
    }

    #[local]
    struct Local {
        led: gpio::Pin<gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullNone>,
        env_sensor: AsyncBme280<ArbiterDevice<'static, I2CMainBus>, DelayTimer>
    }


    #[init(local=[
        // Task local initialized resources are static
        // Here we use MaybeUninit to allow for initialization in init()
        // This enables its usage in driver initialization
        i2c_main_bus_arbiter: MaybeUninit<Arbiter<I2CMainBus>> = MaybeUninit::uninit(),
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {

        // Reset the spinlocks - this is skipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        // Channel for sending strings to the USB console
        let (usb_console_line_sender, usb_console_line_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

        // Channel for incoming commands from the USB console
        let (usb_console_command_sender, usb_console_command_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

        // Set up clocks
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);

        // The single-cycle I/O block controls our GPIO pins
        let sio = Sio::new(ctx.device.SIO);

        // Set the pins to their default state
        let bank0_pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Configure GPIO25 as an output
        let mut led_pin = bank0_pins
            .gpio25
            .into_pull_type::<PullNone>()
            .into_push_pull_output();
        led_pin.set_low().unwrap();
        // Start the heartbeat task
        heartbeat::spawn().ok();

        // Get clock frequency
        let clock_freq = clocks.peripheral_clock.freq();

        // Pin setup for UART1
        let uart1_pins = (
            bank0_pins.gpio8.into_function(),
            bank0_pins.gpio9.into_function(),
        );
        let mut uart1_peripheral =
            UartPeripheral::new(ctx.device.UART1, uart1_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart1_peripheral.enable_rx_interrupt(); // Make sure we can drive our interrupts


        // Use pin 14 (GPIO10) as the HC12 configuration pin
        let hc12_configure_pin = bank0_pins.gpio10.into_push_pull_output();
        let hc12 = HC12::new(uart1_peripheral, hc12_configure_pin).unwrap();
        let radio_link = LinkLayerDevice::new(hc12, Device::Ejector);

        // Servo
        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);
        let mut ejection_pwm = pwm_slices.pwm0;
        ejection_pwm.enable();
        ejection_pwm.set_div_int(48);
        // Pin for servo mosfet digital
        let mut mosfet_pin: EjectionServoMosfet = bank0_pins.gpio1.into_push_pull_output();
        mosfet_pin.set_low().unwrap();
        let mut channel_a = ejection_pwm.channel_a;
        let channel_pin = channel_a.output_to(bank0_pins.gpio0);
        channel_a.set_enabled(true);
        let ejection_servo = Servo::new(channel_a, channel_pin, mosfet_pin);
        // Create ejector servo
        let mut ejector_servo: EjectorServo = EjectorServo::new(ejection_servo);
        ejector_servo.enable();
        ejector_servo.hold();

        // Motor Initialization
        let mut motor_xy_pwm = pwm_slices.pwm2;
        motor_xy_pwm.enable();
        motor_xy_pwm.set_top(65534/2);
        motor_xy_pwm.set_div_int(1); 
        let mut motor_x_channel: PWM2a = motor_xy_pwm.channel_a;
        let motor_x_channel_pin = motor_x_channel.output_to(bank0_pins.gpio4);
        let mut motor_x = Motor::new(motor_x_channel, motor_x_channel_pin);
        motor_x.set_speed(0);

        // Sensors
        // Init I2C pins
        let sda_pin = bank0_pins.gpio14.reconfigure();
        let scl_pin = bank0_pins.gpio15.reconfigure();

        let i2c_main_bus: I2CMainBus= I2C::new_controller(
            ctx.device.I2C1,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut ctx.device.RESETS,
            clocks.system_clock.freq(),
        );

        let i2c_main_bus_arbiter = ctx.local.i2c_main_bus_arbiter.write(Arbiter::new(i2c_main_bus));
        let mut delay = hal::Timer::new_timer1(ctx.device.TIMER1, &mut ctx.device.RESETS, &clocks); 
        let device_arbiter = ArbiterDevice::new(i2c_main_bus_arbiter);
        // let mut delay = cortex_m::delay::Delay::new(ctx.core.SYST, clocks.system_clock.freq().to_Hz());

        let mut bme280 = AsyncBme280::new(device_arbiter, delay);
        
        // Set up USB Device allocator
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            ctx.device.USB,
            ctx.device.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut ctx.device.RESETS,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        #[allow(static_mut_refs)]
        let usb_bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

        let serial = SerialPort::new(usb_bus_ref);
        let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("UAH TERMINUS PROGRAM")
                .product("Canonical Toolchain USB Serial Port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(2)
            .build();

        // Serial Writer Structure
        let serial_console_writer = serial_handler::SerialWriter::new(usb_console_line_sender);

        usb_serial_console_printer::spawn(usb_console_line_receiver).ok();
        usb_console_reader::spawn(usb_console_command_sender).ok();
        #[cfg(debug_assertions)]
        command_handler::spawn(usb_console_command_receiver).ok();
        radio_flush::spawn().ok();
        //incoming_packet_handler::spawn().ok();
        state_machine_update::spawn().ok();
        // sample_sensors::spawn(i2c_main_bus_arbiter).ok();

        (
            Shared {
                radio_link,
                ejector_servo,
                usb_device: usb_dev,
                usb_serial: serial,
                serial_console_writer,
                clock_freq_hz: clock_freq.to_Hz(),
                state_machine: EjectorStateMachine::new(),
                blink_status_delay_millis: 1000,
                motor_x: motor_x,
                // delay_timer: delay_timer,
            },
            Local { 
                led: led_pin,
                env_sensor: bme280
             },
        )
    }

    // Heartbeats the main led
    #[task(local = [led], shared = [blink_status_delay_millis], priority = 2)]
    async fn heartbeat(mut ctx: heartbeat::Context) {
        loop {
            _ = ctx.local.led.toggle();

            Mono::delay(
                ctx.shared
                    .blink_status_delay_millis
                    .lock(|delay| *delay)
                    .millis(),
            )
            .await;
        }
    }

    // State machine update
    #[task(shared = [state_machine, serial_console_writer, ejector_servo, blink_status_delay_millis], priority = 1)]
    async fn state_machine_update(mut ctx: state_machine_update::Context) {
        loop {
            let wait_time = ctx.shared.state_machine.lock(|state_machine| {
                let wait_time = state_machine.transition();
                wait_time
            });

            match ctx
                .shared
                .state_machine
                .lock(|state_machine| state_machine.phase())
            {
                EjectorPhase::Standby => {
                    // Hold the deployable
                    ctx.shared.ejector_servo.lock(|servo| {
                        servo.hold();
                    });

                    // 1000ms delay
                    ctx.shared
                        .blink_status_delay_millis
                        .lock(|delay| *delay = 1000);
                }

                EjectorPhase::Ejection => {
                    // Eject the deployable
                    ctx.shared.ejector_servo.lock(|servo| {
                        servo.eject();
                    });
                    // 200ms delay
                    ctx.shared
                        .blink_status_delay_millis
                        .lock(|delay| *delay = 200);
                }

                EjectorPhase::Hold => {
                    // Hold the deployable
                    ctx.shared.ejector_servo.lock(|servo| {
                        servo.hold();
                    });
                    // 5000ms delay
                    ctx.shared
                        .blink_status_delay_millis
                        .lock(|delay| *delay = 5000);
                }
            }

            // We should never wait less than 1ms, tbh
            Mono::delay(max(wait_time, 1).millis()).await;
        }
    }

    // Takes care of receiving incoming packets
    #[task(shared = [radio_link, state_machine], priority = 3)]
    async fn incoming_packet_handler(mut ctx: incoming_packet_handler::Context) {
        loop {
            while let Some(packet) = ctx.shared.radio_link.lock(|radio| radio.read_link_packet()) {
                // Only act on packets with valid checksums
                if !packet.verify_checksum() {
                    continue;
                }

                match packet.payload {
                    LinkLayerPayload::Payload(app_packet) => {
                        // Ejector only handles commands right now
                        match app_packet {
                            ApplicationPacket::Command(command) => {
                                // Enter phase, based on the command
                                match command {
                                    bin_packets::packets::CommandPacket::EjectorPhaseSet(phase) => {
                                        ctx.shared.state_machine.lock(|state_machine| {
                                            state_machine.set_phase(phase);
                                        });

                                        // Send a response, with no data (for my testing)
                                        ctx.shared.radio_link.lock(|radio| {
                                            let packet = LinkPacket::default();
                                            radio.write_link_packet(packet).ok();
                                        });
                                    }

                                    // Ping commands return an empty packet
                                    bin_packets::packets::CommandPacket::Ping => {
                                        ctx.shared.radio_link.lock(|radio| {
                                            let packet = LinkPacket::default();
                                            radio.write_link_packet(packet).ok();
                                        });
                                    }

                                    _ => {
                                        // Unhandled command on the Ejector
                                    }
                                }
                            }

                            _ => {}
                        }
                    }

                    _ => {
                        // Link not implimented
                    }
                }
            }

            Mono::delay(10_u64.millis()).await;
        }
    }

    // Updates the radio module on the serial interrupt
    #[task(binds = UART1_IRQ, shared = [radio_link, serial_console_writer])]
    fn uart_interrupt(mut ctx: uart_interrupt::Context) {
        ctx.shared.radio_link.lock(|radio| {
            radio.device.update().ok();
        });
    }

    #[task(priority = 3, shared = [usb_device, usb_serial, serial_console_writer])]
    async fn usb_console_reader(
        mut ctx: usb_console_reader::Context,
        mut command_sender: Sender<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
        let mut buf = [0u8; 64];
        let mut command_buffer = heapless::String::<HEAPLESS_STRING_ALLOC_LENGTH>::new();

        let mut end_of_line = false;

        loop {
            ctx.shared.usb_device.lock(|usb_dev| {
                ctx.shared.usb_serial.lock(|serial| {
                    if usb_dev.poll(&mut [serial]) {
                        // For the moment, we're just going to echo back the input, after a newline
                        match serial.read(&mut buf) {
                            Ok(count) if count > 0 => {
                                // Collect buffer into an array
                                let bytes = &buf[..count];
                                for byte in bytes.iter() {
                                    // Conv to char
                                    let c = *byte as char;

                                    // Write to serial to echo
                                    serial.write(&[*byte]).ok();

                                    // Detect eol
                                    if c == '\r' || c == '\n' {
                                        end_of_line = true;
                                        serial.write_all("\r\n".as_bytes()).ok();
                                    }

                                    if c == '\x08' || c == '\x7F' {
                                        command_buffer.pop();
                                        serial.write_all("\x08 \x08".as_bytes()).ok();
                                    } else {
                                        // Append to buffer
                                        command_buffer.push(c).ok();
                                    }
                                }
                            }

                            _ => {
                                // Ignore errors on read, assume it was just a desync
                            }
                        }
                    }
                })
            });

            if end_of_line {
                end_of_line = false;
                // Send the command to the command handler
                command_sender.try_send(command_buffer.clone()).ok();
                command_buffer.clear();
            }

            // Wait for a bit to poll again
            Mono::delay(1000_u64.micros()).await;
        }
    }

    #[task(priority = 3, shared = [usb_device, usb_serial])]
    async fn usb_serial_console_printer(
        mut ctx: usb_serial_console_printer::Context,
        mut reciever: Receiver<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
        while let Ok(mut line) = reciever.recv().await {
            // If the line ends with a newline, pop it off, and then add a \r\n
            if line.ends_with('\n') {
                line.pop();
                line.push_str("\r\n").ok();
            }

            let mut wr_ptr = line.as_bytes();
            while !wr_ptr.is_empty() {
                let res = ctx.shared.usb_serial.lock(|serial| serial.write(wr_ptr));
                match res {
                    Ok(len) => wr_ptr = &wr_ptr[len..],
                    Err(_) => break,
                }
                Mono::delay(10_u64.millis()).await;
            }
        }
    }

    // Radio Flush Task
    #[task(shared = [radio_link], priority = 1)]
    async fn radio_flush(mut ctx: radio_flush::Context) {
        let mut on_board_baudrate: BaudRate = BaudRate::B9600;
        let bytes_to_flush = 16;

        loop {
            ctx.shared.radio_link.lock(|radio| {
                radio.device.flush(bytes_to_flush).ok();
                on_board_baudrate = radio.device.get_baudrate();
            });

            // Need to wait wait the in-air baudrate, or the on-board baudrate
            // whichever is slower

            let mut slower =
                core::cmp::min(on_board_baudrate.to_u32(), on_board_baudrate.to_in_air_bd());

            // slower is bps, so /1000 to get ms
            slower = slower / 1000;

            // Delay for that times the number of bytes flushed
            Mono::delay((slower as u64 * bytes_to_flush as u64).millis()).await;
        }
    }

    // Command Handler
    #[task(shared=[serial_console_writer, radio_link, clock_freq_hz, ejector_servo, state_machine], priority = 3)]
    async fn command_handler(
        mut ctx: command_handler::Context,
        mut reciever: Receiver<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
        use embedded_io::{Read as _, ReadReady as _};

        while let Ok(line) = reciever.recv().await {
            // Split into commands and arguments, on whitespace
            let mut parts = line.split_whitespace();

            // Get the command
            let command = parts.next().unwrap_or_default();

            match command {
                "usb-reboot" => {
                    // Reboots to the USB bootloader interface
                    println!(ctx, "Rebooting...");

                    hal::reboot::reboot(
                        hal::reboot::RebootKind::BootSel {
                            picoboot_disabled: false,
                            msd_disabled: false,
                        },
                        hal::reboot::RebootArch::Normal,
                    );
                }

                "set-phase-eject" => {
                    // Set the phase to ejection
                    ctx.shared.state_machine.lock(|state_machine| {
                        state_machine.set_phase(EjectorPhase::Ejection);
                    });
                }

                "set-phase-standby" => {
                    // Set the phase to hold
                    ctx.shared.state_machine.lock(|state_machine| {
                        state_machine.set_phase(EjectorPhase::Standby);
                    });
                }

                "phase" => {
                    // Print the current phase
                    let phase = ctx
                        .shared
                        .state_machine
                        .lock(|state_machine| state_machine.phase());

                    println!(ctx, "Current Phase: {:?}", phase);
                }

                "command-eject" => {
                    // Send an ejection command to the ejector
                    let packet = ApplicationPacket::Command(
                        bin_packets::packets::CommandPacket::EjectorPhaseSet(
                            bin_packets::phases::EjectorPhase::Ejection,
                        ),
                    );

                    let link_packet = ctx
                        .shared
                        .radio_link
                        .lock(|radio| radio.construct_packet(packet, Device::Ejector));

                    ctx.shared.radio_link.lock(|radio| {
                        radio.write_link_packet(link_packet).ok();
                    });
                }

                "command-standby" => {
                    // Send a standby command to the ejector
                    let packet = ApplicationPacket::Command(
                        bin_packets::packets::CommandPacket::EjectorPhaseSet(
                            bin_packets::phases::EjectorPhase::Standby,
                        ),
                    );

                    let link_packet = ctx
                        .shared
                        .radio_link
                        .lock(|radio| radio.construct_packet(packet, Device::Ejector));

                    ctx.shared.radio_link.lock(|radio| {
                        radio.write_link_packet(link_packet).ok();
                    });
                }

                // Runs a ping test, pinging Ejector 'n' times
                "ping" => {
                    let n = parts.next().unwrap_or("1").parse::<u32>().unwrap_or(1);
                    let mut recieved = 0;
                    let mut sent = 0;

                    while recieved < n {
                        // Send a ping command
                        let packet =
                            ApplicationPacket::Command(bin_packets::packets::CommandPacket::Ping);
                        let link_packet = ctx
                            .shared
                            .radio_link
                            .lock(|radio| radio.construct_packet(packet, Device::Ejector));

                        println!(ctx, "Sending Ping: {}", sent);
                        ctx.shared.radio_link.lock(|radio| {
                            radio.write_link_packet(link_packet).ok();
                        });
                        sent += 1;
                        println!(ctx, "Sent: {}", sent);

                        // Note the current time
                        let start_time = Mono::now();

                        // Wait for a reply, if we don't get one in 1s, we'll just send another
                        let del: Duration<u64, 1, 1000000> = 1u64.secs();
                        while Mono::now() - start_time < del {
                            // Check for a reply
                            if let Some(_packet) =
                                ctx.shared.radio_link.lock(|radio| radio.read_link_packet())
                            {
                                recieved += 1;
                                println!(ctx, "Recieved: {}", recieved);
                                break;
                            }

                            Mono::delay(10_u64.millis()).await;
                        }
                        Mono::delay(30_u64.millis()).await;
                    }

                    println!(ctx, "Sent: {}, Recieved: {}", sent, recieved);
                    // Percentage of packets recieved
                    Mono::delay(300_u64.millis()).await;
                    println!(ctx, "{}%", (recieved as f32 / sent as f32) * 100.0);
                }

                // Peeks at the buffer, printing it to the console
                "link-peek" => {
                    let buffer = ctx
                        .shared
                        .radio_link
                        .lock(|radio| radio.device.clone_buffer());

                    for c in buffer.iter() {
                        print!(ctx, "{}", *c as char);
                        Mono::delay(10_u64.millis()).await;
                    }
                    println!(ctx, "");
                }

                // HC12 Configuration Utility
                "hc-configure" => {
                    // Clear out the buffer, the HC12 often sends a bit of junk when
                    // it goes into config mode
                    println!(ctx, "Clearing Buffer");
                    ctx.shared.radio_link.lock(|link| {
                        link.device.clear();
                        link.device.write("AT\n".as_bytes()).ok();
                    });

                    Mono::delay(500_u64.millis()).await;

                    println!(ctx, "AT Command Sent");
                    ctx.shared.radio_link.lock(|link| {
                        link.device.update().ok();
                        while link.device.read_ready().unwrap_or(false) {
                            let mut buffer = [0u8; 1];
                            link.device.read(&mut buffer).ok();
                            print!(ctx, "{}", buffer[0] as char);
                        }
                    });

                    // Set baudrate
                    ctx.shared.radio_link.lock(|link| {
                        link.device.write("AT+B9600\n".as_bytes()).ok();
                    });
                    Mono::delay(500_u64.millis()).await;
                    ctx.shared.radio_link.lock(|link| {
                        link.device.update().ok();
                        while link.device.read_ready().unwrap_or(false) {
                            let mut buffer = [0u8; 1];
                            link.device.read(&mut buffer).ok();
                            print!(ctx, "{}", buffer[0] as char);
                        }
                    });

                    // Set channel (100)
                    ctx.shared.radio_link.lock(|link| {
                        link.device.write("AT+C100\n".as_bytes()).ok();
                    });
                    Mono::delay(500_u64.millis()).await;
                    ctx.shared.radio_link.lock(|link| {
                        link.device.update().ok();
                        while link.device.read_ready().unwrap_or(false) {
                            let mut buffer = [0u8; 1];
                            link.device.read(&mut buffer).ok();
                            print!(ctx, "{}", buffer[0] as char);
                        }
                    });

                    // Set power to max (8)
                    ctx.shared.radio_link.lock(|link| {
                        link.device.write("AT+P8\n".as_bytes()).ok();
                    });
                    Mono::delay(500_u64.millis()).await;
                    ctx.shared.radio_link.lock(|link| {
                        link.device.update().ok();
                        while link.device.read_ready().unwrap_or(false) {
                            let mut buffer = [0u8; 1];
                            link.device.read(&mut buffer).ok();
                            print!(ctx, "{}", buffer[0] as char);
                        }
                    });
                }

                "clock-freq" => {
                    // Print the current clock frequency
                    ctx.shared.clock_freq_hz.lock(|freq| {
                        println!(ctx, "Clock Frequency: {} Hz", freq);
                    });
                }

                "sp" => {
                    // Print the stack pointer
                    println!(
                        ctx,
                        "Stack Pointer: 0x{:08X}",
                        utilities::arm::get_stack_pointer()
                    );
                }

                "check" => {
                    // Check and print any incoming packets
                    while let Some(packet) =
                        ctx.shared.radio_link.lock(|radio| radio.read_link_packet())
                    {
                        // Print the packet
                        println!(ctx, "{:?}", packet);

                        Mono::delay(10_u64.millis()).await;
                    }

                    println!(ctx, "Done checking");
                }

                _ => {
                    println!(ctx, "Invalid command: {}", command);
                }
            }
        }
    }

    // #[task(local = [env_sensor], shared=[serial_console_writer, delay_timer], priority = 2)]
    // async fn sample_sensors(mut ctx: sample_sensors::Context, i2c: &'static Arbiter<I2c<'static, I2C1>>) {
    //     let mut motor_x_address = 0x01u8;
    //     let motor_isd_address = 0x80u8;
    //     let mut data = [0,0,0,0];
    //     loop{
    //         ctx.shared.delay_timer.lock(|delay_timer_unlock|{
    //             let measurements = ctx.local.env_sensor.measure(delay_timer_unlock).unwrap();
    //             println!(ctx, "Relative Humidity = {}%", measurements.humidity);
    //             println!(ctx, "Temperature = {} deg C", measurements.temperature);
    //             println!(ctx, "Pressure = {} pascals", measurements.pressure);
    //         });
    //     }
    // }
}
