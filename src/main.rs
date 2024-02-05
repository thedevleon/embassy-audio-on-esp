#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, Ticker, Instant};
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp32c3_hal::{
    clock::ClockControl,
    dma::{gdma::Gdma, DmaPriority},
    dma_buffers,
    embassy,
    i2s::{asynch::*, DataFormat, I2s, Standard},
    peripherals::Peripherals,
    prelude::*,
    rmt::Rmt,
    IO};
use esp_backtrace as _;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma, SmartLedsWrite, RGB8
};
use rgb::RGBA8;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

const SINE: [i16; 64] = [
    0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
    32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
    15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787, -23169,
    -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30272,
    -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511, -6392, -3211,
];

enum LedAnimation {
    SolidColor(RGBA8), // color
    FadeOnOff(RGBA8, u64), // color, duration
    FadeFromTo(RGBA8, RGBA8, u64), // color1, color2, duration 
    Blink(RGBA8, u64, u64), // color, duration, interval,
    BlinkBurst(RGBA8, u8, u64, u64), // color, bursts, duration, interval
    Off,
}

// LED Animation Queue as https://docs.embassy.dev/embassy-sync/git/default/signal/struct.Signal.html
static LED_ANIMATION_SIGNAL: Signal<CriticalSectionRawMutex, LedAnimation> = Signal::new();

#[embassy_executor::task] // Note: embassy does not yet support generics in tasks
async fn led_animator(mut led: SmartLedsAdapter<esp32c3_hal::rmt::Channel<0>, 0, 25>) {
    let ticker_duration = Duration::from_millis(33); // 30 Hz
    let mut ticker = Ticker::every(ticker_duration.clone());
    let mut start_time: Instant = Instant::now();
    let mut current_led_animation = LedAnimation::SolidColor(RGBA8{r: 0, g: 0, b: 0, a: 0});
    let mut current_color = RGB8{r: 0, g: 0, b: 0};
    let mut current_brightness: u8 = 0;

    loop {
        // get the new animation from the queue if one is available
        if LED_ANIMATION_SIGNAL.signaled()
        {
            current_led_animation = LED_ANIMATION_SIGNAL.wait().await;
        }

        let now = Instant::now();
        
        match current_led_animation {
            LedAnimation::SolidColor(color) => {
                current_color = RGB8{r: color.r, g: color.g, b: color.b};
                current_brightness = color.a;
    
            },
            LedAnimation::FadeOnOff(color, duration) => {
                let mut progress = (now - start_time).as_millis() as f32 / duration as f32;

                // wrap around
                if progress > 1.0 {
                    start_time = now;
                    progress = progress - 1.0;
                }

                current_color = RGB8{r: color.r, g: color.g, b: color.b};
                let alpha = color.a as f32;

                if progress < 0.5 {
                    // Fade On
                    current_brightness = (alpha * (progress * 2.0)) as u8; 
                } else if progress > 0.5 {
                    // Fade Off
                    current_brightness = (alpha * (1.0 - ((progress - 0.5) * 2.0))) as u8;
                }
            },
            LedAnimation::FadeFromTo(color1, color2, duration) => {
                // TODO: fade from color1 to color2 over the duration
            },
            LedAnimation::Blink(color, duration, interval) => {
                // TODO: blink the color over the duration with the interval
            },
            LedAnimation::BlinkBurst(color, bursts, duration, interval) => {
                // TODO: blink the color over the duration with the interval
            },
            LedAnimation::Off => {
                // TODO: turn off the led
            },
        }

        //log::info!("color: {:?}, brightness: {}", current_color, current_brightness);

        led.write(gamma(brightness([current_color].iter().cloned(), current_brightness))).unwrap();

        ticker.next().await;
    }
}


// pass some kind of buffer via SIGNAL
#[embassy_executor::task] // Note: embassy does not yet support generics in tasks
async fn audio_player() {
    // todo
}


#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();

    log::info!("Hello, world!");

    init_heap();

    // This needs the features esp32c6-hal/embassy-time-systick !AND! embassy-time/tick-hz-16_000_000
    embassy::init(
        &clocks,
        esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio8, rmt_buffer);

    spawner.spawn(led_animator(led)).ok();

    // initialize wifi

    // initialize audio player

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(32000, 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        16000u32.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );
    //.with_mclk(io.pins.gpio4);

    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(io.pins.gpio4)
        .with_ws(io.pins.gpio5)
        .with_dout(io.pins.gpio0)
        .build();

    let data =
        unsafe { core::slice::from_raw_parts(&SINE as *const _ as *const u8, SINE.len() * 2) };

    let buffer = tx_buffer;
    let mut idx = 0;
    for i in 0..usize::min(data.len(), buffer.len()) {
        buffer[i] = data[idx];

        idx += 1;

        if idx >= data.len() {
            idx = 0;
        }
    }

    let mut filler = [0u8; 10000];
    let mut idx = 32000 % data.len();

    log::info!("Start");
    let mut transaction = i2s_tx.write_dma_circular_async(buffer).unwrap();
    loop {
        for i in 0..filler.len() {
            filler[i] = data[(idx + i) % data.len()];
        }
        log::info!("Next");

        let written = transaction.push(&filler).await.unwrap();
        idx = (idx + written) % data.len();
        log::info!("written {}", written);
    }


}