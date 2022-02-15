#![no_std]
#![no_main]
#![feature(default_alloc_error_handler)]
#![allow(clippy::single_match)]

extern crate panic_semihosting;

#[allow(unused_imports)]
#[macro_use]
extern crate alloc;
use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[link_section = ".boot_loader"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(
    device = rp_pico::pac,
    peripherals = true,
    dispatchers = [TIMER_IRQ_0])
]
mod inner_app {
  use alloc::{boxed::Box, vec::Vec};

  use rp_pico::hal::{
    self,
    clocks::{self, ClockSource},
  };
  use rtic::Mutex;
  use systick_monotonic::*;

  #[derive(Default)]
  pub struct MyResource(u32);
  pub trait MyTrait: Send {
    fn my_func(&self, resource_lock: &mut shared_resources::my_resource_lock);
  }
  struct MyStructA;
  struct MyStructB;
  impl MyTrait for MyStructA {
    fn my_func(&self, resource_lock: &mut shared_resources::my_resource_lock) {
      resource_lock.lock(|r| r.0 = 1);
    }
  }
  impl MyTrait for MyStructB {
    fn my_func(&self, resource_lock: &mut shared_resources::my_resource_lock) {
      resource_lock.lock(|r| r.0 = 2);
    }
  }

  // A monotonic timer to enable scheduling in RTIC
  #[monotonic(binds = SysTick, default = true)]
  type MyMono = Systick<100_000>; // frequency in Hz determining granularity

  #[shared]
  struct Shared {
    my_resource: MyResource,
  }

  #[local]
  struct Local {
    trait_objects: Vec<Box<dyn MyTrait>>,
  }

  #[init]
  fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
    let heap_start = cortex_m_rt::heap_start() as usize;
    let heap_size = 200 * 1024;
    unsafe { crate::ALLOCATOR.init(heap_start, heap_size) }

    let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
    let clocks = clocks::init_clocks_and_plls(
      rp_pico::XOSC_CRYSTAL_FREQ,
      ctx.device.XOSC,
      ctx.device.CLOCKS,
      ctx.device.PLL_SYS,
      ctx.device.PLL_USB,
      &mut ctx.device.RESETS,
      &mut watchdog,
    )
    .ok()
    .unwrap();

    let systick = ctx.core.SYST;
    let systick_freq = clocks.system_clock.get_freq().0;
    let mono = Systick::new(systick, systick_freq);

    let my_resource = MyResource::default();
    let my_struct_a = MyStructA;
    let my_struct_b = MyStructB;
    let trait_objects: Vec<Box<dyn MyTrait>> = vec![Box::new(my_struct_a), Box::new(my_struct_b)];

    (
      Shared { my_resource },
      Local { trait_objects },
      init::Monotonics(mono),
    )
  }

  #[task(
        priority = 1,
        shared = [my_resource],
        local = [trait_objects],
    )]
  fn some_task(mut ctx: some_task::Context) {
    for obj in ctx.local.trait_objects {
      obj.my_func(&mut ctx.shared.my_resource);
    }
  }
}
