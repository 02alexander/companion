#![no_std]
#![no_main]
extern crate alloc;
use embassy_executor::Spawner;

use embedded_alloc::TlsfHeap as Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1 << 14;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }
    firmware::entrypoints::test_encoder::entrypoint(spawner).await;
}