use core::slice::from_raw_parts;

use defmt::info;
use rp2040_flash::flash::{flash_range_erase, flash_range_program};

const XIP_BASE: u32 = 0x10000000;
const FLASH_PAGE_SIZE: usize = 1usize << 8;
const FLASH_SECTOR_SIZE: u32 = 1u32 << 12;
const FLASH_BLOCK_SIZE: u32 = 1u32 << 16;
const FLASH_BLOCK_ERASE_CMD: u8 = 0xd8;

const FLASH_TARGET_OFFSET: u32 = 256 * 1024;

pub fn read_from_flash() -> u8 {
    let flash_target_contents = (XIP_BASE + FLASH_TARGET_OFFSET) as *const u8;

    let d = unsafe { from_raw_parts(flash_target_contents, FLASH_PAGE_SIZE) };

    d[0]
}

pub unsafe fn write_to_flash(b: u8) {
    let mut data = [0u8; FLASH_PAGE_SIZE];
    data[0] = b;
    info!("About to write with XIP_BASE: 0x{:x}", XIP_BASE);
    info!("FLASH_PAGE_SIZE is {}", FLASH_PAGE_SIZE);
    info!("FLASH_SECTOR_SIZE is {}", FLASH_SECTOR_SIZE);
    info!("FLASH_BLOCK_SIZE is {}", FLASH_BLOCK_SIZE);
    info!("FLASH_BLOCK_ERASE_CMD is 0x{:x}", FLASH_BLOCK_ERASE_CMD);
    info!("FLASH_TARGET_OFFSET is {}", FLASH_TARGET_OFFSET);

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE, true);
    info!("Erased flash");
    flash_range_program(FLASH_TARGET_OFFSET, &data, true);
    info!("Wrote to flash");
}
