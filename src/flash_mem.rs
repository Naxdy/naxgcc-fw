use core::slice::from_raw_parts;

use rp2040_flash::flash::{flash_range_erase, flash_range_program};

const XIP_BASE: u32 = 0x10000000;
const FLASH_PAGE_SIZE: usize = 1usize << 8;
const FLASH_SECTOR_SIZE: u32 = 1u32 << 12;

const FLASH_TARGET_OFFSET: u32 = 256 * 1024;

pub fn read_from_flash() -> u8 {
    let flash_target_contents = (XIP_BASE + FLASH_TARGET_OFFSET) as *const u8;

    let d = unsafe { from_raw_parts(flash_target_contents, FLASH_PAGE_SIZE) };

    d[0]
}

pub unsafe fn write_to_flash(b: u8) {
    let mut data = [0u8; FLASH_PAGE_SIZE];
    data[0] = b;

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE, true);
    flash_range_program(FLASH_TARGET_OFFSET, &data, true);
}
