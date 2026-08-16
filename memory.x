MEMORY
{
    /*
     * Pico 2 has 4096 KiB of QSPI flash.
     *
     * GOLFER deliberately exposes only the first 4032 KiB to the linker. The
     * upper 64 KiB is reserved for persistent application data and to keep our
     * config away from RP2350 end-of-flash boot/update behavior.
     *
     * system.rs currently uses the first two 4 KiB sectors of that reserved
     * region as redundant A/B SystemConfig slots:
     *
     *   0x103F0000 .. 0x103F0FFF  config slot A
     *   0x103F1000 .. 0x103F1FFF  config slot B
     *
     * The final flash sector remains untouched by GOLFER configuration.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4032K

    RAM   : ORIGIN = 0x20000000, LENGTH = 512K

    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS
{
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS
{
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS
{
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
        __flash_binary_end = .;
    } > FLASH
} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
