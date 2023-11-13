MEMORY
{
  /* These values correspond to the NRF52832 with SoftDevices S132 7.3.0 */
  FLASH : ORIGIN = 0x00000000 + 152K, LENGTH = 256K - 152K
  RAM : ORIGIN = 0x20000000 + 13K, LENGTH = 32K - 13K
}
