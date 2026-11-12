# Third-party code

The files under `third_party/stm32h7/` are vendored from STM32CubeH7:

- `STM32H7xx_HAL_Driver/` — STMicroelectronics HAL driver, BSD-3-Clause.
- `CMSIS_Device/ST/STM32H7xx/` — ST CMSIS device headers and system files,
  Apache-2.0 / BSD-3-Clause as marked in the individual files.
- `CMSIS_Include/` — ARM CMSIS core headers, Apache-2.0.

The `firmware/platform/startup_stm32h723vgtx.s`,
`firmware/platform/system_stm32h7xx.c` and
`firmware/platform/STM32H723VGTX_FLASH.ld` files originate from the
STM32CubeH7-generated projects and retain their ST copyright headers.

Vendored version: STM32CubeH7 package used by the DM-MC02 vendor examples
(`dm02_test_pure`, 2025). Keep the upstream license files when updating.
