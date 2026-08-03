# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "Camera_Display.bin"
  "Camera_Display.map"
  "bootloader\\bootloader.bin"
  "bootloader\\bootloader.elf"
  "bootloader\\bootloader.map"
  "config\\sdkconfig.cmake"
  "config\\sdkconfig.h"
  "esp-idf\\esptool_py\\flasher_args.json.in"
  "esp-idf\\mbedtls\\x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "index_gc2145.html.gz.S"
  "index_ov2640.html.gz.S"
  "index_ov3660.html.gz.S"
  "index_ov5640.html.gz.S"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "monitor.html.gz.S"
  "project_elf_src_esp32s3.c"
  "x509_crt_bundle.S"
  )
endif()
