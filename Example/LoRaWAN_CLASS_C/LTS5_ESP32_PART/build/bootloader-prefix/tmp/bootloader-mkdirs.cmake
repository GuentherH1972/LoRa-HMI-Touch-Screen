# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/espidf_IDE/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/tmp"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/src/bootloader-stamp"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/src"
  "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/main/Example/LoRaWAN_CLASS_C/LTS5_ESP32_PART/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
