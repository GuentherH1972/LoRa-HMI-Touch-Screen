# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/espidf_IDE/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "D:/main/Example/Hello_World/build/bootloader"
  "D:/main/Example/Hello_World/build/bootloader-prefix"
  "D:/main/Example/Hello_World/build/bootloader-prefix/tmp"
  "D:/main/Example/Hello_World/build/bootloader-prefix/src/bootloader-stamp"
  "D:/main/Example/Hello_World/build/bootloader-prefix/src"
  "D:/main/Example/Hello_World/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/main/Example/Hello_World/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/main/Example/Hello_World/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
