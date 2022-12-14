cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)


set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PATH C:/Keil_v5/ARM/ARMCLANG)
find_program(ARM_CC NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_CXX NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_ASM NAMES armasm PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_LINK NAMES armlink PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_FROMELF NAMES fromelf PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)

set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
set(CMAKE_ASM_COMPILER ${ARM_ASM} CACHE STRING "")
set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m23 -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "--cpu Cortex-M23 -g --16 --apcs=interwork --pd \"__MICROLIB SETA 1\"" )
set(CMAKE_EXECUTABLE_SUFFIX ".axf")


project(nuc1263bsp)

set(BSP_DIR  ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/nuc1263/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)

# Build library 
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/nuc1263/Source)

# Build sample code
#add_subdirectory(${BSP_DIR}/MyCode/DAC/dac)
#add_subdirectory(${BSP_DIR}/MyCode/FMC/CRC32)
#add_subdirectory(${BSP_DIR}/MyCode/ACMP/Basic)
#add_subdirectory(${BSP_DIR}/MyCode/ACMP/Compare)
#add_subdirectory(${BSP_DIR}/MyCode/ACMP/Calibration)
#add_subdirectory(${BSP_DIR}/MyCode/I2C/I2C_Loopback_v2)
#add_subdirectory(${BSP_DIR}/MyCode/I2C/I2C_GCMode_v2)
#add_subdirectory(${BSP_DIR}/MyCode/I2C/I2C_LoopbackPDMA)
#add_subdirectory(${BSP_DIR}/MyCode/I2C/I2C_LoopbackSimple)
#add_subdirectory(${BSP_DIR}/MyCode/I2C/I2C_EEPROM)

#add_subdirectory(${BSP_DIR}/SampleCode/Hard_Fault_Sample)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_Wakeup)

#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_EEPROM)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_GCMode)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Loopback)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Loopback_10bit)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Master)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Master_PDMA)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_MultiBytes_Master)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_SingleByte_Master)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Slave)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Slave_PDMA)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_SMBUS)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/I2C_Wakeup_Slave)

#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_EEPROM)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_GCMode_Master)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_GCMode_Slave)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Loopback)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Loopback_10bit)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Master)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Master_PDMA)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Slave)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Slave_PDMA)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_SMBUS)
#add_subdirectory(${BSP_DIR}/SampleCode/RegBased/I2C_Wakeup_Slave)
add_subdirectory(${BSP_DIR}/SampleCode/RegBased/ACMP)
add_subdirectory(${BSP_DIR}/SampleCode/RegBased/ACMP_Wakeup)











