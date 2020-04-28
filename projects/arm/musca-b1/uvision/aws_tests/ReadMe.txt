﻿How to get the TF-M (Trusted Firmware M) source code:
    1. TF-M - https://git.trustedfirmware.org/trusted-firmware-m.git/)
       commit: f5cd3693
       Change the macro ITS_MAX_ASSET_SIZE to (3 * 512) in file platform/ext/target/musca_b1/partition/flash_layout.h

       Note: Make sure it's in the same directory with the Amazon FreeRTOS.
             For example, if the Amazon FreeRTOS is in workspace/Amazon-FreeRTOS
             Then TF-M should be in workspace/trusted-firmware-m

    2. TF-M dependencies, please follow the TF-M software requiremnts:
       https://git.trustedfirmware.org/trusted-firmware-m.git/tree/docs/user_guides/tfm_sw_requirement.rst


Introduction of the project:
    1. The project runs the PKCS#11 library test suites.
    2. The PKCS#11 library calls TF-M at the secure side.

How to build:
    1. Build TF-M for Musca B1 platform
       a. cd trusted-firmware-m
       b. mkdir build
       c. cd build
       d. build for Library model:
          cmake -G"Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D MCUBOOT_HW_KEY=FALSE -D MCUBOOT_IMAGE_NUMBER=1 -D CRYPTO_ENGINE_BUF_SIZE=0x8000 -D TFM_NS_CLIENT_IDENTIFICATION=OFF -DTARGET_PLATFORM=MUSCA_B1 -DCOMPILER=ARMCLANG ../
          build for IPC model:
          cmake -G"Unix Makefiles" -DPROJ_CONFIG=`readlink -f ../configs/ConfigCoreIPCTfmLevel2.cmake` -D MCUBOOT_HW_KEY=FALSE -D MCUBOOT_IMAGE_NUMBER=1  -D CMAKE_BUILD_TYPE=Debug -D CRYPTO_ENGINE_BUF_SIZE=0x8000 -D TFM_NS_CLIENT_IDENTIFICATION=OFF -D TFM_PARTITION_PLATFORM=ON -DTARGET_PLATFORM=MUSCA_B1 -DCOMPILER=ARMCLANG ../
       e. cmake --build ./ -- install
       You can find detailed instructions from:
       https://git.trustedfirmware.org/trusted-firmware-m.git/tree/docs/user_guides/tfm_build_instruction.rst

    2. Build the project
       Open the Keil project and build. Note that two configurable targets are contained in the project: Library Model and IPC Model(by default)
       The default build is for TF-M IPC model.
       If you want to build for IPC model:
           a. Build TF-M in IPC model.
           b. Select target as IPC Model.
           c. Rebuild the project.
       If you want to build for Library Model:
           a. Build TF-M in Library model.
           b. Select target as Library Model.
           c. Rebuild the project.
       In both models, the output image is rtos_tfm_MUSCA_B1.hex.

       Note: Keil µVision V5.25.2.0 or above is required.

How to debug:
    The default configuration of the project supports debug by CMSIS-DAP ARMv8-M Debugger which is part of Musca-b1. Just click the debug button to start debugging.

How to run the test on Musca-B1 board:

    Download the image(rtos_tfm_MUSCA_B1.hex) to Musca-B1 board following the "Execute TF-M example and regression tests on Musca test chip boards" section in
    https://git.trustedfirmware.org/trusted-firmware-m.git/tree/docs/user_guides/tfm_user_guide.rst.

*Copyright (c) 2019-2020, Arm Limited. All rights reserved.*
