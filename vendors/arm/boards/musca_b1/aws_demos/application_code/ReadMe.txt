This demo is a work based on:
    1. Amazon FreeRTOS
       commit: 5251eeaf0
    2. TF-M (Trusted Firmware M) - https://git.trustedfirmware.org/trusted-firmware-m.git/)
       commit: b1637d53d

Introduction of the demo:
    1. Get the application version via the PSA Firmware Update service
    2. Provision the credentials for the TLS connection
    3. Provision the public key for image verification in OTA
    4. Turn on wifi and connect to the access point as a station
    5. Run the OTA demo provided by Amazon FreeRTOS

How to build:
    This project is a Keil project. We use the Keil IDE to build it.
    1. Build TF-M for Musca B1 platform
       a. cd trusted-firmware-m
       b. build for Library mode:
          cmake -S ./ -B build -DTFM_PLATFORM=musca_b1/sse_200 -DMCUBOOT_IMAGE_NUMBER=1 -DMCUBOOT_HW_KEY=OFF -D CMAKE_BUILD_TYPE=Debug -D TFM_NS_CLIENT_IDENTIFICATION=OFF -DMCUBOOT_SIGNATURE_TYPE=RSA -DITS_MAX_ASSET_SIZE=1300 -DPS_RAM_FS=ON -DMCUBOOT_SIGNATURE_KEY_LEN=2048 -DCRYPTO_HW_ACCELERATOR=OFF  -DCRYPTO_ENGINE_BUF_SIZE=0x8000
          build for IPC mode:
          cmake -S ./ -B build -DTFM_PLATFORM=musca_b1/sse_200 -DMCUBOOT_IMAGE_NUMBER=1 -DMCUBOOT_HW_KEY=OFF -D CMAKE_BUILD_TYPE=Debug -D TFM_NS_CLIENT_IDENTIFICATION=OFF -DMCUBOOT_SIGNATURE_TYPE=RSA -DITS_MAX_ASSET_SIZE=1300 -DPS_RAM_FS=ON -DMCUBOOT_SIGNATURE_KEY_LEN=2048 -DCRYPTO_HW_ACCELERATOR=OFF  -DCRYPTO_ENGINE_BUF_SIZE=0x8000 -DTFM_PSA_API=ON
       e. cmake --build ./ -- install
       You can find detailed instructions from:
       https://git.trustedfirmware.org/trusted-firmware-m.git/tree/docs/user_guides/tfm_build_instruction.rst
    4. Build the demo
       Open the Keil project of the demo and build. Note that two configurable targets are contained in the project: Library mode(by default) and IPC mode.s
       The default build is for TF-M Library mode.
       If you want to build for IPC mode:
           a. Build TFM in IPC mode.
           b. Select target as IPC mode.
           c. Rebuild the demo.
       If you want to build for Library mode:
           a. Build TFM in Library mode.
           b. Select target as Library mode.
           c. Rebuild the demo.
       In both modes, the output image is rtos_tfm_MUSCA_B1.hex.


How to run the demo on Arm Musca-B1 board:
    Download the image(rtos_tfm_MUSCA_B1.hex) to Musca-B1 board following the
    "Execute TF-M example and regression tests on Musca test chip boards" section
    in https://git.trustedfirmware.org/trusted-firmware-m.git/tree/docs/user_guides/tfm_user_guide.rst.

*Copyright (c) 2019-2021, Arm Limited. All rights reserved.*
