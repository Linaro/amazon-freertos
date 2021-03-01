# What is this project

The Amazon FreeRTOS OTA PAL to PSA shim layer provides a reference implementation of FreeRTOS OTA PAL based on PSA API.

PSA is Platform Security Architecture which is initiated by Arm. Please get the details from this [link](https://www.arm.com/why-arm/architecture/platform-security-architecture).

In general, this shim layer maps the Amazon FreeRTOS OTA PAL APIs to PSA Firmware Update and Cyrpto APIs. It follows the PSA Firmware Update API 0.7 and PSA Cryptography API V1.0 beta3. The process of the image write, image verification and image activation are protected by the PSA secure service.

# Integration guide

## Integrate PSA Based OTA PAL with the FreeRTOS project

- Use the `libraries/abstractions/ota_pal_psa/aws_ota_pal.c` as the implementation of APIs defined in `libraries/freertos_plus/aws/ota/src/aws_iot_ota_pal.h`
- Add the source file `libraries/abstractions/ota_pal_psa/version/application_version.c` to the project.
- Get the key handle named `ota_code_verify_key_handle` which is dedicated for OTA image verification by either provisionning it or opening a key which is provisioned by others.
- Build the PSA implementation as the secure side image (check the Trusted Firmware-M example in the following section).
- Integrate the FreeRTOS project with the interface files of the PSA implementation (check the TF-M example below).
- Build the FreeRTOS project.
- Follow the platform specific instructions to sign/combine the FreeRTOS image and secure side image.

## Integrate FreeRTOS project with Trusted Firmware-M (TF-M)

Please refer to [this](https://github.com/Linaro/freertos-pkcs11-psa/blob/master/ReadMe.md#integrate-freertos-project-with-trusted-firmware-m-tf-m) link.
