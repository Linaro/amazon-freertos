/*
 * Amazon FreeRTOS OTA PAL V1.0.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file aws_ota_pal.c
 * @brief Platform Abstraction layer for AWS OTA
 *
 */

/* C Runtime includes. */
#include <stdlib.h>
#include <string.h>

/* Amazon FreeRTOS include. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "aws_iot_ota_pal.h"
#include "aws_application_version.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"
#include "iot_crypto.h"
#include "aws_ota_codesigner_certificate.h"

/* For aws_tests support */
#include "aws_ota_agent_config.h"

#include "firmware_update.h"
#include "psa/crypto.h"

#define PSA_CODE_SIGNING_KEY_ID     ( ( psa_key_id_t )0x11 )

/***********************************************************************
 *
 * defines
 *
 **********************************************************************/
#define OTA_PAL_PRINT( X )            vLoggingPrintf X
static const char pcOTA_PAL_CERT_BEGIN[] = "-----BEGIN CERTIFICATE-----";
static const char pcOTA_PAL_CERT_END[] = "-----END CERTIFICATE-----";

#define MUSCA_SECONDARY_SLOT    0x01

/***********************************************************************
 *
 * Macros
 *
 **********************************************************************/

/***********************************************************************
 *
 * Structures
 *
 **********************************************************************/

/***********************************************************************
 *
 * Variables
 *
 **********************************************************************/
/**
 * @brief File Signature Key
 *
 * The OTA signature algorithm we support on this platform.
 */
const char cOTA_JSON_FileSignatureKey[ OTA_FILE_SIG_KEY_STR_MAX_LENGTH ] = "sig-sha256-rsa";

/**
 * @brief Signature Verification Context
 */
void        *pvSigVerifyContext = NULL;

/**
 * @brief Signature Certificate size
 */
uint32_t    ulSignerCertSize;

/**
 * @brief pointer to Signer Certificate
 */
u8          *pucSignerCert = NULL;

/**
 * @brief Ptr to system context
 *
 * Keep track of system context between calls from the OTA Agent
 *
 */
const OTA_FileContext_t *sys_ctx = NULL;
static tfm_image_id_t ota_image_id = 0;

/**
 * @brief Current OTA Image State
 *
 * Keep track of the state OTA Agent wants us to be in.
 */
static OTA_ImageState_t current_OTA_ImageState = eOTA_ImageState_Unknown;

/* The key handle for OTA image verification. The key should be provisioned
 * before starting an OTA process by the user.
 */
extern psa_key_handle_t ota_code_verify_key_handle;

/***********************************************************************
 *
 * Functions
 *
 **********************************************************************/

/**
 * @brief Abort an OTA transfer.
 *
 * Aborts access to an existing open file represented by the OTA file context C. This is only valid
 * for jobs that started successfully.
 *
 * @note The input OTA_FileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * This function may be called before the file is opened, so the file pointer ota_image_id may be NULL
 * when this function is called.
 *
 * @param[in] C OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in aws_iot_ota_agent.h.
 *
 * The file pointer will be set to NULL after this function returns.
 * kOTA_Err_None is returned when aborting access to the open file was successful.
 * kOTA_Err_FileAbort is returned when aborting access to the open file context was unsuccessful.
 */
OTA_Err_t prvPAL_Abort( OTA_FileContext_t * const C )
{
    if( (C == NULL) || ((C != sys_ctx ) && ( sys_ctx != NULL ) ) )
    {
        OTA_PAL_PRINT( ( "Invalid input argument!" ) );
        return kOTA_Err_FileAbort;
    }

    if( sys_ctx == NULL )
    {
        return kOTA_Err_None;
    }

    if( tfm_fwu_abort( ota_image_id ) != PSA_SUCCESS )
    {
        return kOTA_Err_FileAbort;
    }

    sys_ctx = NULL;
    ota_image_id = 0;
    current_OTA_ImageState = eOTA_ImageState_Unknown;

    return kOTA_Err_None;
}

/**
 * @brief Create a new receive file for the data chunks as they come in.
 *
 * @note Opens the file indicated in the OTA file context in the MCU file system.
 *
 * @note The previous image may be present in the designated image download partition or file, so the partition or file
 * must be completely erased or overwritten in this routine.
 *
 * @note The input OTA_FileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * The device file path is a required field in the OTA job document, so C->pucFilePath is
 * checked for NULL by the OTA agent before this function is called.
 *
 * @param[in] C OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in aws_iot_ota_agent.h.
 *
 * kOTA_Err_None is returned when file creation is successful.
 * kOTA_Err_RxFileTooLarge is returned if the file to be created exceeds the device's non-volatile memory size contraints.
 * kOTA_Err_BootInfoCreateFailed is returned if the bootloader information file creation fails.
 * kOTA_Err_RxFileCreateFailed is returned for other errors creating the file in the device's non-volatile memory.
 */
OTA_Err_t prvPAL_CreateFileForRx( OTA_FileContext_t * const C )
{
    uint32_t image_type = 0;
    uint32_t image_slot = 0;

    if( C == NULL )
    {
        OTA_PAL_PRINT( ( "Invalid input argument!" ) );
        return kOTA_Err_RxFileCreateFailed;
    }

    sys_ctx = C;

    /* pucFilePath field is get from the OTA server. */
    if( memcmp( C->pucFilePath, "secure image", strlen("secure image") ) == 0 )
    {
        image_type = (uint32_t)( FWU_IMAGE_TYPE_SECURE << FWU_IMAGE_ID_TYPE_POSITION );
    }
    else if( memcmp( C->pucFilePath, "non_secure image", strlen("non_secure image") ) == 0 )
    {
        image_type = (uint32_t)( FWU_IMAGE_TYPE_NONSECURE << FWU_IMAGE_ID_TYPE_POSITION );
    }
    else if( memcmp( C->pucFilePath, "full image", strlen("full image") ) == 0 )
    {
        image_type = (uint32_t)( FWU_IMAGE_TYPE_FULL << FWU_IMAGE_ID_TYPE_POSITION );
    }
    else
    {
        OTA_PAL_PRINT( ( "Invalid input argument: %s!", C->pucFilePath ) );
        return kOTA_Err_RxFileCreateFailed;
    }

    /* The target position of the image should always be the secondary slot. */
    image_slot = ( uint32_t )( FWU_IMAGE_ID_SLOT_1);

    ota_image_id = ( ( uint32_t )(( uint16_t )C << FWU_IMAGE_ID_RANDOM_POSITION ) ) |  image_type | image_slot;
    C->lFileHandle = ota_image_id;
    return kOTA_Err_None;
}

static OTA_Err_t prvPAL_CheckSignature( OTA_FileContext_t * const C )
{
    tfm_image_info_t info = { 0 };
    tfm_image_id_t image_id = 0;
    psa_key_handle_t uxPsaDeviceKeyHandle = 0;
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_status_t uxStatus;

    /* Get the version information of the full image in primary slot. */
    image_id = FWU_IMAGE_ID_SLOT_1 | ( FWU_IMAGE_TYPE_FULL << FWU_IMAGE_ID_TYPE_POSITION );
    uxStatus = tfm_fwu_query(image_id, &info);
    if( uxStatus != PSA_SUCCESS )
    {
        OTA_PAL_PRINT(("tfm_fwu_query failed with error: %d", uxStatus));
        return kOTA_Err_SignatureCheckFailed;
    }

    psa_key_attributes_t key_attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_algorithm_t key_algorithm = 0;
    uxStatus = psa_get_key_attributes( ota_code_verify_key_handle, &key_attributes );
    if( uxStatus != PSA_SUCCESS )
    {
        OTA_PAL_PRINT(("psa_get_key_attributes failed with error: %d", uxStatus));
        return kOTA_Err_SignatureCheckFailed;
    }

    key_algorithm = psa_get_key_algorithm( &key_attributes );
    uxStatus = psa_verify_hash( ota_code_verify_key_handle,
                                key_algorithm,
                                ( const uint8_t * )info.digest,
                                ( size_t )TFM_FWU_MAX_DIGEST_SIZE,
                                C->pxSignature->ucData,
                                C->pxSignature->usSize );
    if( uxStatus != PSA_SUCCESS )
    {
        OTA_PAL_PRINT(("psa_asymmetric_verify failed with error: %d", uxStatus));
        return kOTA_Err_SignatureCheckFailed;
    }

    return kOTA_Err_None;
}

/* @brief Authenticate and close the underlying receive file in the specified OTA context.
 *
 * @note The input OTA_FileContext_t C is checked for NULL by the OTA agent before this
 * function is called. This function is called only at the end of block ingestion.
 * prvPAL_CreateFileForRx() must succeed before this function is reached, so
 * ota_image_id(or C->pucFile) is never NULL.
 * The certificate path on the device is a required job document field in the OTA Agent,
 * so C->pucCertFilepath is never NULL.
 * The file signature key is required job document field in the OTA Agent, so C->pxSignature will
 * never be NULL.
 *
 * If the signature verification fails, file close should still be attempted.
 *
 * @param[in] C OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in aws_iot_ota_agent.h.
 *
 * kOTA_Err_None is returned on success.
 * kOTA_Err_SignatureCheckFailed is returned when cryptographic signature verification fails.
 * kOTA_Err_BadSignerCert is returned for errors in the certificate itself.
 * kOTA_Err_FileClose is returned when closing the file fails.
 */
OTA_Err_t prvPAL_CloseFile( OTA_FileContext_t * const C )
{
    /* Check the signature. */
    if( prvPAL_CheckSignature( C ) != kOTA_Err_None)
    {
        return kOTA_Err_SignatureCheckFailed;
    }

    return kOTA_Err_None;
}

/**
 * @brief Write a block of data to the specified file at the given offset.
 *
 * @note The input OTA_FileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * The file pointer/handle C->pucFile, is checked for NULL by the OTA agent before this
 * function is called.
 * pacData is checked for NULL by the OTA agent before this function is called.
 * ulBlockSize is validated for range by the OTA agent before this function is called.
 * ulBlockIndex is validated by the OTA agent before this function is called.
 *
 * @param[in] C OTA file context information.
 * @param[in] ulOffset Byte offset to write to from the beginning of the file.
 * @param[in] pacData Pointer to the byte array of data to write.
 * @param[in] ulBlockSize The number of bytes to write.
 *
 * @return The number of bytes written on a success, or a negative error code from the platform abstraction layer.
 */
int16_t prvPAL_WriteBlock( OTA_FileContext_t * const C,
                           uint32_t ulOffset,
                           uint8_t * const pcData,
                           uint32_t ulBlockSize )
{
    OTA_PAL_PRINT( ( "\n\rprvPAL_WriteBlock: %d\n\r", ulBlockSize ) );
    if( (C == NULL) || (C != sys_ctx ) || ( ota_image_id == 0 ) )
    {
        OTA_PAL_PRINT( ( "Invalid input argument!" ) );
        return -1;
    }

    /* Call the PSA Firmware Update service to write image data. */
    if( tfm_fwu_write( ( tfm_image_id_t ) ota_image_id, ( size_t ) ulOffset , ( const void * )pcData, ( size_t ) ulBlockSize ) != PSA_SUCCESS)
    {
        return -3;
    }

    return ulBlockSize;
}

/**
 * @brief Activate the newest MCU image received via OTA.
 *
 * This function shall do whatever is necessary to activate the newest MCU
 * firmware received via OTA. It is typically just a reset of the device.
 *
 * @note This function SHOULD not return. If it does, the platform doesn't support
 * an automatic reset or an error occurred.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in aws_iot_ota_agent.h.
 */
OTA_Err_t prvPAL_ActivateNewImage( void )
{
    tfm_image_id_t dependency_uuid;
    tfm_image_version_t dependency_version;
    psa_status_t status;

    if( sys_ctx == NULL )
    {
        OTA_PAL_PRINT( ( "Invalid input argument!" ) );
        return -1;
    }
    status = tfm_fwu_install( ( tfm_image_id_t ) ota_image_id, &dependency_uuid, &dependency_version );
    if( status == TFM_SUCCESS_REBOOT )
    {
        prvPAL_ResetDevice();

        /* Reset failure happened. */
        return kOTA_Err_ResetNotSupported;
    }
    else if( status == PSA_SUCCESS )
    {
        return kOTA_Err_None;
    }
    else
    {
        return kOTA_Err_ActivateFailed;
    }
}

/**
 * @brief Reset the device.
 *
 * This function shall reset the MCU and cause a reboot of the system.
 *
 * @note This function SHOULD not return. If it does, the platform doesn't support
 * an automatic reset or an error occurred.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in aws_iot_ota_agent.h.
 */

OTA_Err_t prvPAL_ResetDevice( void )
{
    tfm_fwu_request_reboot();

    return kOTA_Err_None;
}

/**
 * @brief Attempt to set the state of the OTA update image.
 *
 * Do whatever is required by the platform to Accept/Reject the OTA update image (or bundle).
 * Refer to the PAL implementation to determine what happens on your platform.
 *
 * @param[in] eState The desired state of the OTA update image.
 *
 * @return The OTA_Err_t error code combined with the MCU specific error code. See aws_iot_ota_agent.h for
 *         OTA major error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   kOTA_Err_None on success.
 *   kOTA_Err_BadImageState: if you specify an invalid OTA_ImageState_t. No sub error code.
 *   kOTA_Err_AbortFailed: failed to roll back the update image as requested by eOTA_ImageState_Aborted.
 *   kOTA_Err_RejectFailed: failed to roll back the update image as requested by eOTA_ImageState_Rejected.
 *   kOTA_Err_CommitFailed: failed to make the update image permanent as requested by eOTA_ImageState_Accepted.
 */
OTA_Err_t prvPAL_SetPlatformImageState( OTA_ImageState_t eState )
{
    OTA_Err_t   result = kOTA_Err_None;

    if( eState == eOTA_ImageState_Unknown || eState > eOTA_LastImageState )
    {
        return kOTA_Err_BadImageState;
    }

    if( sys_ctx == NULL )
    {
        /* In this case, a reboot should have happened. */
        switch ( eState )
        {
            case eOTA_ImageState_Accepted:
                /* Make this image as a pernament one. */
                if( tfm_fwu_accept() != PSA_SUCCESS )
                {
                    result = kOTA_Err_CommitFailed;
                }
                break;
            case eOTA_ImageState_Rejected:
                /* The image is not the running image, the image in the secondary slot will be ereased if
                 * it is not a valid image. */
                result = kOTA_Err_None;
                break;
            case eOTA_ImageState_Testing:
                break;
            case eOTA_ImageState_Aborted:
                /* The image download has been finished or has not been started.*/
                break;
            default:
                result = kOTA_Err_BadImageState;
                break;
        }
    }
    else
    {
        if( eState == eOTA_ImageState_Accepted )
        {
            /* The image can only be set as accepted after a reboot. So the sys_ctx should be NULL. */
            result = kOTA_Err_CommitFailed;
        }
        else
        {
            /* The image is still downloading and the OTA process will not continue. The image is in
             * the secondary slot and does not impact the later update process. So nothing to do here. */
        }
    }

    /* keep track of the state OTA Agent sent. */
    current_OTA_ImageState = eState;

    return result;
}

/**
 * @brief Get the state of the OTA update image.
 *
 * We read this at OTA_Init time and when the latest OTA job reports itself in self
 * test. If the update image is in the "pending commit" state, we start a self test
 * timer to assure that we can successfully connect to the OTA services and accept
 * the OTA update image within a reasonable amount of time (user configurable). If
 * we don't satisfy that requirement, we assume there is something wrong with the
 * firmware and automatically reset the device, causing it to roll back to the
 * previously known working code.
 *
 * If the update image state is not in "pending commit," the self test timer is
 * not started.
 *
 * @return An OTA_PAL_ImageState_t. One of the following:
 *   eOTA_PAL_ImageState_PendingCommit (the new firmware image is in the self test phase)
 *   eOTA_PAL_ImageState_Valid         (the new firmware image is already committed)
 *   eOTA_PAL_ImageState_Invalid       (the new firmware image is invalid or non-existent)
 *
 *   NOTE: eOTA_PAL_ImageState_Unknown should NEVER be returned and indicates an implementation error.
 */
OTA_PAL_ImageState_t prvPAL_GetPlatformImageState( void )
{
    OTA_PAL_ImageState_t result = eOTA_PAL_ImageState_Unknown;

    /**
     * After swap/copy of secondary slot to primary slot, boot_swap_type() returns NONE.
     * It does not reflect the fact we may be in self-test mode.
     * Use the saved value from SetImageState() to report our status.
     */
    if( current_OTA_ImageState == eOTA_ImageState_Testing )
    {
        /* in self-test, report Pending. */
        result = eOTA_PAL_ImageState_PendingCommit;
    }
    else if( (current_OTA_ImageState == eOTA_ImageState_Rejected ) ||
              (current_OTA_ImageState == eOTA_ImageState_Aborted ) )
    {
        result = eOTA_PAL_ImageState_Invalid;
    }
    else if( current_OTA_ImageState == eOTA_ImageState_Accepted )
    {
        result = eOTA_PAL_ImageState_Valid;
    }
    else
    {
        result = eOTA_PAL_ImageState_Unknown;
    }

    return result;
}
