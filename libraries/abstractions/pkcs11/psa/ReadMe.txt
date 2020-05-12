1. To make use of this PSA based PKCS#11 shim layer, a PSA firmware implementation
should run in the secure world. As an option Trusted Firmware M(TF-M) can be used.

2. The default UID definations for certificates are:
    #define PSA_DEVICE_CERTIFICATE_UID    ( ( psa_storage_uid_t )5 )
    #define PSA_JITP_CERTIFICATE_UID      ( ( psa_storage_uid_t )6 )
    #define PSA_ROOT_CERTIFICATE_UID      ( ( psa_storage_uid_t )7 )
    You can configure them with different values manually.

3. The default key ID definations for persistent keys are:
    #define PSA_DEVICE_PRIVATE_KEY_ID     ( ( psa_key_id_t )0x01 )
    #define PSA_DEVICE_PUBLIC_KEY_ID      ( ( psa_key_id_t )0x10 )
    You can configure them with different values manually.

4. If MPU is enabled on the device's non-secure world, tasks which call the PKCS#11
APIs should have access to a region named "tasks_share".
