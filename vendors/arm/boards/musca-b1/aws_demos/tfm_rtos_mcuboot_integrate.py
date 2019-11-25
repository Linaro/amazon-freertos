#-------------------------------------------------------------------------------
# Copyright (c) 2019 Arm Limited. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
#-------------------------------------------------------------------------------

import subprocess
import sys
import getopt
import os.path
from pathlib import Path
import runpy
import platform

NS_BIN = 0
TFM_DIR = 0
PLATFORM = 0
VERSION = 0
BL2_BIN_OFFSET = 0
SIGN_BIN_OFFSET = 0
FLASH_LAYOUT = 0
SIGN_BIN_SIZE = 0x0
tfm_directory = 0

def usageDescription():
    print("Usage: tfm_rtos_mcuboot_integrate.py <-n path_to_ns_bin> <-t path_to_tfm> <-p xxx>\n \
      -n, --ns_bin     The path of non-secure bin file. Absolute path and relative path to the execute path are accepted. <mandatory>\n \
      -t, --tfm_dir    The path of TFM base folder. Absolute path and relative path to the execute path are accepted. <mandatory>\n \
      -p, --platform   The target platform. 'MUSCA_B' is supported. <mandatory>\n")

def getInputArg():
    try:
        opts,args = getopt.getopt(sys.argv[1:],'-n:-t:-p:',['ns_bin=','tfm_dir=','platform='])
        if len(opts) != 3:
            usageDescription()
            sys.exit(2)
        for opt_name,opt_value in opts:
            if opt_name in ('-n','--ns_bin'):
                global NS_BIN
                NS_BIN = opt_value
                print(NS_BIN)
            if opt_name in ('-t','--tfm_dir'):
                global TFM_DIR
                TFM_DIR = opt_value
                print(TFM_DIR)
            if opt_name in ('-p','--platform'):
                global PLATFORM
                PLATFORM = opt_value
                print(PLATFORM)
    except getopt.GetoptError as err:
        print (str(err))
        usageDescription()
        sys.exit(2)

''' Check the input arguments' validity. 0, 1, 2 is returned respectivily for invalid input case,
    valid input and file found case, valid input but file no found case.'''
def checkInputArgValid():
    global FLASH_LAYOUT
    global SIGN_BIN_SIZE
    global BL2_BIN_OFFSET
    global SIGN_BIN_OFFSET
    global NS_BIN
    global TFM_DIR
    global TFM_BUILD_DIR
    valid_param = 0
    NS_BIN = os.path.abspath(NS_BIN)
    ns_bin_path = Path(NS_BIN)
    if not ns_bin_path.is_file():
        print(NS_BIN, "does not exist!")
        valid_param = 0
        return valid_param
    TFM_DIR = os.path.abspath(TFM_DIR)
    TFM_BUILD_DIR = os.path.join(TFM_DIR, 'build')
    tfm_directory = Path(TFM_DIR)
    if not tfm_directory.is_dir():
        print(tfm_directory, "does not exit!")
        valid_param = 0
        return valid_param
    if PLATFORM == "MUSCA_B1":
        flash_layout_file = os.path.join(TFM_BUILD_DIR, 'image_macros_preprocessed.c')
        SIGN_BIN_SIZE = 0xC0000
        BL2_BIN_OFFSET = 0xA000000
        SIGN_BIN_OFFSET = 0xA020000
    else:
        print(PLATFORM, "is not supported!")
        valid_param = 0
        return valid_param

    flash_layout_path = Path(flash_layout_file)
    if not flash_layout_path.is_file():
        print(flash_layout_path, "does not exist!")
        valid_param = 2
        return valid_param
    FLASH_LAYOUT = flash_layout_path
    print("BL2 binary offset is hardcode at", BL2_BIN_OFFSET)
    print("Sign binary offset is hardcode at", SIGN_BIN_OFFSET)
    valid_param = 1
    return valid_param

def getTFMFullBin():
    global TFM_DIR
    SCRIPTS_DIR=os.path.join(TFM_DIR, 'bl2', 'ext', 'mcuboot', 'scripts')
    assemble_path = os.path.join(SCRIPTS_DIR, 'assemble.py')
    assemble_file = Path(assemble_path)
    if not assemble_file.is_file():
        print("Cannot find", ASSEMBLE_PY)
        sys.exit(2)
    tfm_s_bin = os.path.join(TFM_BUILD_DIR, 'install', 'outputs', PLATFORM, "tfm_s.bin")
    cmd = 'python {path} --layout {flash_path} -s {tfm_s_bin_arg} -n {ns_bin} -o {tfm_full_bin}'
    p_cmd = cmd.format(path=assemble_file, flash_path=FLASH_LAYOUT, tfm_s_bin_arg = tfm_s_bin, ns_bin=NS_BIN, tfm_full_bin=r'tfm_full.bin')
    subprocess.call(p_cmd, shell=True)


def getSignTFMBin():
    global TFM_DIR
    IMGTOOL_PY_PATH = os.path.join(TFM_DIR, 'bl2', 'ext', 'mcuboot', 'scripts', 'imgtool.py')
    imgtool_py_file = Path(IMGTOOL_PY_PATH)
    pem = os.path.join(TFM_DIR, 'bl2', 'ext', 'mcuboot', 'root-rsa-3072.pem')
    value = 0x400
    cmd = 'python {path} sign -k {pem_path} --layout {flash_path} --align 1 -H {value_400} {tfm_full_bin} {tfm_sign_bin}'
    print("SIGN_BIN_SIZE =", SIGN_BIN_SIZE)
    p_cmd = cmd.format(path=imgtool_py_file, pem_path=pem ,flash_path=FLASH_LAYOUT, value_400=value, tfm_full_bin=r'tfm_full.bin', tfm_sign_bin= r'tfm_sign.bin')
    subprocess.call(p_cmd, shell=True)

def getHex():
    global BL2_BIN_OFFSET
    global SIGN_BIN_OFFSET
    if platform.system() == "Windows":
        srec_cat_bin = 'srec_cat.exe'
    else:
        srec_cat_bin = 'srec_cat'
    tfm_hex = "rtos_tfm_" + PLATFORM + ".hex"
    mcuboot_bin = os.path.join(TFM_BUILD_DIR, 'bl2', 'ext', 'mcuboot', 'mcuboot.bin')
    tfm_sign_bin = r'tfm_sign.bin'
    cmd = srec_cat_bin + ' {mcuboot_bin_placeholer} -Binary -offset {bl2_bin_offset_placeholer} {tfm_sign_bin_placeholer}  -Binary -offset {sign_bin_offset_placeholer} -o {tfm_hex_placeholder} -Intel'
    p_cmd = cmd.format(mcuboot_bin_placeholer=mcuboot_bin, bl2_bin_offset_placeholer=BL2_BIN_OFFSET, tfm_sign_bin_placeholer=tfm_sign_bin, sign_bin_offset_placeholer=SIGN_BIN_OFFSET, tfm_hex_placeholder=tfm_hex)
    print(p_cmd)
    subprocess.call(p_cmd, shell=True)

if __name__== "__main__":
    getInputArg()
    valid_param = checkInputArgValid()
    if valid_param == 0:
        print("Invalid parameter!")
        usageDescription()
        sys.exit(-1)
    elif valid_param == 2:
        sys.exit(-1)
    getTFMFullBin()
    getSignTFMBin()
    getHex()
