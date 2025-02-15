#!/usr/bin/python
#coding:utf-8
from __future__ import print_function
import sys
import os
import platform
sys.path.append(os.path.split(os.path.realpath(__file__))[0]+'/..')
from build_option import *

def run_and_check_error(command):
    if os.system(command) != 0:
        print('failed to execute: ', command)
        exit(-1)

def build_one_arch(workingPath, buildtype, arch, option):
    buildPath = workingPath + '/' + arch + '/' + buildtype
    if not os.path.exists(buildPath):
        os.makedirs(buildPath)
    os.chdir(buildPath)

    OHOS_SDK = os.getenv('OHOS_SDK')
    if not OHOS_SDK:
        print('Error: The OHOS_SDK env var is not set', file=sys.stderr)
        sys.exit(-1)
    OHOS_NATIVE_HOME = OHOS_SDK + '/native'
    ohos_toolchain_file = OHOS_NATIVE_HOME + '/build/cmake/ohos.toolchain.cmake'
    cmake_exec = OHOS_NATIVE_HOME + '/build-tools/cmake/bin/cmake'

    print("********** Compiling one ARCH, target:%s, host:%s, buildtype:%s **********" % (arch, hostArch, buildtype))

    cmakeConfig = ['-DCMAKE_SYSTEM_NAME=OHOS',
                   '-DCMAKE_OHOS_ARCH_ABI=' + arch,
                   '-DCMAKE_OHOS_STL_TYPE=c++_static',
                   '-DCMAKE_TOOLCHAIN_FILE=' + ohos_toolchain_file,
                   '-DCMAKE_BUILD_TYPE=' + buildtype,
                   '-DCMAKE_CXX_STANDARD=14']
    run_and_check_error(cmake_exec + ' ../../../../.. ' + ' '.join(cmakeConfig))
    run_and_check_error('make -j8')

def build_ohos(workingPath, option):
    for arch in option['archs']:
        if option['debug']:
            build_one_arch(workingPath, 'Debug', arch, option)
        if option['release']:
            build_one_arch(workingPath, 'Release', arch, option)

def ohos_main(option):
    workingPath = os.path.split(os.path.realpath(__file__))[0] + '/out'
    if not os.path.exists(workingPath):
        os.makedirs(workingPath)
    os.chdir(workingPath)

    build_ohos(workingPath, option)

if __name__ == '__main__':
    option = get_option(sys.argv, 'ohos')
    ohos_main(option)
