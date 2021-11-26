#!/usr/bin/python
#coding:utf-8
from __future__ import print_function
import sys
import os
import getopt
import platform

def mkdir_p(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)

def run_and_check_error(command):
    if os.system(command) != 0:
        print('failed to execute: ', command)
        exit(-1)

def build_one_arch(workingPath, buildtype, arch, memcheck):
    buildPath = workingPath + '/' + arch + '/' + buildtype
    if not os.path.exists(buildPath):
        os.makedirs(buildPath)
    os.chdir(buildPath)
    hostArch = platform.machine()
    if hostArch == 'aarch64':
        hostArch = 'arm64'

    print("********** Compiling one ARCH, target:%s,host:%s buildtype:%s **********" % (arch,hostArch,buildtype))

    if hostArch != arch and hostArch != 'x86_64': # we should only cross platform compile on x86_64 linux
        print('wrong arch,target:%s,host arch:%s' % (arch,hostArch))
        return

    toolchains_path = workingPath + '/../../cmake/toolchains'
    if arch == 'arm' and hostArch == 'x86_64':
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
                   '-DCMAKE_SYSTEM_NAME=Linux',
                   '-DCMAKE_TOOLCHAIN_FILE='+toolchains_path+'/arm-linux.cmake']
    elif arch == 'arm' and hostArch == arch:
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
            '-DCMAKE_SYSTEM_NAME=Linux']
    elif arch == 'arm64' and hostArch == 'x86_64':
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
                   '-DCMAKE_SYSTEM_NAME=Linux',
                   '-DCMAKE_TOOLCHAIN_FILE='+toolchains_path+'/arm64-linux.cmake']
    elif arch == 'arm64' and hostArch == arch:
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
                   '-DCMAKE_SYSTEM_NAME=Linux']
    elif arch == 'x86' and hostArch == arch:
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
                   '-DCMAKE_SYSTEM_NAME=Linux',
                   '-DCMAKE_C_FLAGS=-m32',
                   '-DCMAKE_CXX_FLAGS=-m32']
    elif arch == 'x86' and hostArch != arch:
        print('wrong arch,target:%s,host arch:%s' % (arch,hostArch))
        return
    else:
        cmakeConfig = ['-DCMAKE_BUILD_TYPE='+buildtype,
                   '-DCMAKE_SYSTEM_NAME=Linux',
                   '-DCMAKE_C_FLAGS=-m64',
                   '-DCMAKE_CXX_FLAGS=-m64']
    if memcheck:
        cmakeConfig.append('-DENABLE_ASAN=1')
    run_and_check_error('cmake ../../../../.. ' + ' '.join(cmakeConfig))
    run_and_check_error('make')

def build_linux(workingPath,archs,memcheck):
    for arch in archs:
        print("build_linux %s " % (arch))
        if arch == 'x86_64':
            build_one_arch(workingPath, 'Debug', arch, memcheck)
        build_one_arch(workingPath, 'Release', arch, memcheck)

def checkoptions(argv):
    options={
        "archs" : ['arm','arm64','x86_64'],
        "memcheck": False
    }
    
    try:
        opts, args = getopt.getopt(argv,"a:m",["arch=","memcheck"])
    except getopt.GetoptError as e:
        return options

    for opt, arg in opts:
        if opt in ("-a", "--arch"):
            options["archs"] = arg.split(",")
        elif opt in ("-m", "--memcheck"):
            options["memcheck"] = True
    
    return options

def linux_main(argv):
    options = checkoptions(argv[1:])
    print("archs:",options["archs"])
    workingPath = os.path.split(os.path.realpath(__file__))[0] + '/out'
    if not os.path.exists(workingPath):
        os.makedirs(workingPath)
    os.chdir(workingPath)

    build_linux(workingPath,options["archs"],options["memcheck"])

if __name__ == '__main__':
    linux_main(sys.argv)
