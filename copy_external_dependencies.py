#!/usr/bin/env python3

import subprocess
import argparse
import shutil
import os
import sys

def find_lib(libname):
    env = os.environ.copy()
    env['LD_PRELOAD'] = libname
    ldd_output = subprocess.check_output(['ldd', '/bin/true'], env=env).decode()
    for line in ldd_output.split('\n'):
        tokens = line.strip().split(' ')
        if '=>' not in tokens:
            continue
        if libname == tokens[0]:
            path = tokens[2]
            return path

def set_rpath(libname, rel_rpath=None):
    
    if rel_rpath is None:
        rel_rpath = '.'

    subprocess.run(['patchelf', '--remove-rpath', libname])
    rpath = os.path.join('$ORIGIN', rel_rpath)
    subprocess.run(['patchelf', '--force-rpath', '--set-rpath', rpath, libname])


parser = argparse.ArgumentParser(description='This tool discovers the location of shared library transitive dependencies')
parser.add_argument('solib', nargs=1, help='path to the input shared library')
parser.add_argument('--dst', '-d', required=False, help='destination path where found libs are copied')
parser.add_argument('--find-solib', '-f', action='store_true', help='use dlopen/dladdr to find the path to the shared library')
parser.add_argument('--copy-solib', '-c', action='store_true', help='copy input shared library as well')
parser.add_argument('--solib-dst', required=False, help='destination path where the inputs solib is copied')
parser.add_argument('--solib-rpath', required=False, help='relative rpath for the input solib (w.r.t. $ORIGIN)')

args = parser.parse_args()

if not args.copy_solib and args.solib_rpath is not None:
    raise ValueError('solib_rpath requires copy_solib')

if not args.copy_solib and args.solib_dst is not None:
    raise ValueError('solib_dst requires copy_solib')

solib_path = find_lib(args.solib[0]) if args.find_solib else args.solib[0]
ldd_output = subprocess.check_output(['ldd', solib_path]).decode()

if args.dst:
    os.makedirs(name=args.dst, exist_ok=True)

if args.solib_dst:
    os.makedirs(name=args.dst, exist_ok=True)

if args.copy_solib:
    solib_dst = args.solib_dst
    if solib_dst is None:
        solib_dst = args.dst
    dst_file = os.path.join(solib_dst, os.path.basename(solib_path))
    shutil.copyfile(src=solib_path, dst=dst_file)
    set_rpath(dst_file, args.solib_rpath)

for line in ldd_output.split('\n'):
    tokens = line.strip().split(' ')
    if '=>' not in tokens:
        continue
    libname = tokens[0]
    path = tokens[2]
    exclude_lib = path.startswith('/usr/lib') or path.startswith('/lib')
    
    if not exclude_lib:
        print(path)
        if args.dst is not None:
            dst_file = os.path.join(args.dst, libname)
            shutil.copyfile(src=path, dst=dst_file)
            set_rpath(dst_file)