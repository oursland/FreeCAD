#!/bin/bash

set -e
set -x 

conda_env_root="FreeCAD.app/Contents"
conda_env="FreeCAD.app/Contents/Resources"

mkdir -p ${conda_env}

cp -a ../.pixi/envs/default/* ${conda_env}

export PATH="${PWD}/FreeCAD.app/Contents/Resources/bin:${PATH}"
export CONDA_PREFIX="${PWD}/FreeCAD.app/Contents/Resources"

# delete unnecessary stuff
rm -rf ${conda_env}/include
find ${conda_env} -name \*.a -delete

mv ${conda_env}/bin ${conda_env}/bin_tmp
mkdir ${conda_env}/bin
cp ${conda_env}/bin_tmp/freecad ${conda_env}/bin/
cp ${conda_env}/bin_tmp/freecadcmd ${conda_env}/bin
cp ${conda_env}/bin_tmp/ccx ${conda_env}/bin/
cp ${conda_env}/bin_tmp/python ${conda_env}/bin/
cp ${conda_env}/bin_tmp/pip ${conda_env}/bin/
cp ${conda_env}/bin_tmp/pyside6-rcc ${conda_env}/bin/
cp ${conda_env}/bin_tmp/gmsh ${conda_env}/bin/
cp ${conda_env}/bin_tmp/dot ${conda_env}/bin/
cp ${conda_env}/bin_tmp/unflatten ${conda_env}/bin/
rm -rf ${conda_env}/bin_tmp

sed -i '1s|.*|#!/usr/bin/env python|' ${conda_env}/bin/pip

# copy resources
cp resources/* ${conda_env}

# Remove __pycache__ folders and .pyc files
find . -path "*/__pycache__/*" -delete
find . -name "*.pyc" -type f -delete

# fix problematic rpaths and reexport_dylibs for signing
# see https://github.com/FreeCAD/FreeCAD/issues/10144#issuecomment-1836686775
# and https://github.com/FreeCAD/FreeCAD-Bundle/pull/203
python ../scripts/fix_macos_lib_paths.py ${conda_env}/lib

# build and install the launcher
cmake -B build launcher
cmake --build build
mkdir -p FreeCAD.app/Contents/MacOS
cp build/FreeCAD FreeCAD.app/Contents/MacOS/FreeCAD

FreeCADCmd ../scripts/get_freecad_version.py
version_name=$(<bundle_name.txt)

echo -e "\################"
echo -e "version_name:  ${version_name}"
echo -e "################"

pixi list > FreeCAD.app/Contents/packages.txt
sed -i '1s/.*/\nLIST OF PACKAGES:/' FreeCAD.app/Contents/packages.txt

# copy the plugin into its final location
cp -a ${conda_env}/Library ${conda_env}/..
rm -rf ${conda_env}/Library

# create the dmg
dmgbuild -s dmg_settings.py "FreeCAD" "${version_name}.dmg"

# create hash
shasum -a 256 ${version_name}.dmg > ${version_name}.dmg-SHA256.txt
