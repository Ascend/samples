#!/bin/sh
#
# Create a distributable archive of the current version of Makeself

VER=`cat VERSION`
mkdir ./makeself-$VER
cp -a makeself* test README.md COPYING VERSION .gitmodules ./makeself-$VER/
makeself.sh --notemp ./makeself-$VER makeself-$VER.run "Makeself v$VER" echo "Makeself has extracted itself"

