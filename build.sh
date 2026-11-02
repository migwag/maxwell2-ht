#!/bin/sh
# Builds the universal, ad-hoc signed Maxwell 2 head-tracker plugin into build/.
set -e
cd "$(dirname "$0")"
mkdir -p build
clang -arch arm64 -arch x86_64 -mmacosx-version-min=11.0 -O2 -Wall -Wextra -Wno-unused-parameter \
  -fvisibility=hidden -dynamiclib -install_name @rpath/libMaxwell2HT.dylib \
  -framework IOKit -framework CoreFoundation -o build/libMaxwell2HT.dylib src/maxwell2_headtracker.c
codesign --force --sign - build/libMaxwell2HT.dylib
echo "built build/libMaxwell2HT.dylib"
