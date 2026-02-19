#!/usr/bin/env bash
set -euo pipefail
# Usage: run from /path/to/me326-team7
# Builds and installs GPD from source located at ./grasp_libraries/gpd

GPD_SRC_DIR="$(pwd)/grasp_libraries/gpd"
BUILD_DIR="$GPD_SRC_DIR/build"
INSTALL_PREFIX="/usr/local"

if [ ! -d "$GPD_SRC_DIR" ]; then
  echo "ERROR: GPD source not found at $GPD_SRC_DIR"
  exit 1
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS='-O2 -fopenmp -fPIC -Wno-deprecated -Wno-ignored-attributes -std=c++17'
make -j"$(nproc)"
sudo make install

# Ensure includes and library are in place
sudo mkdir -p "$INSTALL_PREFIX/include"
sudo mkdir -p "$INSTALL_PREFIX/lib"
if [ -f "$BUILD_DIR/libgpd.so" ]; then
  sudo cp -f "$BUILD_DIR/libgpd.so" "$INSTALL_PREFIX/lib/"
fi
if [ -d "$GPD_SRC_DIR/include/gpd" ]; then
  sudo cp -r "$GPD_SRC_DIR/include/gpd" "$INSTALL_PREFIX/include/"
fi
# Install Findgpd.cmake for CMake discovery
sudo mkdir -p "$INSTALL_PREFIX/share/cmake"
cat > /tmp/Findgpd.cmake <<'EOF'
# Find GPD (Grasp Pose Detection) Library
find_path(gpd_INCLUDE_DIR NAMES grasp_detector.h PATHS /usr/local/include /usr/include PATH_SUFFIXES gpd)
find_library(gpd_LIBRARY NAMES gpd PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu)
set(gpd_INCLUDE_DIRS "${gpd_INCLUDE_DIR}/..")
set(gpd_LIBRARIES "${gpd_LIBRARY}")
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gpd REQUIRED_VARS gpd_LIBRARY gpd_INCLUDE_DIR)
mark_as_advanced(gpd_INCLUDE_DIR gpd_LIBRARY)
EOF
sudo mv /tmp/Findgpd.cmake "$INSTALL_PREFIX/share/cmake/Findgpd.cmake"

# Refresh linker cache
sudo ldconfig

echo "GPD installed to $INSTALL_PREFIX (lib + headers + Findgpd.cmake)"

# Print verification
ls -lh "$INSTALL_PREFIX/lib/libgpd.so" || true
ls -ld "$INSTALL_PREFIX/include/gpd" || true
ls -lh "$INSTALL_PREFIX/share/cmake/Findgpd.cmake" || true
