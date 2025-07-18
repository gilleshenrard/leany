#! /bin/sh

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

# Download the ARM toolchain
echo " "
echo "Retrieve the latest version of the ARM toolchain"
ARM_TOOLCHAIN_VERSION=$(curl -s https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads | grep -Po '<h4>Version \K.+(?=</h4>)')
echo "Latest ARM toolchain version: $ARM_TOOLCHAIN_VERSION"
curl -Lo gcc-arm-none-eabi.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz"

# Extract the ARM toolchain
echo " "
echo "Extract the ARM toolchain"
sudo mkdir -v /opt/arm-none-eabi
sudo tar xvf gcc-arm-none-eabi.tar.xz --strip-components=1 -C /opt/arm-none-eabi
rm gcc-arm-none-eabi.tar.xz

# Add the ARM toolchain to the PATH
echo " "
echo "Add the ARM toolchain to the PATH"
echo 'export PATH=$PATH:/opt/arm-none-eabi/bin' | sudo tee -a /etc/profile.d/arm-none-eabi.sh
source /etc/profile


# Installation end
echo " "
echo "Installation of the ARM toolchain is complete"
echo "Please restart your terminal to apply the changes"

# Troubleshooting
echo " "
echo "If you encounter any issues,"
echo "re-run 'source /etc/profile' to update the PATH"
echo "then restart your terminal"
