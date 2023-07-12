# setup environment

# download toolchain
rm -rf out.tar.xz
curl -L https://developer.arm.com/-/media/Files/downloads/gnu/12.2.mpacbti-rel1/binrel/arm-gnu-toolchain-12.2.mpacbti-rel1-darwin-arm64-arm-none-eabi.tar.xz --output out.tar.xz

# unzip to new folder

# make folder if it doesnt exist
mkdir -p ./arm_toolchain
rm -rf ./arm_toolchain/*
echo "\nextracting toolchain...\n"
tar -xf out.tar.xz --directory=./arm_toolchain

rm -rf out.tar.xz

echo "removing all instances of libc_nano.a in toolchain path\n"

find arm_toolchain -name "libc_nano.a" -type f -exec rm {} +
