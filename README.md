# infiray-thermal-camera
Contains code from InfiRay SDK

sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libusb-dev


g++ main.cpp Usb.cpp -L. -lthermometry -lSimple -lm -lpthread -lpot -lusb-1.0 -o main1 `pkg-config opencv4 --libs --cflags`
export LD_LIBRARY_PATH=./
./main1

export LD_LIBRARY_PATH=./

