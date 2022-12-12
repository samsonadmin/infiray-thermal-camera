#ifndef USB_H
#define USB_H

#include <libusb-1.0/libusb.h>


class Usb
{
    public:
        Usb(int vid,int pid);

        unsigned char Get_Device_satus();
        bool Init();
        int ListDev();
        bool Open();
        bool Claim(int interface = 0);
        void Release();
        void Close();

        int Read(unsigned char *data,unsigned int size,unsigned int timeout = 1000);
        int Write(unsigned char *data,unsigned int size,unsigned int timeout = 1000);

    private:
        int m_ventorID;
        int m_ProductID;

        libusb_device **m_devList; //pointer to pointer of device, used to retrieve a list of devices
        libusb_device_handle *m_devhandle; //a device handle
        libusb_context *m_ctx; //a libusb session
        int res;

        bool m_opened;
        bool m_claimed;
        int m_interface_number;
};

#endif // USB_H
