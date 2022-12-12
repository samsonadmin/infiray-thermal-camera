#include <stdio.h>
#include "Usb.h"

Usb::Usb(int vid,int pid)
{
    m_ventorID = vid;
    m_ProductID = pid;

    m_devhandle = NULL;
    m_ctx = NULL;
    m_opened = false;
    m_claimed =false;
    m_interface_number = 0;
    m_devList=NULL;
}



bool Usb::Init()
{
    res = libusb_init(&m_ctx); //initialize the library for the session we just declared
	if(res < 0)
	{
	    return false;
	}

	return true;
}

int Usb::ListDev()
{

    ssize_t cnt,i;
    int devType=0;//0:null  1:uvc   2:bulk
    struct libusb_device_descriptor descriptor;
    cnt = libusb_get_device_list(m_ctx, &m_devList); //get the list of devices
    if(cnt < 0)
    {
        printf("NO USB DEVICE");
        return devType;
    }
    for (i = 0; i < cnt; ++i)
    {
        libusb_device *device = m_devList[i];
        if (libusb_get_device_descriptor (device, &descriptor) == 0)
        {
            printf("idVendor:%X ,idProduct:%X , iProduct:%X , iSerialNumber:%X \n",descriptor.idVendor,descriptor.idProduct,descriptor.iProduct,descriptor.iSerialNumber);
            if(descriptor.idVendor==0x1514 && descriptor.idProduct==0x0001){devType=1;}
            if(descriptor.idVendor==0x1514 && descriptor.idProduct==0xFFFF){devType=2;}
        }
    }
    libusb_free_device_list(m_devList, 1);
    m_devList=NULL;
    return devType;
}

bool Usb::Open()
{
	m_devhandle = libusb_open_device_with_vid_pid(m_ctx, m_ventorID, m_ProductID);
	if(m_devhandle == NULL)
	{
            printf("*** Permission denied or Can not find the USB board (Maybe the USB driver has not been installed correctly), quit!\n");
            return false;
	}
	else
	{
            printf("open bulk sucess!\n");
            m_opened = true;
            return true;
	}

}

bool Usb::Claim(int interface)
{
    if(m_opened)
    {
        m_interface_number = interface;
        if(libusb_kernel_driver_active(m_devhandle, interface) > 0)
        { //find out if kernel driver is attached
            printf("this device is in use,detaching");
            libusb_detach_kernel_driver(m_devhandle, interface);//detach it
        }
        res = libusb_claim_interface(m_devhandle, interface);
        if(res < 0)
        {
            return false;
        }
        m_claimed = true;
        return true;
    }
    return false;
}

void Usb::Release()
{
    if(m_opened)
        Close();

    if(m_devList)
        delete m_devList;

    if(m_ctx!=NULL) 
    {
        libusb_exit(m_ctx);
        m_ctx=NULL;
    }

}


void Usb::Close()
{
    if(m_claimed)
    {
        libusb_release_interface(m_devhandle, m_interface_number);
        m_claimed = false;
    }
    if(m_devhandle!=NULL) 
    {
        libusb_close(m_devhandle);
        m_devhandle=NULL;
    }
    m_opened = false;
}

unsigned char Usb::Get_Device_satus()
{
    if(m_claimed)
    {
        int interface = 0;
        unsigned char byte;
        libusb_control_transfer(m_devhandle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                LIBUSB_REQUEST_CLEAR_FEATURE,
                0,
                interface,
                &byte, 1, 5000);
        return byte;
    }

    return 0;

}

int Usb::Read(unsigned char *data,unsigned int size,unsigned int timeout)
{
    int readed = 0;
    if(m_claimed)
    {
        res = libusb_bulk_transfer(m_devhandle,(2 | LIBUSB_ENDPOINT_IN), data, size, &readed, timeout);
        printf("Read libusb_bulk_transfer err:%d,readed:%d",res,readed);
    }
    return res;
}

int Usb::Write(unsigned char *data,unsigned int size,unsigned int timeout)
{
    int wrote = 0;
    if(m_claimed)
    {
        res = libusb_bulk_transfer(m_devhandle,(2 | LIBUSB_ENDPOINT_OUT), data, size, &wrote, timeout);
        printf("Write libusb_bulk_transfer err:%d,wrote:%d",res,wrote);
    }
    return res;
}
