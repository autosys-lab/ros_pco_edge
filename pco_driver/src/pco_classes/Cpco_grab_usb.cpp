//-----------------------------------------------------------------//
// Name        | CPco_grab_usb.cpp           | Type: (*) source    //
//-------------------------------------------|       ( ) header    //
// Project     | pco.camera                  |       ( ) others    //
//-----------------------------------------------------------------//
// Platform    | Linux                                             //
//-----------------------------------------------------------------//
// Environment | gcc                                               //
//             |                                                   //
//-----------------------------------------------------------------//
// Purpose     | pco.camera - libusb Image Grab                    //
//-----------------------------------------------------------------//
// Author      | MBL, PCO AG                                       //
//-----------------------------------------------------------------//
// Revision    | rev. 1.01 rel. 0.00                               //
//-----------------------------------------------------------------//
// Notes       | In this file are all functions and definitions,   //
//             | for grabbing from usb camera                      //
//             |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// (c) 2014 PCO AG * Donaupark 11 *                                //
// D-93309      Kelheim / Germany * Phone: +49 (0)9441 / 2005-0 *  //
// Fax: +49 (0)9441 / 2005-20 * Email: info@pco.de                 //
//-----------------------------------------------------------------//

#include "../../include/pco_driver/pco_classes/Cpco_grab_usb.h"

CPco_grab_usb::CPco_grab_usb(CPco_com_usb *camera)
{
  clog = NULL;
  hgrabber = (PCO_HANDLE) NULL;

  cam = NULL;

  libusb_ctx = NULL;
  libusb_hdev = NULL;

  ImageInAddr = 0;
  ImageInMaxPacketSize = 0;

  last_buffer_1 = last_buffer_2 = NULL;
  async_transfer_1 = async_transfer_2 = NULL;
  async_transfer_status = 0;
  async_transfer_num = 0;
  async_transfer_size = ASYNC_PACKET_SIZE;
  async_copy_buffer_size = 0;

  act_bitpix = 0;
  act_width = 0;
  act_height = 0;
  act_align = 0;
  act_line_width = 0;
  act_dmalength = 0;

  DataFormat = 0;
  ImageTimeout=0;
  packet_timeout=0;
  aquire_flag=0;

 //reset these settings
  camtype=0;
  serialnumber=0;
  cam_pixelrate=0;
  cam_timestampmode=0;
  cam_doublemode=0;
  cam_align=0;
  cam_noisefilter=0;
  cam_colorsensor=0;
  cam_width=cam_height=1000;
  memset(&usbpar,0,sizeof(usbpar));
  memset(&description,0,sizeof(description));
  gl_Coding=0;

  if(camera != NULL)
   cam = camera;

}


CPco_grab_usb::~CPco_grab_usb()
{
  Close_Grabber();
}


void CPco_grab_usb::SetLog(CPco_Log *elog)
{
   clog=elog;
}

void CPco_grab_usb::writelog(DWORD lev,PCO_HANDLE hdriver,const char *str,...)
{
  if(clog)
  {
   va_list arg;
   va_start(arg,str);
   clog->writelog(lev,hdriver,str,arg);
   va_end(arg);
  }
}

DWORD CPco_grab_usb::Get_actual_size(unsigned int *width,unsigned int *height,unsigned int *bitpix)
{
  if((hgrabber==(PCO_HANDLE)NULL)||(cam==NULL))
  {
   writelog(ERROR_M,hgrabber,"%s: grabber class not initialized",__FUNCTION__);
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  if(width)
   *width=act_width;

  if(height)
   *height=act_height;

  if(bitpix)
   *bitpix=act_bitpix;

  return PCO_NOERROR;
}

DWORD CPco_grab_usb::Set_Grabber_Size(int width, int height)
{
  return Set_Grabber_Size(width,height,0);
}

DWORD CPco_grab_usb::Set_Grabber_Size(int width,int height, int bitpix)
{
  int dmalength;

  if((hgrabber==(PCO_HANDLE)NULL)||(cam==NULL))
  {
   writelog(ERROR_M,hgrabber,"%s: grabber class not initialized",__FUNCTION__);
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: start w:%d h:%d",width,height);

  if((bitpix == 0)&&(act_bitpix==0))
  {
   SC2_Camera_Description_Response description;
   cam->PCO_GetCameraDescriptor(&description);
   act_bitpix = description.wDynResDESC;
  }

  dmalength=width*height*((act_bitpix + 7) / 8);

//for async
  if(act_dmalength!=dmalength)
  {
   if((async_transfer_status&(ASYNC_SETUP_1|ASYNC_PENDING_1|ASYNC_SETUP_2|ASYNC_PENDING_2))!=0)
   {
    writelog(ERROR_M,hgrabber,"Set_Grabber_Size: async_transfer_status SETUP or PENDING cannot setup for new size");
    return PCO_ERROR_DRIVER_IOFAILURE | PCO_ERROR_DRIVER_USB;
   }
   else
   {
    //reset need last & copy buffer
    async_transfer_status&=~ASYNC_NEED_LAST;
    async_transfer_status&=~ASYNC_USE_COPY_BUFFER;

    //free allocated transfers
    if((async_transfer_status&ASYNC_ALLOC_1) == ASYNC_ALLOC_1)
    {
     usb_clear_input();
     for (int i = 0; i < async_transfer_num; i++)
     {
      writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: free transfer %d",i);
      libusb_free_transfer(async_transfer_1[i]);
     }
     free(async_transfer_1);
    }

    if(async_transfer_size == 0)
     async_transfer_size=ASYNC_PACKET_SIZE;

    if(async_transfer_size>LARGE_PACKET_SIZE)
     async_transfer_size = LARGE_PACKET_SIZE;

    //async_transfer_size must be a multiple of ImageInMaxPacketSize, so we round down
    if(async_transfer_size%ImageInMaxPacketSize)
     async_transfer_size=(async_transfer_size/ImageInMaxPacketSize)*ImageInMaxPacketSize;

    writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: async packet size %d",async_transfer_size);
    async_transfer_num=dmalength/async_transfer_size;

    if(dmalength%async_transfer_size)
    {
     async_last_size=dmalength-async_transfer_size*async_transfer_num;
     async_transfer_num++;
     async_transfer_status|=ASYNC_NEED_LAST;
     writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: async need last size %d",async_last_size);

    //same check as above for last size, the copy buffer removes excess bytes at the end
     if(async_last_size%ImageInMaxPacketSize)
     {
      writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: use copy buffer");
      //effective copy buffer size
      async_copy_buffer_size = async_last_size-((async_last_size/ImageInMaxPacketSize)*ImageInMaxPacketSize);
      async_last_size=(async_last_size/ImageInMaxPacketSize)*ImageInMaxPacketSize;
      async_transfer_status|=ASYNC_USE_COPY_BUFFER;
      async_transfer_num++;
     }
     writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: async transfer num %d",async_transfer_num);

    }
    //allocate transfer
    async_transfer_1=(libusb_transfer**)malloc(async_transfer_num*sizeof(libusb_transfer*));
    if(async_transfer_1)
    {
     memset(async_transfer_1,0,async_transfer_num*sizeof(libusb_transfer*));
     for(int i=0;i< async_transfer_num;i++)
      async_transfer_1[i]=libusb_alloc_transfer(0);
     async_transfer_status|=ASYNC_ALLOC_1;
    }
   }
  }

  act_height=height;
  act_width=width;
  act_line_width=width;
  act_dmalength=dmalength;

  writelog(PROCESS_M,hgrabber,"Set_Grabber_Size: done w:%d h:%d lw:%d bitpix: %d dmalength:%d"
                              ,act_width,act_height,act_line_width,act_bitpix,act_dmalength);
  return PCO_NOERROR;
}

int CPco_grab_usb::Get_Async_Packet_Size()
{
  return async_transfer_size;
}

void CPco_grab_usb::Set_Async_Packet_Size(int size)
{
  if(ImageInMaxPacketSize == 0)
  {
   writelog(ERROR_M,hgrabber,"Could not set async_transfer_size, open grabber first!");
  }
  else
  {
   //Async_transfer_size must be a multiple of ImageInMaxPacketSize, so we round down
   async_transfer_size = (size/ImageInMaxPacketSize)*ImageInMaxPacketSize;
   writelog(PROCESS_M,hgrabber,"Async packet size set to: %d",async_transfer_size);
   act_dmalength=0;
   Set_Grabber_Size(act_width,act_height);
  }
}


DWORD CPco_grab_usb::Open_Grabber(int board)
{
  return Open_Grabber(board,0);
}

DWORD CPco_grab_usb::Open_Grabber(int board,int initmode)
{
  DWORD err=PCO_NOERROR;

  if(hgrabber!=(PCO_HANDLE) NULL)
  {
   writelog(INIT_M,(PCO_HANDLE)1,"Open_Grabber: grabber was opened before");
   return PCO_NOERROR;
  }

  if(cam==NULL)
  {
   writelog(INIT_M,(PCO_HANDLE)1,"Open_Grabber: camera command interface must be available");
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_CAMERALINK;
  }

  if(!cam->IsOpen())
  {
   writelog(INIT_M,(PCO_HANDLE)1,"Open_Grabber: camera command interface must be opened and initialized");
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  libusb_hdev=cam->get_device_handle();
  if(libusb_hdev==NULL)
  {
   writelog(INIT_M,(PCO_HANDLE)1,"Open_Grabber: cam->get_devce_handle failed");
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  err=usb_get_endpoints();
  if(err!=PCO_NOERROR)
  {
   writelog(INIT_M,(PCO_HANDLE)1,"Open_Grabber: usb_get_endpoints failed");
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  if(clog)
  {
   clog->start_time_mess();
  }

  gl_Coding = cam->getCoding();
  writelog(PROCESS_M,(PCO_HANDLE)1,"Open_Grabber: GetCoding returned %d",gl_Coding);
  if((gl_Coding) &&(async_transfer_size>ASYNC_PACKET_SIZE_D))
  {
   async_transfer_size=ASYNC_PACKET_SIZE_D;
   writelog(PROCESS_M,(PCO_HANDLE)1,"Open_Grabber: Coding enabled transfer_size set to %dkB",async_transfer_size/1024);
  }

  hgrabber=(PCO_HANDLE)0x200+board;

  ImageTimeout=10000;
  packet_timeout=200;

  DataFormat=PCO_CL_DATAFORMAT_1x16;

  return err;
}

DWORD CPco_grab_usb::Close_Grabber()
{
  writelog(INIT_M,hgrabber,"Close_Grabber: ");

  if(hgrabber==(PCO_HANDLE)NULL)
  {
   writelog(INIT_M,hgrabber,"Close_Grabber: driver was closed before");
   return PCO_NOERROR;
  }
  hgrabber=(PCO_HANDLE)NULL;

  if(last_buffer_1)
  {
   free(last_buffer_1);
   last_buffer_1=NULL;
  }
  if(last_buffer_2)
  {
   free(last_buffer_2);
   last_buffer_2=0;
  }
  if((async_transfer_status&ASYNC_ALLOC_1)==ASYNC_ALLOC_1)
  {
   for(int i=0;i< async_transfer_num;i++)
    libusb_free_transfer(async_transfer_1[i]);
   free(async_transfer_1);
   async_transfer_1=NULL;
   writelog(INIT_M,hgrabber,"Close_Grabber: free async_transfer_1 done");
  }
  if((async_transfer_status&ASYNC_ALLOC_2)==ASYNC_ALLOC_2)
  {
   for(int i=0;i< async_transfer_num;i++)
    libusb_free_transfer(*async_transfer_2);
   free(async_transfer_2);
   writelog(INIT_M,hgrabber,"Close_Grabber: free async_transfer_2 done");
  }

  libusb_reset_device(libusb_hdev);

  if(libusb_ctx)
  {
   libusb_close(libusb_hdev);
   libusb_hdev = NULL;
   libusb_exit(libusb_ctx);
   libusb_ctx=NULL;
  }

  ImageInAddr = 0;
  ImageInMaxPacketSize = 0;

  last_buffer_1 = last_buffer_2 = NULL;
  async_transfer_1 = async_transfer_2 = NULL;
  async_transfer_status = 0;
  async_transfer_num = 0;
  async_last_size = 0;

  act_width = 0;
  act_height = 0;
  act_line_width = 0;
  act_dmalength = 0;
  act_bitpix = 0;
  DataFormat = 0;

  return PCO_NOERROR;
}

bool CPco_grab_usb::IsOpen()
{
  if(hgrabber!=(PCO_HANDLE)NULL)
   return true;
  else
   return false;
}

void CPco_grab_usb::SetBitAlignment(int align)
{
  act_align = align;
}

DWORD CPco_grab_usb::Set_DataFormat(DWORD dataformat)
{
  DataFormat=dataformat;
  return PCO_NOERROR;
}

DWORD CPco_grab_usb::Set_Grabber_Timeout(int timeout)
{
  ImageTimeout=timeout;
  return PCO_NOERROR;
}

DWORD CPco_grab_usb::Get_Grabber_Timeout(int *timeout)
{
  *timeout=ImageTimeout;
  return PCO_NOERROR;
}


DWORD CPco_grab_usb::Get_Camera_Settings()
{
  DWORD err=PCO_NOERROR;

  if(cam&&cam->IsOpen())
  {
   cam->PCO_GetTransferParameter(&usbpar,sizeof(usbpar));
   writelog(INFO_M,hgrabber,"Get_Camera_Settings: usbpar.MaxNumUsb       %d",usbpar.MaxNumUsb);
   writelog(INFO_M,hgrabber,"Get_Camera_Settings: usbpar.Clockfrequency  %d",usbpar.ClockFrequency);
   writelog(INFO_M,hgrabber,"Get_Camera_Settings: usbpar.Transmit        0x%x",usbpar.Transmit);
   writelog(INFO_M,hgrabber,"Get_Camera_Settings: usbpar.UsbConfig       0x%x",usbpar.UsbConfig);
   writelog(INFO_M,hgrabber,"Get_Camera_Settings: usbpar.ImgTransMode    0x%x",usbpar.ImgTransMode);

   if(description.wSensorTypeDESC==0)
   {
    cam->PCO_GetCameraDescriptor(&description);
    cam_colorsensor=description.wColorPatternTyp ? 1 : 0;
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_colorsensor       %d",cam_colorsensor);
   }

   err=cam->PCO_GetTimestampMode(&cam_timestampmode);
   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_timestampmode     %d",cam_timestampmode);
    err=cam->PCO_GetPixelRate(&cam_pixelrate);
   }
   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_pixelrate         %d",cam_pixelrate);
    err=cam->PCO_GetDoubleImageMode(&cam_doublemode);
    if((err&0xC000FFFF)==PCO_ERROR_FIRMWARE_NOT_SUPPORTED)
     err=PCO_NOERROR;
   }

   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_doublemode        %d",cam_doublemode);
    err=cam->PCO_GetNoiseFilterMode(&cam_noisefilter);
    if((err&0xC000FFFF)==PCO_ERROR_FIRMWARE_NOT_SUPPORTED)
     err=PCO_NOERROR;
   }

   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_noisefilter       %d",cam_noisefilter);
    err=cam->PCO_GetBitAlignment(&cam_align);
   }

   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_align             %d",cam_align);
    act_align=cam_align;
    err=cam->PCO_GetActualSize(&cam_width,&cam_height);
   }
   if(err==PCO_NOERROR)
   {
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_width             %d",cam_width);
    writelog(INFO_M,hgrabber,"Get_Camera_Settings: cam_height            %d",cam_height);
   }
  }
  return err;
}


DWORD CPco_grab_usb::PostArm(int userset)
{
  DWORD err=PCO_NOERROR;
  if((hgrabber==(PCO_HANDLE)NULL)||(cam==NULL))
  {
   writelog(ERROR_M,hgrabber,"%s: grabber class not initialized",__FUNCTION__);
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }


  writelog(PROCESS_M,hgrabber,"%s: userset %d",__FUNCTION__,userset);

  if(err==PCO_NOERROR)
   err=Get_Camera_Settings();

  if((err==PCO_NOERROR)&&(userset==0))
  {
   writelog(PROCESS_M,hgrabber,"PostArm: call Set_Grabber_Size(%d,%d)",cam_width,cam_height);
   err=Set_Grabber_Size(cam_width,cam_height);
  }
  return err;
}


DWORD  CPco_grab_usb::Allocate_Framebuffer(int nr_of_buffer ATTRIBUTE_UNUSED)
{
  DWORD err=PCO_NOERROR;
  return err;
}


DWORD  CPco_grab_usb::Free_Framebuffer()
{
  DWORD err=PCO_NOERROR;
  return err;
}


///////////////////////////////////////////////////////////////////////
//
//  Use these functions if you want simple image transfers
//
// Acquire_Image: Get a single image, timeout setting optional
//
// Acquire_Image_Async: Get single/multiple images asynchronous.
//                      If a preview is needed, use Acquire_Image_Async_wait (this ensures that decoding of the image is complete)
//                      If a preview is not needed, use Acquire_Image_Async for the first X-1 images and
//                      Acquire_Image_Async_wait for the last image.
//                      See usb_grabthreadworker.cpp if you need an example
//
//////////////////////////////////////////////////////////////////////

DWORD CPco_grab_usb::Acquire_Image(void *adr)
{
  DWORD err=PCO_NOERROR;
  err=Acquire_Image(adr,ImageTimeout);
  return err;
}

DWORD CPco_grab_usb::Acquire_Image(void *adr,int timeout)
{
  DWORD err=PCO_NOERROR;
  err=usb_read_image(adr,act_dmalength,timeout,0,0);
  return err;
}

DWORD CPco_grab_usb::Get_Image(WORD Segment,DWORD ImageNr,void *adr)
{
  DWORD err=PCO_NOERROR;
  err=usb_read_image(adr,act_dmalength,ImageTimeout,Segment,ImageNr);
  return err;
}


DWORD CPco_grab_usb::Acquire_Image_Async(void *adr)
{
  DWORD err = PCO_NOERROR;
  err = Acquire_Image_Async(adr, ImageTimeout, false);
  return err;
}


DWORD CPco_grab_usb::Acquire_Image_Async(void *adr, int timeout)
{
  DWORD err = PCO_NOERROR;
  err = Acquire_Image_Async(adr, timeout, false);
  return err;
}

DWORD CPco_grab_usb::Acquire_Image_Async_wait(void *adr)
{
  DWORD err = PCO_NOERROR;
  err = Acquire_Image_Async(adr, ImageTimeout, true);
  return err;
}

DWORD CPco_grab_usb::Acquire_Image_Async_wait(void *adr, int timeout)
{
  DWORD err = PCO_NOERROR;
  err = Acquire_Image_Async(adr, timeout, true);
  return err;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main Async acquire function (you can use the overloaded functions above to help you fill in the parameters)
// adr: adress of the buffer to write the current image to
// timeout: the timeout of the command
// cancel: pointer for the cancel option. If you want to stop grabbing, set this to TRUE.
// If you just want one picture, a NULL pointer is just fine (or use the Aquire image without async)
// WARNING: If you don't use CANCEL on the last image, the images might not be valid right away!
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

DWORD CPco_grab_usb::Acquire_Image_Async(void *adr, int timeout,BOOL waitforimage)
{
  DWORD err = PCO_NOERROR;
  err = usb_async_image(adr, act_dmalength, timeout,waitforimage);
  return err;

}


/////////////////////////////////////////////////////////////////
//
// Looks for USB devices and gets their endpoints for the grabber
//
////////////////////////////////////////////////////////////////

DWORD CPco_grab_usb::usb_get_endpoints()
{
  DWORD err;
  int eptCount;
  int ep_contr_in;

  int alt_set=cam->get_altsetting();

  struct libusb_device_descriptor desc;
  libusb_config_descriptor *libusb_ConfigDescr;
  libusb_device *dev;

  dev=libusb_get_device(libusb_hdev);
  err=libusb_get_device_descriptor(dev,&desc);
  if(err!=0)
  {
   writelog(ERROR_M,(PCO_HANDLE)1,"usb_get_endpoints: Get device descriptor failed (%d) %s",err,libusb_error_name(err));
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  err=libusb_get_config_descriptor(dev, 0, &libusb_ConfigDescr);
  if(err!=0)
  {
   writelog(ERROR_M,(PCO_HANDLE)1,"usb_get_endpoints: Get config descriptor failed (%d) %s",err,libusb_error_name(err));
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  eptCount = libusb_ConfigDescr->interface[0].altsetting[alt_set].bNumEndpoints;
  ep_contr_in=0;

  err=PCO_NOERROR;

  switch(desc.idProduct)
  {
   case USB_PID_IF_GIGEUSB_20:       // 0x0001  FX2 (Cypress 68013a)
   case USB_PID_CAM_PIXFLY_20:       // 0x0002  FX2 (Cypress 68013a)
    ep_contr_in = USB_EP_FX2_IMG_IN;
    break;

   case USB_PID_IF_GIGEUSB_30:       // 0x0003  FX3 (Cypress CYUSB3014-BZX) Application Code
   case USB_PID_IF_GIGEUSB_30_B1:    // 0x0004  FX3 (Cypress CYUSB3014-BZX) SPI Boot Code (FPGA Update)
   case USB_PID_IF_GIGEUSB_30_B2:    // 0x0005  FX3 (Cypress CYUSB3014-BZX) I2C Boot Code (FX3 Update)
   case USB_PID_CAM_EDGEUSB_30:      // 0x0006  Fx3 (Cypress CYUSB3014-BZX)
   case USB_PID_CAM_PANDA_30:        // 0x000C  Panda FX3 USB3.1 Interface
   case USB_PID_CAM_EDGE_42BI:       // 0x000F  edge 4.2 bi USB 3.1 Interface
   case USB_PID_CAM_PANDA_42BI:      // 0x0011  panda 4.2 bi USB 3.1 Interface
   case USB_PID_CAM_EDGE_260:        // 0x0013  edge 26 USB 3.1 Interface
    ep_contr_in = USB_EP_FX3_IMG_IN;
    break;

   default:
     writelog(ERROR_M,(PCO_HANDLE)1,"usb_get_endpoints: No image endpoint avaiable");
     err=PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
     break;
  }

  for(int  i=0; (i<eptCount)&&(err==PCO_NOERROR);  i++)
  {
   if(libusb_ConfigDescr->interface[0].altsetting[alt_set].endpoint[i].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK)
   {
    if(libusb_ConfigDescr->interface[0].altsetting[alt_set].endpoint[i].bEndpointAddress==ep_contr_in)
    {
     ImageInAddr = libusb_ConfigDescr->interface[0].altsetting[alt_set].endpoint[i].bEndpointAddress;
     writelog(INIT_M,hgrabber,"usb_get_endpoints: BulkControlIn found (EP 0x%02x) %d", ImageInAddr, i);
     ImageInMaxPacketSize=libusb_ConfigDescr->interface[0].altsetting[alt_set].endpoint[i].wMaxPacketSize;
     writelog(INIT_M,hgrabber, "get_endpoints: ImageInMaxPacketSize %d",ImageInMaxPacketSize);
    }
   }
  }
  if(libusb_ConfigDescr)
  {
   libusb_free_config_descriptor(libusb_ConfigDescr);
   libusb_ConfigDescr = NULL;
  }


  if(ImageInAddr==0)
   err=PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  else
  {
   last_buffer_1=(unsigned char*)malloc(ImageInMaxPacketSize*2);
   last_buffer_2=(unsigned char*)malloc(ImageInMaxPacketSize*2);
  }
  return err;
}


///////////////////////////////////////////////////
//
// Tries to clear any pending USB inputs
//
///////////////////////////////////////////////////

DWORD CPco_grab_usb::usb_clear_input()
{
  DWORD err = PCO_NOERROR;
  int Len;
  int totallength = 0;
  unsigned char *buf;
  buf=(unsigned char*)malloc(ImageInMaxPacketSize);
  Len = ImageInMaxPacketSize;

//while there are packets to read
  while(Len!=0)
  {
   err=libusb_bulk_transfer(libusb_hdev, ImageInAddr, buf, Len, &Len,1);
   totallength+=Len;
  }
  if(err!=0)
  {
   writelog(PROCESS_M,hgrabber,"usb_clear_input: nothing to clear");
   err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
  }
  else
  {
   writelog(COMMAND_M,hgrabber,"usb_clear_input: transfer done *buf 0x%04X  bytes read: %d",*((unsigned short*)buf),totallength);
  }
  free(buf);

  return err;
}

DWORD CPco_grab_usb::usb_clear_input_async()
{
  DWORD err = PCO_NOERROR;
  int totallength;
  unsigned char *buf;
  libusb_transfer **async_transfer; 
  struct timeval tv;

  async_packets_done=async_clear_len=0;

  buf=(unsigned char*)malloc(ImageInMaxPacketSize+100);
  async_transfer=(libusb_transfer**)malloc(1*sizeof(libusb_transfer*));
  async_transfer[0]=libusb_alloc_transfer(0);

  totallength=0;
  for(int x=0;x<5;x++)
  {
   libusb_fill_bulk_transfer(async_transfer[0],libusb_hdev,ImageInAddr,buf,ImageInMaxPacketSize,&this->async_clear_callback,(void*)this,0);
   err=libusb_submit_transfer(async_transfer[0]);
   if(err!=0)
    writelog(ERROR_M,hgrabber,"usb_clear_input_async: submit transfer failed. Error: %s",libusb_error_name((err)));
   else
   {
    writelog(INTERNAL_3_M,hgrabber,"usb_clear_input_async: submit done");

    tv.tv_usec=1000;
    tv.tv_sec=0;
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);
    totallength+=async_clear_len;
    if(async_clear_len<ImageInMaxPacketSize)
     break;
   }
  }

  if(err!=0)
  {
   err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
  }
  else
  {
   if(async_packets_done==0)
   {
    tv.tv_usec=500;
    tv.tv_sec=0;
    libusb_cancel_transfer(async_transfer[0]);
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);
    writelog(PROCESS_M,hgrabber,"usb_clear_input_async: nothing to clear");
   }
   writelog(PROCESS_M,hgrabber,"usb_clear_input_async: transfer done *buf 0x%04X  bytes read: %d",*((unsigned short*)buf),totallength);
  }

  libusb_free_transfer(async_transfer[0]);
  free(async_transfer);
  free(buf);
  return err;
}

void CPco_grab_usb::async_clear_callback(struct libusb_transfer *transfer)
{
  CPco_grab_usb* mygrab;
  mygrab=(CPco_grab_usb*)transfer->user_data;

  mygrab->async_clear_len=transfer->actual_length;
  if(transfer->status==LIBUSB_TRANSFER_COMPLETED)
  {
   mygrab->async_packets_done++;
   mygrab->writelog(INTERNAL_3_M,mygrab->hgrabber,"async_clear_callback: transfer complete adr %p len %d",transfer->buffer,transfer->actual_length);
  }
  else if(transfer->status==LIBUSB_TRANSFER_CANCELLED)
  {
   mygrab->writelog(INTERNAL_3_M,mygrab->hgrabber,"async_clear_callback: transfer cancel adr %p",transfer->buffer);
  }
  else
  {
   if(transfer->status==LIBUSB_TRANSFER_ERROR)
    mygrab->writelog(ERROR_M,mygrab->hgrabber,"async_clear_callback: transfer error");
   if(transfer->status==LIBUSB_TRANSFER_TIMED_OUT)
    mygrab->writelog(ERROR_M,mygrab->hgrabber,"async_clear_callback: transfer timeout");
   if(transfer->status==LIBUSB_TRANSFER_STALL)
    mygrab->writelog(ERROR_M,mygrab->hgrabber,"async_clear_callback: transfer stall");
   if(transfer->status==LIBUSB_TRANSFER_NO_DEVICE)
    mygrab->writelog(ERROR_M,mygrab->hgrabber,"async_clear_callback: transfer no device");
   if(transfer->status==LIBUSB_TRANSFER_OVERFLOW)
    mygrab->writelog(ERROR_M,mygrab->hgrabber,"async_clear_callback: transfer overflow");
  }
}


///////////////////////////////////////////////////////////////////
//
// Get a single image from USB where timing isn't critical
// If you want to get multiple images, take a look at the
// asynchronous functions
//
////////////////////////////////////////////////////////////////////

DWORD CPco_grab_usb::usb_read_image(void *buf,int buflen,DWORD ima_timeout,WORD Segment,DWORD ImageNr)
{
  DWORD err;
  int done,Len,tdone;
  unsigned char *bufin;
  int timeout;
  DWORD tc1,tc2;
  int transfer_size,copy_size;
  pthread_t pthread;
  unsigned char *copybuffer;

  bufin=(unsigned char*)buf;
  copy_size = 0;

  if((hgrabber==(PCO_HANDLE)NULL)||(cam==NULL))
  {
   writelog(ERROR_M,hgrabber,"%s: grabber class not initialized",__FUNCTION__);
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  transfer_size=async_transfer_size;   

//for smaller images
  if(transfer_size > buflen)
   transfer_size = buflen;

//transfer size has to be a multiple of ImageInMaxPacketSize
  if(transfer_size%ImageInMaxPacketSize)
   transfer_size=(async_transfer_size/ImageInMaxPacketSize)*ImageInMaxPacketSize;

//calculate number of transfers 
  Len = transfer_size;
  if(buflen%transfer_size)
  {
   int num=buflen/transfer_size;
   int last_size=buflen-transfer_size*num;

 //same check as above for last size, the copy buffer removes excess bytes at the end
   if(last_size%ImageInMaxPacketSize)
   {
    last_size=(last_size/ImageInMaxPacketSize)*ImageInMaxPacketSize;
    copy_size=ImageInMaxPacketSize*4;
    copybuffer = (unsigned char*)malloc(copy_size);
    memset(copybuffer,0,copy_size);
   }
   writelog(INTERNAL_3_M,hgrabber,"usb_read_image: last size %d",last_size);
   if(num==0)
    Len=last_size;
  }

  done=0;   //counts transferred bytes
  timeout=ima_timeout; //wait for first transfer with image timeout
  tdone=0;
  tc2=tc1=GetTickCount();

  writelog(PROCESS_M,hgrabber,"usb_read_image: buflen %d first Len %d copy_size %d timeout %d",buflen,Len,copy_size,timeout);

  cam->set_transfer_msgwait(0);

  if((Segment==0)||(ImageNr==0))
   err=cam->PCO_RequestImage();
  else
   err=cam->PCO_ReadImagesFromSegment(Segment,ImageNr,ImageNr);

  writelog(INTERNAL_3_M,hgrabber,"usb_async_image: RequestImage or ReadImage returned 0x%x",err);
  if(err!=PCO_NOERROR)
  {
   writelog(ERROR_M,hgrabber,"usb_read_image: RequestImage or ReadImage returned 0x%x",err);
   return err;
  }


  if(gl_Coding)
   writelog(INTERNAL_3_M,hgrabber,"Use single thread for decoding");
  coding_buffer=NULL;
  coding_buflen=0;

//while we are not done ...
  while(done<((buflen/ImageInMaxPacketSize)*ImageInMaxPacketSize)) //remove partial packet length at the end
  {

   writelog(INTERNAL_3_M,hgrabber,"usb_read_image: call bulk_transfer buffer %p bytes %d timeout %d",bufin,Len,timeout);
   err=libusb_bulk_transfer(libusb_hdev, ImageInAddr, bufin, Len, &Len, timeout);
   if(err!=0)
   {
    writelog(ERROR_M,hgrabber,"usb_read_image: libusb_bulk_transfer failed (%d) %s", err,libusb_error_name(err));
    err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
    break;
   }

   writelog(INTERNAL_3_M,hgrabber,"usb_read_image: transfer done buffer %p bytes read: %d",bufin,Len);

   if(gl_Coding)
   {
//set parameters for gl_coding thread for next block
    if((coding_buffer!=NULL)&&(done>0))
    {
//the coding thread for the last block should be finished here
     pthread_join(pthread,NULL);
     writelog(INTERNAL_3_M,hgrabber,"usb_read_image: pthread_join done");
    }
    coding_buffer=(WORD*)bufin;
    coding_buflen=Len;
    err=pthread_create(&pthread, NULL, this->CodingThreadfunc,(void*)this);
    if(err != 0)
    {
     writelog(INTERNAL_3_M,hgrabber,"Error creating pthread, using non threaded decoding");
     Pixelfly_decode((WORD*)bufin,Len);
    }
   }

   bufin+=Len;
   done+=Len;

   if((done+transfer_size)>buflen)
   {
    Len=buflen-done;
//transfer size has to be a multiple of ImageInMaxPacketSize
    if(Len%ImageInMaxPacketSize)
    {
     Len=(Len/ImageInMaxPacketSize)*ImageInMaxPacketSize;
    }
   }
   else
    Len=transfer_size;
   timeout=packet_timeout;

   writelog(INTERNAL_3_M,hgrabber,"usb_read_image: done %d remaining %d Len %d next adr %p",done,buflen-done,Len,bufin);

   tc2=GetTickCount();
   if((tc2-tc1)>ima_timeout)
   {
    writelog(ERROR_M,hgrabber,"usb_read_image: done %d buflen %d  tc2-tc1 %d > ima_timeout %d",done,buflen,(tc2-tc1),ima_timeout);
    err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
    break;
   }
  }

  if((copy_size)&&(err==PCO_NOERROR))
  {
   Len = copy_size;
   err=libusb_bulk_transfer(libusb_hdev, ImageInAddr, copybuffer, Len, &Len, timeout);
   if(err!=0)
   {
    writelog(ERROR_M,hgrabber,"usb_read_image: Copy buffer libusb_bulk_transfer failed (%d) %s", err,libusb_error_name(err));
    err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
   }
   else
   {
    writelog(INTERNAL_3_M,hgrabber,"usb_read_image: Copy buffer Len %d done %d remaining %d ",Len,done,buflen-done);

    if(gl_Coding)
     Pixelfly_decode((WORD*)copybuffer,Len);
    memcpy(bufin,copybuffer,Len);
    writelog(INTERNAL_3_M,hgrabber,"usb_read_image: memcpy(%p,%p,%d)",bufin,copybuffer,Len);
    done+=Len; 
   }
  }

  tc2=GetTickCount();
  writelog(INTERNAL_3_M,hgrabber,"usb_read_image: data transfer done %d time %dms",done,tc2-tc1);
  while(err==PCO_NOERROR)
  {
   writelog(PROCESS_M,hgrabber,"usb_read_image: wait for IMAGE_TRANSFER_DONE message");

   if((tdone=cam->transfer_msgwait())!=0)
    break;
   Sleep_ms(5);
   tc2=GetTickCount();
   if((tc2-tc1)>ima_timeout)
   {
    writelog(ERROR_M,hgrabber,"usb_read_image: wait transfer msg tc2-tc1 %d > ima_timeout %d",(tc2-tc1),ima_timeout);
    err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
    break;
   }
  }

  if(gl_Coding&&(coding_buffer!=NULL))
  {
//thread should be finished here
   pthread_join(pthread,NULL);
   writelog(INTERNAL_3_M,hgrabber,"usb_read_image: gl_Coding pthread_join done");
  }

  if(err!=PCO_NOERROR)
  {   
   cam->PCO_CancelImage();
   usb_clear_input();
  }
  else if(ImageInAddr==USB_EP_FX3_IMG_IN)
   usb_clear_input_async();

  writelog(PROCESS_M,hgrabber,"usb_read_image: bytes read: %d, transfer message received %d, return 0x%x",done,tdone,err);
  return err;
}

////////////////////////////////////////////////////////////
//
// Asynchronous image grabbing
// *buf = address to the receive buffer
//  buflen = length of the buffer
// ima_timeout = time in ms after which the transfer stops with a timeout error
// waitforimage = if gl_coding is on (should be on if possible for pco.pixelfly) the images are only immediately valid if this flag is set to TRUE!
// if you are grabbing a larger number of images, you only have to set the flag for the last image
// (the function waits until ALL acquired images up to this point are valid!)
//
/////////////////////////////////////////////////////////////

DWORD CPco_grab_usb::usb_async_image(void *buf,int buflen,DWORD ima_timeout, BOOL waitforimage)
{
  unsigned char *tadr;
  DWORD tc1,tc2;
  DWORD err=PCO_NOERROR;
  int request_send=0;
  int async_bytes_setup,last_done;
  int num_set;
  int tdone;

  if((hgrabber==(PCO_HANDLE)NULL)||(cam==NULL))
  {
   writelog(ERROR_M,hgrabber,"%s: grabber class not initialized",__FUNCTION__);
   return PCO_ERROR_DRIVER_NODRIVER | PCO_ERROR_DRIVER_USB;
  }

  writelog(PROCESS_M,hgrabber,"usb_async_image: start buflen %d async_transfer_size %d timeout %d",buflen,async_transfer_size,ima_timeout);
  if((async_transfer_status&ASYNC_ALLOC_1)==0)
  {
   writelog(ERROR_M,hgrabber,"usb_async_image: async_transfer_status not alloc");
   return PCO_ERROR_DRIVER_IOFAILURE | PCO_ERROR_DRIVER_USB;
  }

  if(buflen<act_dmalength)
  {
   writelog(ERROR_M,hgrabber,"usb_async_image: buflen %d < act_dmalength %d ",buflen,act_dmalength);
   return PCO_ERROR_BUFFERSIZE | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
  }

  tadr=(unsigned char *)buf;
  async_buffer_adr=tadr;
  async_bytes_done=async_packets_done=async_bytes_setup=0;
  last_done=0;

  async_transfer_status|=ASYNC_PENDING_1;
  async_transfer_status&=~ASYNC_ERROR;

  int num=async_transfer_num;
  if(async_transfer_status&ASYNC_NEED_LAST)
  {
   num--;
  }
  if(async_transfer_status&ASYNC_USE_COPY_BUFFER)
   num--;

  cam->set_transfer_msgwait(0);

//Generate the request packets and submit them all at once
//the function async_callback() is invoked as soon as a partial transfer is completed
//the image request is only sent to the camera after the first request has been submitted (speeds up the process a bit)
  tc2=tc1=GetTickCount();
  for(num_set=0;num_set<num;)
  {
//only 16Mbyte of Data can be setup in libusb, if image size is larger we have to wait until the first packages are done
   if(async_bytes_setup+async_transfer_size<16*1024*1024)
   {
//do not use timeout for the first transfer, we have to handle triggered images too
//we will cancel the transfers if the image timeout timed out
    libusb_fill_bulk_transfer(async_transfer_1[num_set],libusb_hdev,ImageInAddr,tadr,async_transfer_size,&this->async_callback,(void*)this,0);

    err=libusb_submit_transfer(async_transfer_1[num_set]);
    if(err!=0)
    {
     writelog(ERROR_M,hgrabber,"usb_async_image: submit transfer failed. Error: %s",libusb_error_name((err)));
     err=PCO_NOERROR;
    }
    writelog(INTERNAL_3_M,hgrabber,"usb_async_image: submit async_transfer_1[%d] len %d request_send %d",num_set,async_transfer_1[num_set]->length,request_send);
    tadr+=async_transfer_size;
    async_bytes_setup+=async_transfer_size;
    num_set++;

    if((num>1)&&(request_send==0))
    {
     err=cam->PCO_RequestImage();
     writelog(INTERNAL_3_M,hgrabber,"usb_async_image: RequestImage returned 0x%x",err);
     if(err==PCO_NOERROR)
      request_send=1;
     else
      break;
    }
   }
   else
   {
    struct timeval tv;
    if(async_packets_done==0)
     tv.tv_usec=500000;
    else
     tv.tv_usec=20000;
    tv.tv_sec=0;
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);

    tc2=GetTickCount();
    writelog(INTERNAL_3_M,hgrabber,"usb_async_image: packets_done %d bytes_done %d  tc2-tc1 %d transfer_num %d"
                                ,async_packets_done,async_bytes_done,(tc2-tc1),async_transfer_num);

    if((tc2-tc1)>ima_timeout)
    {
     writelog(ERROR_M,hgrabber,"usb_async_image: done %d buflen %d  tc2-tc1 %d > ima_timeout %d",async_bytes_done,buflen,(tc2-tc1),ima_timeout);
     err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
     break;
    }
    if(async_packets_done>last_done)
    {
     async_bytes_setup-=(async_transfer_size*(async_packets_done-last_done));
     last_done=async_packets_done;
    }
   }
  }

  if(err==PCO_NOERROR)
  {
   if(async_transfer_status&ASYNC_NEED_LAST)
   {
    libusb_fill_bulk_transfer(async_transfer_1[num_set],libusb_hdev,ImageInAddr,tadr,async_last_size,&this->async_callback,(void*)this,0);
    libusb_submit_transfer(async_transfer_1[num_set]);
    tadr+=async_last_size;
    writelog(PROCESS_M,hgrabber,"usb_async_image: submit last async_transfer_1[%d] len %d",num_set,async_transfer_1[num_set]->length);
    num_set++;
   }

   if(async_transfer_status&ASYNC_USE_COPY_BUFFER)
   {
    libusb_fill_bulk_transfer(async_transfer_1[num_set],libusb_hdev,ImageInAddr,last_buffer_1,ImageInMaxPacketSize*2,&this->async_callback,(void*)this,0);
    libusb_submit_transfer(async_transfer_1[num_set]);
    writelog(PROCESS_M,hgrabber,"usb_async_image: submit copy buffer async_transfer_1[%d] len %d",num_set,async_transfer_1[num_set]->length);
    num_set++;
   }
  }

  tc2=tc1=GetTickCount();
  async_transfer_status|=ASYNC_SETUP_1;

  if((request_send==0)&&(err==PCO_NOERROR))
  {
   err=cam->PCO_RequestImage();
   writelog(INTERNAL_3_M,hgrabber,"usb_async_image: RequestImage returned 0x%x",err);
   if(err==PCO_NOERROR)
    request_send=1;
  }

  if(err!=PCO_NOERROR)
  {
   struct timeval tv;
   tv.tv_usec=1000;  
   tv.tv_sec=0;

//cancel all transfers
   for(int i=0;i<num_set;i++)
   {
    writelog(PROCESS_M,hgrabber,"usb_async_image: cancel transfer[%d]",i);
    libusb_cancel_transfer(async_transfer_1[i]);
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);
   }
   num_set=0;
  }
  else
  {
//Wait for all packets to arrive
   while(async_packets_done<async_transfer_num)
   {
    struct timeval tv;

    if(async_packets_done==0)
     tv.tv_usec=500000; //check every 500ms when waiting for first package
    else
     tv.tv_usec=30000;  //check every 30ms if transfer is running
    tv.tv_sec=0;
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);

    tc2=GetTickCount();
    if(async_packets_done>last_done)
    {
     writelog(INTERNAL_3_M,hgrabber,"usb_async_image: while packets_done %d bytes_done %d  tc2-tc1 %d transfer_num %d"
                               ,async_packets_done,async_bytes_done,(tc2-tc1),async_transfer_num);
     last_done=async_packets_done;
    }
    else
    {
     writelog(PROCESS_M,hgrabber,"usb_async_image: no packets arrived. packets_done %d",async_packets_done);
    }
    if((tc2-tc1)>ima_timeout)
    {
     writelog(ERROR_M,hgrabber,"usb_async_image: done %d buflen %d  tc2-tc1 %d > ima_timeout %d",async_bytes_done,buflen,(tc2-tc1),ima_timeout);
     err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
     break;
    }
   }
  }

  if((err!=PCO_NOERROR)||(async_transfer_status&ASYNC_ERROR))
  {
   struct timeval tv;
   tv.tv_sec=0;
   tv.tv_usec=1000;

   writelog(ERROR_M,hgrabber,"usb_async_image: err 0x%x  async_transfer_status 0x%x & 0x%x",err,async_transfer_status,ASYNC_ERROR);

   for(int i=async_packets_done;(i<num_set)&&(i<async_transfer_num);i++)
   {
    libusb_cancel_transfer(async_transfer_1[i]);
    libusb_handle_events_timeout(cam->libusb_ctx,&tv);
   }
   writelog(ERROR_M,hgrabber,"usb_async_image: found error after cancel");
   if(async_transfer_status&ASYNC_ERROR)
    err= PCO_ERROR_DRIVER_IOFAILURE | PCO_ERROR_DRIVER_USB;
  }
  tc2=GetTickCount();

  async_transfer_status&=~(ASYNC_SETUP_1|ASYNC_PENDING_1);
  if(err==PCO_NOERROR)
  {
   writelog(PROCESS_M,hgrabber,"usb_async_image: all packages done %d time %dms async_transfer_status 0x%x"
                              ,async_packets_done,tc2-tc1,async_transfer_status);

   if(async_transfer_status&ASYNC_USE_COPY_BUFFER)
   {
            //int s=(int)(tadr-(unsigned char*)buf+buflen);
    memcpy(tadr,last_buffer_1,async_copy_buffer_size);
    writelog(INTERNAL_3_M,hgrabber,"usb_async_image: memcpy(%p,%p,%d)",tadr,last_buffer_1,async_copy_buffer_size);
   }

//Catch the "image done" telegram
   while(err==PCO_NOERROR)
   {
    writelog(PROCESS_M,hgrabber,"usb_async_image: wait for IMAGE_TRANSFER_DONE message");
    if((tdone=cam->transfer_msgwait())!=0)
     break;

    Sleep_ms(5);
    tc2=GetTickCount();
    if((tc2-tc1)>ima_timeout)
    {
     writelog(ERROR_M,hgrabber,"usb_async_image: wait transfer msg tc2-tc1 %d > ima_timeout %d",(tc2-tc1),ima_timeout);
     err=PCO_ERROR_TIMEOUT | PCO_ERROR_DRIVER | PCO_ERROR_DRIVER_USB;
     break;
    }
   }
  }

  if(err!=PCO_NOERROR)
  {
   cam->PCO_CancelImage();
   usb_clear_input();
  }
  else if(ImageInAddr==USB_EP_FX3_IMG_IN)
   usb_clear_input_async();

//wait option set
#ifdef ASYNC_THREAD
  if (waitforimage & gl_Coding)
  {
    //wait for coding thread to finish
   for (unsigned int i = 0;i<gl_codingthreads.size();i++)
   {
    pthread_join(*gl_codingthreads[i],NULL);
    delete gl_codingthreads[i];
   }
   gl_codingthreads.clear();
   writelog(PROCESS_M,hgrabber,"Finished decoding threads.");
  }
#endif
  writelog(PROCESS_M,hgrabber,"usb_async_image: bytes read: %d, transfer message received %d, return 0x%x",async_bytes_done,tdone,err);

  return err;
}

//Callback function that is invoked when a packet arrives
//The function starts gl_coding directly if necessary
//Not need for threading here because transfer from usb is done in the background
//and decoding is fast enough  


void CPco_grab_usb::async_callback(struct libusb_transfer *transfer)
{
  CPco_grab_usb* mygrab;
  mygrab=(CPco_grab_usb*)transfer->user_data;

  if(transfer->status==LIBUSB_TRANSFER_COMPLETED)
  {
   mygrab->async_packets_done++;
   mygrab->async_bytes_done+=transfer->actual_length;
   mygrab->writelog(INTERNAL_3_M,mygrab->hgrabber,"async callback: transfer complete adr %p next %p",transfer->buffer,transfer->buffer+transfer->actual_length);

   if(mygrab->gl_Coding)
   {
    mygrab->writelog(INTERNAL_3_M,mygrab->hgrabber,"Start decoding function");
    if((transfer->buffer!=NULL)&&(transfer->actual_length>0))
     mygrab->Pixelfly_decode((WORD*)transfer->buffer,transfer->actual_length);
   }
  }
  else if(transfer->status==LIBUSB_TRANSFER_CANCELLED)
  {
   mygrab->writelog(PROCESS_M,mygrab->hgrabber,"async callback: transfer cancel adr %p",transfer->buffer);
  }
  else
  {
   mygrab->async_transfer_status|=ASYNC_ERROR;
  }

  if(transfer->status==LIBUSB_TRANSFER_ERROR)
   mygrab->writelog(ERROR_M,mygrab->hgrabber,"async callback: transfer error");
  if(transfer->status==LIBUSB_TRANSFER_TIMED_OUT)
   mygrab->writelog(ERROR_M,mygrab->hgrabber,"async callback: transfer timeout");
  if(transfer->status==LIBUSB_TRANSFER_STALL)
   mygrab->writelog(ERROR_M,mygrab->hgrabber,"async callback: transfer stall");
  if(transfer->status==LIBUSB_TRANSFER_NO_DEVICE)
   mygrab->writelog(ERROR_M,mygrab->hgrabber,"async callback: transfer no device");
  if(transfer->status==LIBUSB_TRANSFER_OVERFLOW)
   mygrab->writelog(ERROR_M,mygrab->hgrabber,"async callback: transfer overflow");
}

//This is used by the single thread decoding
//does use public class parameters 'coding_buffer' and 'coding_buflen'
//which are set from usb_read_image, when a bulk transfer has been done
void* CPco_grab_usb::CodingThreadfunc(void* param)
{
  CPco_grab_usb* mygrab;
  mygrab=reinterpret_cast<CPco_grab_usb*>(param);

  if((mygrab->coding_buffer!=NULL)&&(mygrab->coding_buflen>0))
   mygrab->Pixelfly_decode(mygrab->coding_buffer,mygrab->coding_buflen);
  return 0;
}

void CPco_grab_usb::Sleep_ms(int time) //time in ms
{
#ifdef __linux__
  int ret_val;
  fd_set rfds;
  struct timeval tv;

  FD_ZERO(&rfds);
  FD_SET(0,&rfds);
  tv.tv_sec=time/1000;
  tv.tv_usec=(time%1000)*1000;
  ret_val=select(1,NULL,NULL,NULL,&tv);
  if(ret_val<0)
   writelog(ERROR_M,hgrabber,"Sleep: error in select");
#endif
#ifdef WIN32
  if(time<25)
   time=25;
  Sleep(time);
#endif
}

void CPco_grab_usb::Pixelfly_decode(WORD* buffer, int len)
{
  WORD* adrdc = (WORD*) buffer;
  WORD bits;
  WORD tempadr;
  int c=0;

  if(act_align == 0)
  {
   WORD bittable[4] = {0x1040,0x1000,0x0040,0x0000};  // Table for this alignment
   for(LONG ly=0;ly<len/2;ly++)
   {
    tempadr = *(adrdc+ly);             // Get entry from memory
    bits = tempadr & 0x0003;           // Get "Reserve" bits (0+1)
    tempadr |= bittable[bits];         // logical OR with the appropriate table entry
    tempadr &= 0xFFFC;                 // Set "Reserve" bits to 0
    *(adrdc+ly) = tempadr;             // write entry back to memory
    c++;
   }
   writelog(INTERNAL_3_M,hgrabber,"Pixelfly_decode: align 0 adr %p done %d",buffer,c);
  }
  else if(act_align == 1)
  {
   WORD bittable[4] = {0x0410,0x0400,0x0010,0x0000};
   for(LONG ly=0;ly<len/2;ly++)
   {
    tempadr = *(adrdc+ly);
    bits = tempadr & 0xC000;          // Get "Reserve" bits (14+15)
    bits >>= 14;                      // Shift bits to 0+1
    tempadr |= bittable[bits];        // logical OR with the appropriate table entry
    tempadr &= 0x3FFF;                // Set "Reserve bits to 0
    *(adrdc+ly) = tempadr;
    c++;
   }
   writelog(INTERNAL_3_M,hgrabber,"Pixelfly_decode: align 1 adr %p done %d",buffer,c*2);
  }
  else
   writelog(ERROR_M,hgrabber,"Pixelfly_decode: wrong alignment");
}


#ifndef WIN32
DWORD CPco_grab_usb::GetTickCount(void)
{
  struct timeval t;
  gettimeofday(&t,NULL);
  return(t.tv_usec/1000+t.tv_sec*1000);
}
#endif

