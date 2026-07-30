/* Quick probe: can we open the Rift Sensor MI_00 via its device interface
   path and drive it with the native WinUsb API (i.e., is ocusbvid.sys a
   WinUSB-compatible driver)? */
#include <windows.h>
#include <winusb.h>
#include <usb.h>
#include <cstdio>

int main(int argc,char* argv[])
	{
	const char* path="\\\\?\\usb#vid_2833&pid_0211&mi_00#6&34481fe8&0&0000#{0b6cce76-2100-4c02-919e-01555ecba665}";
	if(argc>1)
		path=argv[1];
	printf("Opening %s\n",path);
	HANDLE h=CreateFileA(path,GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,0);
	if(h==INVALID_HANDLE_VALUE)
		{
		printf("CreateFile failed: %lu\n",GetLastError());
		return 1;
		}
	printf("CreateFile OK\n");

	WINUSB_INTERFACE_HANDLE wh;
	if(!WinUsb_Initialize(h,&wh))
		{
		printf("WinUsb_Initialize failed: %lu\n",GetLastError());
		CloseHandle(h);
		return 2;
		}
	printf("WinUsb_Initialize OK\n");

	/* Try reading the device descriptor: */
	USB_DEVICE_DESCRIPTOR dd;
	ULONG transferred=0;
	if(WinUsb_GetDescriptor(wh,USB_DEVICE_DESCRIPTOR_TYPE,0,0,(PUCHAR)&dd,sizeof(dd),&transferred))
		printf("Device descriptor: VID %04x PID %04x bcdUSB %04x\n",dd.idVendor,dd.idProduct,dd.bcdUSB);
	else
		printf("WinUsb_GetDescriptor failed: %lu\n",GetLastError());

	UCHAR speed=0; ULONG len=sizeof(speed);
	if(WinUsb_QueryDeviceInformation(wh,DEVICE_SPEED,&len,&speed))
		printf("Device speed: %u (1=low/full, 3=high)\n",speed);

	WinUsb_Free(wh);
	CloseHandle(h);
	printf("SUCCESS: ocusbvid is WinUSB-compatible\n");
	return 0;
	}
