/* Force-assign the inbox WinUSB driver ("WinUsb Device" from winusb.inf) to
   a given device instance, replicating Device Manager's manual driver
   selection. Must run elevated. */
#include <windows.h>
#include <setupapi.h>
#include <newdev.h>
#include <cstdio>
#include <cstring>

typedef BOOL (WINAPI* DiInstallDeviceFunc)(HWND,HDEVINFO,PSP_DEVINFO_DATA,PSP_DRVINFO_DATA,DWORD,PBOOL);

int main(int argc,char* argv[])
	{
	const char* instanceId="USB\\VID_2833&PID_0211&MI_00\\6&34481FE8&0&0000";
	const char* infPath="C:\\Windows\\INF\\winusb.inf";
	const char* targetDesc="WinUsb Device";
	if(argc>1)
		instanceId=argv[1];

	HDEVINFO devs=SetupDiCreateDeviceInfoList(NULL,NULL);
	if(devs==INVALID_HANDLE_VALUE)
		{
		printf("SetupDiCreateDeviceInfoList failed: %lu\n",GetLastError());
		return 1;
		}

	SP_DEVINFO_DATA devInfo;
	memset(&devInfo,0,sizeof(devInfo));
	devInfo.cbSize=sizeof(devInfo);
	if(!SetupDiOpenDeviceInfoA(devs,instanceId,NULL,0,&devInfo))
		{
		printf("SetupDiOpenDeviceInfo(%s) failed: %lu\n",instanceId,GetLastError());
		return 1;
		}

	/* Change the devnode's setup class to USBDevice so class-driver
	   enumeration considers winusb.inf (Device Manager does the same when a
	   driver of a different class is hand-picked): */
	const char* usbDeviceClass="{88BAE032-5A81-49f0-BC3D-A4FF138216D6}"; // ClassGuid declared by this build's winusb.inf
	if(!SetupDiSetDeviceRegistryPropertyA(devs,&devInfo,SPDRP_CLASSGUID,(const BYTE*)usbDeviceClass,DWORD(strlen(usbDeviceClass)+1)))
		{
		printf("SetDeviceRegistryProperty(SPDRP_CLASSGUID) failed: %lu\n",GetLastError());
		return 1;
		}
	printf("Device class changed to USBDevice\n");

	/* Restrict driver search to winusb.inf only: */
	SP_DEVINSTALL_PARAMS_A dip;
	memset(&dip,0,sizeof(dip));
	dip.cbSize=sizeof(dip);
	if(!SetupDiGetDeviceInstallParamsA(devs,&devInfo,&dip))
		{
		printf("GetDeviceInstallParams failed: %lu\n",GetLastError());
		return 1;
		}
	strncpy(dip.DriverPath,infPath,sizeof(dip.DriverPath)-1);
	dip.Flags|=DI_ENUMSINGLEINF;
	dip.FlagsEx|=DI_FLAGSEX_ALLOWEXCLUDEDDRVS;
	if(!SetupDiSetDeviceInstallParamsA(devs,&devInfo,&dip))
		{
		printf("SetDeviceInstallParams failed: %lu\n",GetLastError());
		return 1;
		}

	/* Build the class driver list from that INF (lists all models regardless of ID match): */
	if(!SetupDiBuildDriverInfoList(devs,&devInfo,SPDIT_CLASSDRIVER))
		{
		printf("SetupDiBuildDriverInfoList failed: %lu\n",GetLastError());
		return 1;
		}

	SP_DRVINFO_DATA_A drv;
	memset(&drv,0,sizeof(drv));
	drv.cbSize=sizeof(drv);
	bool found=false;
	for(DWORD i=0;SetupDiEnumDriverInfoA(devs,&devInfo,SPDIT_CLASSDRIVER,i,&drv);++i)
		{
		printf("Driver %lu: '%s' provider '%s'\n",i,drv.Description,drv.ProviderName);
		if(strcmp(drv.Description,targetDesc)==0)
			{
			found=true;
			break;
			}
		}
	if(!found)
		{
		printf("No '%s' model found in %s (last error %lu)\n",targetDesc,infPath,GetLastError());
		return 1;
		}

	if(!SetupDiSetSelectedDriverA(devs,&devInfo,&drv))
		{
		printf("SetupDiSetSelectedDriver failed: %lu\n",GetLastError());
		return 1;
		}

	HMODULE newdev=LoadLibraryA("newdev.dll");
	DiInstallDeviceFunc diInstallDevice=newdev!=0?(DiInstallDeviceFunc)GetProcAddress(newdev,"DiInstallDevice"):0;
	if(diInstallDevice==0)
		{
		printf("Cannot load DiInstallDevice from newdev.dll: %lu\n",GetLastError());
		return 1;
		}

	/* DiInstallDevice is Unicode-only; re-fetch the selected driver as a wide struct: */
	SP_DRVINFO_DATA_W drvW;
	memset(&drvW,0,sizeof(drvW));
	drvW.cbSize=sizeof(drvW);
	if(!SetupDiGetSelectedDriverW(devs,&devInfo,&drvW))
		{
		printf("SetupDiGetSelectedDriverW failed: %lu\n",GetLastError());
		return 1;
		}

	BOOL reboot=FALSE;
	if(!diInstallDevice(NULL,devs,&devInfo,(PSP_DRVINFO_DATA)&drvW,0,&reboot))
		{
		printf("DiInstallDevice failed: %lu\n",GetLastError());
		return 1;
		}
	printf("SUCCESS: WinUsb Device installed on %s (reboot needed: %d)\n",instanceId,int(reboot));

	SetupDiDestroyDeviceInfoList(devs);
	return 0;
	}
