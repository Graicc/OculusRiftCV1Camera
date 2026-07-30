/***********************************************************************
RiftSensorSetup - One-shot setup engine for the Rift Sensor Camera
virtual webcam. Run elevated (normally as an MSI custom action):

  RiftSensorSetup.exe install    Stop OVRService, bind the inbox WinUSB
                                 driver to every present Rift CV1 sensor
                                 video interface, set the device
                                 interface GUID, restart the devnodes,
                                 and register RiftSensorCamFilter.dll
                                 (from the same directory).
  RiftSensorSetup.exe uninstall  Unregister the filter and revert the
                                 sensors to their original driver by
                                 removing the devnodes and rescanning.
***********************************************************************/

#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

typedef BOOL (WINAPI* DiInstallDeviceFunc)(HWND,HDEVINFO,PSP_DEVINFO_DATA,PSP_DRVINFO_DATA,DWORD,PBOOL);
typedef BOOL (WINAPI* DiUninstallDeviceFunc)(HWND,HDEVINFO,PSP_DEVINFO_DATA,DWORD,PBOOL);

static FILE* logFile=0;
static void logf(const char* format,...)
	{
	va_list args;
	va_start(args,format);
	vprintf(format,args);
	va_end(args);
	if(logFile!=0)
		{
		va_start(args,format);
		vfprintf(logFile,format,args);
		va_end(args);
		fflush(logFile);
		}
	}

/* Reads the ClassGuid declared by the inbox winusb.inf (it differs between
   Windows builds), e.g. "{88BAE032-5A81-49f0-BC3D-A4FF138216D6}": */
static std::string getWinUsbInfClassGuid(void)
	{
	char windir[MAX_PATH];
	GetWindowsDirectoryA(windir,sizeof(windir));
	std::string infPath=std::string(windir)+"\\INF\\winusb.inf";

	HANDLE file=CreateFileA(infPath.c_str(),GENERIC_READ,FILE_SHARE_READ,0,OPEN_EXISTING,0,0);
	if(file==INVALID_HANDLE_VALUE)
		return "";
	DWORD size=GetFileSize(file,0);
	std::vector<char> raw(size+2,0);
	DWORD bytesRead=0;
	ReadFile(file,&raw[0],size,&bytesRead,0);
	CloseHandle(file);

	/* Convert UTF-16LE (with BOM) or ANSI content to a narrow string: */
	std::string content;
	if(size>=2&&(unsigned char)raw[0]==0xff&&(unsigned char)raw[1]==0xfe)
		{
		const wchar_t* wide=(const wchar_t*)&raw[2];
		size_t wideLen=(size-2)/2;
		for(size_t i=0;i<wideLen;++i)
			content.push_back(wide[i]<128?char(wide[i]):'?');
		}
	else
		content.assign(&raw[0],size);

	/* Find ClassGuid = {...}: */
	size_t pos=content.find("ClassGuid");
	if(pos==std::string::npos)
		return "";
	size_t braceStart=content.find('{',pos);
	size_t braceEnd=content.find('}',braceStart);
	if(braceStart==std::string::npos||braceEnd==std::string::npos)
		return "";
	return content.substr(braceStart,braceEnd-braceStart+1);
	}

/* Enumerates present devnodes whose hardware IDs contain the given match
   string and returns their device instance IDs: */
static std::vector<std::string> findDevices(const char* hardwareIdMatch)
	{
	std::vector<std::string> result;
	HDEVINFO devs=SetupDiGetClassDevsA(0,"USB",0,DIGCF_ALLCLASSES|DIGCF_PRESENT);
	if(devs==INVALID_HANDLE_VALUE)
		return result;

	SP_DEVINFO_DATA devInfo;
	memset(&devInfo,0,sizeof(devInfo));
	devInfo.cbSize=sizeof(devInfo);
	for(DWORD i=0;SetupDiEnumDeviceInfo(devs,i,&devInfo);++i)
		{
		char hardwareIds[4096];
		memset(hardwareIds,0,sizeof(hardwareIds));
		if(!SetupDiGetDeviceRegistryPropertyA(devs,&devInfo,SPDRP_HARDWAREID,0,(PBYTE)hardwareIds,sizeof(hardwareIds)-2,0))
			continue;

		/* Scan the multi-sz for the match string: */
		bool match=false;
		for(const char* id=hardwareIds;*id!='\0';id+=strlen(id)+1)
			if(strstr(id,hardwareIdMatch)!=0)
				{
				match=true;
				break;
				}
		if(!match)
			continue;

		char instanceId[512];
		if(SetupDiGetDeviceInstanceIdA(devs,&devInfo,instanceId,sizeof(instanceId),0))
			result.push_back(instanceId);
		}
	SetupDiDestroyDeviceInfoList(devs);
	return result;
	}

/* Force-binds the inbox WinUSB driver to the given device instance
   (programmatic Device Manager manual driver selection): */
static bool bindWinUsb(const std::string& instanceId)
	{
	std::string classGuid=getWinUsbInfClassGuid();
	if(classGuid.empty())
		{
		logf("  Cannot determine winusb.inf ClassGuid\n");
		return false;
		}
	logf("  winusb.inf ClassGuid: %s\n",classGuid.c_str());

	char windir[MAX_PATH];
	GetWindowsDirectoryA(windir,sizeof(windir));
	std::string infPath=std::string(windir)+"\\INF\\winusb.inf";

	HDEVINFO devs=SetupDiCreateDeviceInfoList(0,0);
	SP_DEVINFO_DATA devInfo;
	memset(&devInfo,0,sizeof(devInfo));
	devInfo.cbSize=sizeof(devInfo);
	if(!SetupDiOpenDeviceInfoA(devs,instanceId.c_str(),0,0,&devInfo))
		{
		logf("  SetupDiOpenDeviceInfo failed: %lu\n",GetLastError());
		return false;
		}

	/* Switch the devnode's setup class to the INF's class so class-driver
	   enumeration considers it: */
	if(!SetupDiSetDeviceRegistryPropertyA(devs,&devInfo,SPDRP_CLASSGUID,(const BYTE*)classGuid.c_str(),DWORD(classGuid.length()+1)))
		{
		logf("  Set class GUID failed: %lu\n",GetLastError());
		return false;
		}

	SP_DEVINSTALL_PARAMS_A dip;
	memset(&dip,0,sizeof(dip));
	dip.cbSize=sizeof(dip);
	SetupDiGetDeviceInstallParamsA(devs,&devInfo,&dip);
	strncpy(dip.DriverPath,infPath.c_str(),sizeof(dip.DriverPath)-1);
	dip.Flags|=DI_ENUMSINGLEINF;
	dip.FlagsEx|=DI_FLAGSEX_ALLOWEXCLUDEDDRVS;
	SetupDiSetDeviceInstallParamsA(devs,&devInfo,&dip);

	if(!SetupDiBuildDriverInfoList(devs,&devInfo,SPDIT_CLASSDRIVER))
		{
		logf("  SetupDiBuildDriverInfoList failed: %lu\n",GetLastError());
		return false;
		}

	SP_DRVINFO_DATA_A drv;
	memset(&drv,0,sizeof(drv));
	drv.cbSize=sizeof(drv);
	bool found=false;
	for(DWORD i=0;SetupDiEnumDriverInfoA(devs,&devInfo,SPDIT_CLASSDRIVER,i,&drv);++i)
		if(strcmp(drv.Description,"WinUsb Device")==0)
			{
			found=true;
			break;
			}
	if(!found)
		{
		logf("  No 'WinUsb Device' model found in winusb.inf\n");
		return false;
		}

	if(!SetupDiSetSelectedDriverA(devs,&devInfo,&drv))
		{
		logf("  SetupDiSetSelectedDriver failed: %lu\n",GetLastError());
		return false;
		}

	HMODULE newdev=LoadLibraryA("newdev.dll");
	DiInstallDeviceFunc diInstallDevice=newdev!=0?(DiInstallDeviceFunc)GetProcAddress(newdev,"DiInstallDevice"):0;
	if(diInstallDevice==0)
		{
		logf("  Cannot load DiInstallDevice\n");
		return false;
		}

	SP_DRVINFO_DATA_W drvW;
	memset(&drvW,0,sizeof(drvW));
	drvW.cbSize=sizeof(drvW);
	if(!SetupDiGetSelectedDriverW(devs,&devInfo,&drvW))
		{
		logf("  SetupDiGetSelectedDriverW failed: %lu\n",GetLastError());
		return false;
		}

	BOOL reboot=FALSE;
	if(!diInstallDevice(0,devs,&devInfo,(PSP_DRVINFO_DATA)&drvW,0,&reboot))
		{
		logf("  DiInstallDevice failed: %lu\n",GetLastError());
		return false;
		}
	logf("  WinUSB bound (reboot needed: %d)\n",int(reboot));

	/* Give libusb a device interface GUID to find the device by: */
	HKEY key=SetupDiCreateDevRegKeyA(devs,&devInfo,DICS_FLAG_GLOBAL,0,DIREG_DEV,0,0);
	if(key!=INVALID_HANDLE_VALUE&&key!=0)
		{
		const char guids[]="{b35924d6-3e16-4a9e-9782-5524a4b79bac}\0"; //multi-sz: value, terminator, list terminator
		RegSetValueExA(key,"DeviceInterfaceGUIDs",0,REG_MULTI_SZ,(const BYTE*)guids,sizeof(guids));
		RegCloseKey(key);
		logf("  DeviceInterfaceGUIDs set\n");
		}
	else
		logf("  Warning: cannot open device parameters key: %lu\n",GetLastError());

	/* Restart the devnode so winusb.sys picks up the interface GUID: */
	SP_PROPCHANGE_PARAMS pcp;
	memset(&pcp,0,sizeof(pcp));
	pcp.ClassInstallHeader.cbSize=sizeof(SP_CLASSINSTALL_HEADER);
	pcp.ClassInstallHeader.InstallFunction=DIF_PROPERTYCHANGE;
	pcp.StateChange=DICS_PROPCHANGE;
	pcp.Scope=DICS_FLAG_GLOBAL;
	if(SetupDiSetClassInstallParamsA(devs,&devInfo,&pcp.ClassInstallHeader,sizeof(pcp))&&
	   SetupDiCallClassInstaller(DIF_PROPERTYCHANGE,devs,&devInfo))
		logf("  Devnode restarted\n");
	else
		logf("  Warning: devnode restart failed: %lu\n",GetLastError());

	SetupDiDestroyDeviceInfoList(devs);
	return true;
	}

/* Removes the devnode and rescans; Windows re-installs the best-ranked
   driver (the original Oculus driver if present): */
static bool revertDevice(const std::string& instanceId)
	{
	HDEVINFO devs=SetupDiCreateDeviceInfoList(0,0);
	SP_DEVINFO_DATA devInfo;
	memset(&devInfo,0,sizeof(devInfo));
	devInfo.cbSize=sizeof(devInfo);
	if(!SetupDiOpenDeviceInfoA(devs,instanceId.c_str(),0,0,&devInfo))
		return false;

	HMODULE newdev=LoadLibraryA("newdev.dll");
	DiUninstallDeviceFunc diUninstallDevice=newdev!=0?(DiUninstallDeviceFunc)GetProcAddress(newdev,"DiUninstallDevice"):0;
	bool removed=false;
	if(diUninstallDevice!=0)
		{
		BOOL reboot=FALSE;
		removed=diUninstallDevice(0,devs,&devInfo,0,&reboot)!=FALSE;
		}
	SetupDiDestroyDeviceInfoList(devs);

	/* Rescan so the device reappears with its default driver: */
	DEVINST rootDevInst;
	if(CM_Locate_DevNodeA(&rootDevInst,0,CM_LOCATE_DEVNODE_NORMAL)==CR_SUCCESS)
		CM_Reenumerate_DevNode(rootDevInst,0);

	return removed;
	}

static void stopOVRService(void)
	{
	SC_HANDLE scm=OpenSCManagerA(0,0,SC_MANAGER_CONNECT);
	if(scm==0)
		return;
	SC_HANDLE service=OpenServiceA(scm,"OVRService",SERVICE_STOP|SERVICE_QUERY_STATUS);
	if(service!=0)
		{
		SERVICE_STATUS status;
		if(ControlService(service,SERVICE_CONTROL_STOP,&status))
			logf("OVRService stopped\n");
		CloseServiceHandle(service);
		}
	CloseServiceHandle(scm);
	}

/* Registers or unregisters RiftSensorCamFilter.dll from this exe's directory: */
static bool registerFilter(bool install)
	{
	char path[MAX_PATH];
	GetModuleFileNameA(0,path,sizeof(path));
	char* lastSlash=strrchr(path,'\\');
	if(lastSlash!=0)
		lastSlash[1]='\0';
	strcat_s(path,sizeof(path),"RiftSensorCamFilter.dll");

	HMODULE dll=LoadLibraryA(path);
	if(dll==0)
		{
		logf("Cannot load %s: %lu\n",path,GetLastError());
		return false;
		}
	typedef HRESULT (WINAPI* RegisterFunc)(void);
	RegisterFunc func=(RegisterFunc)GetProcAddress(dll,install?"DllRegisterServer":"DllUnregisterServer");
	if(func==0)
		{
		logf("Cannot find registration entry point\n");
		FreeLibrary(dll);
		return false;
		}
	HRESULT result=func();
	FreeLibrary(dll);
	if(FAILED(result))
		{
		logf("%s failed: 0x%08lx\n",install?"DllRegisterServer":"DllUnregisterServer",(unsigned long)result);
		return false;
		}
	logf("Filter %s\n",install?"registered":"unregistered");
	return true;
	}

int main(int argc,char* argv[])
	{
	/* Log to %TEMP%\RiftSensorSetup.log for MSI diagnostics: */
	char logPath[MAX_PATH];
	if(GetTempPathA(sizeof(logPath),logPath)!=0)
		{
		strcat_s(logPath,sizeof(logPath),"RiftSensorSetup.log");
		logFile=fopen(logPath,"a");
		}

	bool install;
	if(argc>=2&&strcmp(argv[1],"install")==0)
		install=true;
	else if(argc>=2&&strcmp(argv[1],"uninstall")==0)
		install=false;
	else
		{
		printf("Usage: %s install|uninstall\n",argv[0]);
		return 1;
		}
	logf("=== RiftSensorSetup %s ===\n",install?"install":"uninstall");

	int exitCode=0;
	if(install)
		{
		stopOVRService();

		/* Register the DirectShow filter (required): */
		if(!registerFilter(true))
			exitCode=1;

		/* Bind WinUSB to every present sensor (camera may be unplugged;
		   that is not an install failure - re-run on plug-in): */
		std::vector<std::string> devices=findDevices("VID_2833&PID_0211&MI_00");
		logf("Found %u present Rift CV1 sensor video interface(s)\n",(unsigned int)devices.size());
		for(size_t i=0;i<devices.size();++i)
			{
			logf("Binding WinUSB to %s\n",devices[i].c_str());
			if(!bindWinUsb(devices[i]))
				{
				logf("Driver binding failed for %s\n",devices[i].c_str());
				exitCode=1;
				}
			}
		if(devices.empty())
			logf("No sensor present; plug it in and run 'RiftSensorSetup.exe install' again (elevated)\n");
		}
	else
		{
		registerFilter(false);
		std::vector<std::string> devices=findDevices("VID_2833&PID_0211&MI_00");
		for(size_t i=0;i<devices.size();++i)
			{
			logf("Reverting driver for %s\n",devices[i].c_str());
			revertDevice(devices[i]);
			}
		}

	logf("=== Done (exit %d) ===\n",exitCode);
	if(logFile!=0)
		fclose(logFile);
	return exitCode;
	}
