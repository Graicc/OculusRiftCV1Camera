/***********************************************************************
OculusRiftCV1CameraWin - Windows port of Oliver Kreylos' Oculus Rift CV1
tracking camera viewer. Talks to the camera's low-level USB interface
directly via libusb-1.0 (WinUSB or UsbDk backend), with no Vrui
dependency. Captures frames and writes them out as PNG files.

Based on OculusRiftCV1Camera.cpp, Copyright (c) 2018-2019 Oliver
Kreylos, distributed under the GNU General Public License version 2.
This port is likewise distributed under the GNU General Public License
version 2 or later.
***********************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <vector>
#include <atomic>

#include <libusb-1.0/libusb.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#ifdef _WIN32
#include <windows.h>
static void sleepMs(unsigned int ms) { Sleep(ms); }
#else
#include <unistd.h>
static void sleepMs(unsigned int ms) { usleep(ms*1000); }
#endif

/**************************************
Error-checking helpers for libusb calls:
**************************************/

static void throwUsbError(const char* what,int err)
	{
	char buf[256];
	snprintf(buf,sizeof(buf),"%s: %s (%d)",what,libusb_strerror((libusb_error)err),err);
	throw std::runtime_error(buf);
	}

/*******************************************************
UVC class commands on a USB device (control transfers):
*******************************************************/

class UVC
	{
	public:
	static void setCur(libusb_device_handle* device,uint8_t interface,uint8_t entity,uint8_t selector,const uint8_t* data,uint16_t length)
		{
		uint8_t requestType=LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
		int result=libusb_control_transfer(device,requestType,0x01 /* SET_CUR */,uint16_t(selector)<<8,(uint16_t(entity)<<8)|uint16_t(interface),const_cast<uint8_t*>(data),length,1000);
		if(result<0)
			throwUsbError("UVC::setCur",result);
		if(result!=int(length))
			throw std::runtime_error("UVC::setCur: Short write");
		}
	static void getCur(libusb_device_handle* device,uint8_t interface,uint8_t entity,uint8_t selector,uint8_t* data,uint16_t length)
		{
		uint8_t requestType=LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_IN;
		int result=libusb_control_transfer(device,requestType,0x81 /* GET_CUR */,uint16_t(selector)<<8,(uint16_t(entity)<<8)|uint16_t(interface),data,length,1000);
		if(result<0)
			throwUsbError("UVC::getCur",result);
		if(result!=int(length))
			throw std::runtime_error("UVC::getCur: Short read");
		}
	};

/*************************************************************
ESP770U camera controller (UVC extension unit 4 on interface 0):
*************************************************************/

class ESP770U
	{
	private:
	enum Entities { ExtensionUnit=4 };
	enum Selectors { I2C=2,Register=3,Counter=10,Control=11,Data=12 };

	libusb_device_handle* device;

	void setGetCur(int selector,uint8_t* buffer,uint16_t length)
		{
		UVC::setCur(device,0,ExtensionUnit,selector,buffer,length);
		UVC::getCur(device,0,ExtensionUnit,selector,buffer,length);
		}

	public:
	ESP770U(libusb_device_handle* sDevice)
		:device(sDevice)
		{
		}

	uint8_t readRegister(uint16_t registerIndex)
		{
		uint8_t command[4];
		command[0]=0x82U; // Opcode for read register
		command[1]=uint8_t(registerIndex>>8);
		command[2]=uint8_t(registerIndex&0xffU);
		command[3]=0x00U;
		setGetCur(Register,command,sizeof(command));
		if(command[0]!=0x82U||command[2]!=0x00U)
			throw std::runtime_error("ESP770U::readRegister: Invalid command buffer");
		return command[1];
		}

	void writeRegister(uint16_t registerIndex,uint8_t value)
		{
		uint8_t command[4];
		command[0]=0x02U; // Opcode for write register
		command[1]=uint8_t(registerIndex>>8);
		command[2]=uint8_t(registerIndex&0xffU);
		command[3]=value;
		setGetCur(Register,command,sizeof(command));
		if(command[0]!=0x02U||command[1]!=uint8_t(registerIndex>>8)||command[2]!=uint8_t(registerIndex&0xffU)||command[3]!=value)
			throw std::runtime_error("ESP770U::writeRegister: Invalid command buffer");
		}

	uint8_t getCounter(void)
		{
		uint8_t result;
		UVC::getCur(device,0,ExtensionUnit,Counter,&result,sizeof(result));
		return result;
		}

	void setCounter(uint8_t newCounter)
		{
		UVC::setCur(device,0,ExtensionUnit,Counter,&newCounter,sizeof(newCounter));
		}

	void spiSetControl(uint8_t handle,uint8_t length)
		{
		uint8_t command[16];
		memset(command,0,sizeof(command));
		command[0]=0x00U;
		command[1]=handle;
		command[2]=0x80U;
		command[3]=0x01U;
		command[9]=length;
		UVC::setCur(device,0,ExtensionUnit,Control,command,sizeof(command));
		}

	void spiSetData(const uint8_t* data,uint16_t length)
		{
		UVC::setCur(device,0,ExtensionUnit,Data,data,length);
		}

	void spiGetData(uint8_t* data,uint16_t length)
		{
		UVC::getCur(device,0,ExtensionUnit,Data,data,length);
		}

	void writeRadio(const uint8_t* data,uint16_t length)
		{
		if(length>126)
			throw std::runtime_error("ESP770U::writeRadio: Data block too large");

		/* Copy given data block into a fixed-size buffer and calculate a checksum in the last byte: */
		uint8_t buffer[127];
		memset(buffer,0,sizeof(buffer));
		for(uint16_t i=0;i<length;++i)
			{
			buffer[i]=data[i];
			buffer[126]-=buffer[i];
			}

		/* Write the buffer, read it back, write a cleared buffer, read that back: */
		spiSetControl(0x81,sizeof(buffer));
		spiSetData(buffer,sizeof(buffer));
		spiSetControl(0x41,sizeof(buffer));
		spiGetData(buffer,sizeof(buffer));
		memset(buffer,0,sizeof(buffer));
		spiSetControl(0x81,sizeof(buffer));
		spiSetData(buffer,sizeof(buffer));
		spiSetControl(0x41,sizeof(buffer));
		spiGetData(buffer,sizeof(buffer));

		/* Check the returned buffer's checksum and echoed command bytes: */
		uint8_t checkSum=0x00U;
		for(int i=0;i<127;++i)
			checkSum+=buffer[i];
		if(checkSum!=0x00U||buffer[0]!=data[0]||buffer[1]!=data[1])
			throw std::runtime_error("ESP770U::writeRadio: Invalid return buffer");
		}

	uint8_t queryFirmwareVersion(void)
		{
		uint8_t command[4];
		command[0]=0xa0U; // Opcode for get firmware version
		command[1]=0x03U;
		command[2]=0x00U;
		command[3]=0x00U;
		setGetCur(Register,command,sizeof(command));
		if(command[0]!=0xa0U||command[2]!=0x00U||command[3]!=0x00U)
			throw std::runtime_error("ESP770U::queryFirmwareVersion: Invalid command buffer");
		return command[1];
		}

	void readMemory(uint32_t address,uint8_t* buffer,uint16_t length)
		{
		uint8_t counter=getCounter();

		uint8_t command[16];
		memset(command,0,sizeof(command));
		command[0]=counter;
		command[1]=0x41U;
		command[2]=0x03;
		command[3]=0x01;
		command[5]=uint8_t((address>>16)&0xffU);
		command[6]=uint8_t((address>>8)&0xffU);
		command[7]=uint8_t(address&0xffU);
		command[8]=uint8_t(length>>8);
		command[9]=uint8_t(length&0xffU);
		UVC::setCur(device,0,ExtensionUnit,Control,command,sizeof(command));

		memset(buffer,0,length);
		UVC::getCur(device,0,ExtensionUnit,Data,buffer,length);

		setCounter(counter);
		}

	void initController(void)
		{
		uint8_t value=readRegister(0xf05aU);
		if(value!=0x01U&&value!=0x03U)
			printf("ESP770U::initController: Wrong value 0x%02x in register 0xf05a; continuing\n",value);
		writeRegister(0xf05aU,0x01U);

		value=readRegister(0xf018U);
		if(value!=0x0eU)
			printf("ESP770U::initController: Wrong value 0x%02x in register 0xf018; continuing\n",value);
		writeRegister(0xf018U,0x0fU);

		value=readRegister(0xf017U);
		if(value!=0xecU&&value!=0xedU)
			printf("ESP770U::initController: Wrong value 0x%02x in register 0xf017; continuing\n",value);
		writeRegister(0xf017U,value|0x01U);
		writeRegister(0xf017U,value&~0x01U);

		writeRegister(0xf018U,0x0eU);
		}

	void initRadio(void)
		{
		/* Wait for the radio to initialize: */
		sleepMs(50);

		uint8_t command0[2]={0x01U,0x01U};
		writeRadio(command0,sizeof(command0));

		uint8_t command1[2]={0x11U,0x01U};
		writeRadio(command1,sizeof(command1));

		uint8_t value=readRegister(0xf014U);
		if(value!=0x1aU&&value!=0x1bU)
			printf("ESP770U::initRadio: Wrong value 0x%02x in register 0xf014; continuing\n",value);

		uint8_t command2[2]={0x21U,0x01U};
		writeRadio(command2,sizeof(command2));

		uint8_t command3[3]={0x31U,0x01U};
		writeRadio(command3,sizeof(command3));
		}

	uint16_t readI2C(uint8_t address,uint16_t registerIndex)
		{
		uint8_t command[6];
		command[0]=0x86U;
		command[1]=address;
		command[2]=uint8_t(registerIndex>>8);
		command[3]=uint8_t(registerIndex&0xffU);
		command[4]=0x00U;
		command[5]=0x00U;
		setGetCur(I2C,command,sizeof(command));
		if(command[0]!=0x86U||command[4]!=0x00U||command[5]!=0x00U)
			throw std::runtime_error("ESP770U::readI2C: Invalid return buffer");
		return (uint16_t(command[2])<<8)|uint16_t(command[1]);
		}

	void writeI2C(uint8_t address,uint16_t registerIndex,uint16_t value)
		{
		uint8_t command[6];
		command[0]=0x06U;
		command[1]=address;
		command[2]=uint8_t(registerIndex>>8);
		command[3]=uint8_t(registerIndex&0xffU);
		command[4]=uint8_t(value>>8);
		command[5]=uint8_t(value&0xffU);
		setGetCur(I2C,command,sizeof(command));
		if(command[0]!=0x06U||command[1]!=address||command[2]!=uint8_t(registerIndex>>8)||command[3]!=uint8_t(registerIndex&0xffU)||command[4]!=uint8_t(value>>8)||command[5]!=uint8_t(value&0xffU))
			throw std::runtime_error("ESP770U::writeI2C: Invalid return buffer");
		}
	};

/*******************************************************
AR0134 imaging sensor (behind the ESP770U's I2C bus):
*******************************************************/

class AR0134
	{
	private:
	enum Addresses { I2CAddress=0x20 };

	enum Registers
		{
		ChipVersionReg=          0x3000,
		YAddrStart=              0x3002,
		XAddrStart=              0x3004,
		YAddrEnd=                0x3006,
		XAddrEnd=                0x3008,
		FrameLengthLines=        0x300a,
		LineLengthPck=           0x300c,
		RevisionNumber=          0x300e,
		CoarseIntegrationTime=   0x3012,
		FineIntegrationTime=     0x3014,
		ResetRegister=           0x301a,
		ReadMode=                0x3040,
		GlobalGain=              0x305e,
		EmbeddedDataControl=     0x3064,
		DigitalTest=             0x30b0,
		AeCtrlReg=               0x3100
		};

	enum ResetRegisterFlags
		{
		Stream=              0x0004,
		GpiEn=               0x0100,
		ForcedPllOn=         0x0800
		};

	enum ReadModeFlags
		{
		HorizMirror=0x4000,
		VertFlip=   0x8000
		};

	enum EmbeddedDataFlags
		{
		EmbeddedStatsEn=0x0080,
		EmbeddedData=0x0100
		};

	enum DigitalTestFlags
		{
		MonoChrome=       0x0080,
		EnableShortLlpck= 0x0400
		};

	enum AeCtrlRegFlags
		{
		AeEnable=0x0001,
		AutoAgEn=0x0002,
		AutoDgEn=0x0010
		};

	ESP770U& esp770u;

	uint16_t readRegister(uint16_t registerIndex)
		{
		return esp770u.readI2C(I2CAddress,registerIndex);
		}
	void writeRegister(uint16_t registerIndex,uint16_t value)
		{
		esp770u.writeI2C(I2CAddress,registerIndex,value);
		}

	public:
	AR0134(ESP770U& sEsp770u)
		:esp770u(sEsp770u)
		{
		}

	void init(void)
		{
		sleepMs(100);

		/* Read chip version and revision number: */
		unsigned int version=readRegister(ChipVersionReg);
		unsigned int revision=readRegister(RevisionNumber);
		if(version!=0x2406U||revision!=0x1300U)
			{
			char buf[128];
			snprintf(buf,sizeof(buf),"AR0134::init: Unsupported chip version 0x%04x.0x%04x",version,revision);
			throw std::runtime_error(buf);
			}

		/* Check the chip's digital test mode: */
		unsigned int testMode=readRegister(DigitalTest);
		if(testMode!=MonoChrome)
			throw std::runtime_error("AR0134: Unexpected camera mode");

		/* Enable embedded data and statistics: */
		uint16_t edc=readRegister(EmbeddedDataControl);
		writeRegister(EmbeddedDataControl,edc|EmbeddedStatsEn|EmbeddedData);
		}

	void setFlip(bool horizontalFlip,bool verticalFlip)
		{
		uint16_t readMode=readRegister(ReadMode);
		if(horizontalFlip)
			readMode|=HorizMirror;
		else
			readMode&=~HorizMirror;
		if(verticalFlip)
			readMode|=VertFlip;
		else
			readMode&=~VertFlip;
		writeRegister(ReadMode,readMode);
		}

	void setAutoExposure(bool enable,bool adjustAnalogGain,bool adjustDigitalGain)
		{
		uint16_t ae=readRegister(AeCtrlReg);
		ae=ae&~(AeEnable|AutoAgEn|AutoDgEn);
		if(enable)
			ae=ae|AeEnable;
		if(adjustAnalogGain)
			ae=ae|AutoAgEn;
		if(adjustDigitalGain)
			ae=ae|AutoDgEn;
		writeRegister(AeCtrlReg,ae);
		}

	void setGain(uint16_t gain)
		{
		writeRegister(GlobalGain,gain);
		}

	void setWindow(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
		{
		writeRegister(YAddrStart,y);
		writeRegister(XAddrStart,x);
		writeRegister(YAddrEnd,y+height-1);
		writeRegister(XAddrEnd,x+width-1);
		}

	void setFrameTimings(bool minBlank)
		{
		/* Set the capture window to the entire sensor: */
		setWindow(0,0,1280,960);

		/* Set tight or loose total line length: */
		writeRegister(LineLengthPck,1280+(minBlank?108:218));

		/* Enable or disable tight line timing in the digital test register: */
		uint16_t dt=readRegister(DigitalTest);
		if(minBlank)
			dt=dt|EnableShortLlpck;
		else
			dt=dt&(~EnableShortLlpck);
		writeRegister(DigitalTest,dt);

		/* Set tight or loose total frame height: */
		writeRegister(FrameLengthLines,960+(minBlank?23:37));
		}

	void setCoarseExposureTime(uint16_t coarseExposure)
		{
		writeRegister(CoarseIntegrationTime,coarseExposure);
		}

	void setFineExposureTime(uint16_t fineExposure)
		{
		writeRegister(FineIntegrationTime,fineExposure);
		}

	void setExposureTime(uint32_t exposureTime)
		{
		uint32_t frameWidth=readRegister(LineLengthPck);
		writeRegister(CoarseIntegrationTime,uint16_t(exposureTime/frameWidth));
		writeRegister(FineIntegrationTime,uint16_t(exposureTime%frameWidth));
		}

	void setSync(bool enable)
		{
		/* Enable or disable synchronized exposure mode via the reset register: */
		uint16_t r=readRegister(ResetRegister);
		r=r&~(Stream|GpiEn|ForcedPllOn);
		if(enable)
			r|=GpiEn|ForcedPllOn;
		else
			r|=Stream;
		writeRegister(ResetRegister,r);
		}
	};

/*******************************************************
Frame assembly from isochronous UVC payload packets:
*******************************************************/

class FrameAssembler
	{
	public:
	static const unsigned int width=1280;
	static const unsigned int height=960;

	std::vector<uint8_t> frameBuffer;
	uint8_t frameId;
	size_t framePos;
	bool synced; // Don't emit the first, potentially partial, frame
	std::atomic<int> framesDone;

	std::vector<std::vector<uint8_t> > capturedFrames;
	unsigned int framesWanted;

	FrameAssembler(unsigned int sFramesWanted)
		:frameBuffer(size_t(width)*size_t(height),0),
		 frameId(0xffU),framePos(0),synced(false),framesDone(0),
		 framesWanted(sFramesWanted)
		{
		}

	/* Process one isochronous packet's payload; returns true when capture is complete: */
	void processPacket(const uint8_t* data,size_t size)
		{
		/* Check for a valid UVC payload header (12 bytes, error bit clear): */
		if(size<12)
			return;
		size_t headerSize=data[0];
		if(headerSize!=12||(data[1]&0x40U)!=0x00U)
			return;

		/* Check if this packet belongs to a new frame: */
		if(frameId!=(data[1]&0x01U))
			{
			/* If we had a complete frame pending, store it: */
			if(synced&&framePos==frameBuffer.size()&&capturedFrames.size()<framesWanted)
				{
				capturedFrames.push_back(frameBuffer);
				framesDone=int(capturedFrames.size());
				}
			if(frameId!=0xffU)
				synced=true;
			frameId=data[1]&0x01U;
			framePos=0;
			}

		/* Copy payload into the frame: */
		size_t payload=size-headerSize;
		if(framePos+payload<=frameBuffer.size())
			{
			memcpy(&frameBuffer[framePos],data+headerSize,payload);
			framePos+=payload;
			}
		else
			{
			/* Overflow; mark frame as full and ignore the rest: */
			framePos=frameBuffer.size()+1;
			}
		}

	bool done(void) const
		{
		return capturedFrames.size()>=framesWanted;
		}
	};

/*******************************************************
Isochronous transfer management:
*******************************************************/

struct TransferContext
	{
	FrameAssembler* assembler;
	std::atomic<int>* activeTransfers;
	bool stopping;
	};

static void LIBUSB_CALL transferCallback(libusb_transfer* transfer)
	{
	TransferContext* context=static_cast<TransferContext*>(transfer->user_data);

	if(transfer->status==LIBUSB_TRANSFER_COMPLETED||transfer->status==LIBUSB_TRANSFER_ERROR)
		{
		/* Process all packets in the transfer: */
		for(int packetIndex=0;packetIndex<transfer->num_iso_packets;++packetIndex)
			{
			const libusb_iso_packet_descriptor& pd=transfer->iso_packet_desc[packetIndex];
			if(pd.status==LIBUSB_TRANSFER_COMPLETED&&pd.actual_length>0)
				{
				const uint8_t* packetData=libusb_get_iso_packet_buffer_simple(transfer,packetIndex);
				context->assembler->processPacket(packetData,pd.actual_length);
				}
			}
		}

	/* Resubmit unless we're done or shutting down: */
	if(!context->stopping&&!context->assembler->done())
		{
		int result=libusb_submit_transfer(transfer);
		if(result==LIBUSB_SUCCESS)
			return;
		printf("transferCallback: Resubmission failed: %s\n",libusb_strerror((libusb_error)result));
		}

	/* Transfer is retired: */
	--(*context->activeTransfers);
	}

/*******************************************************
Main program:
*******************************************************/

int main(int argc,char* argv[])
	{
	/* Parse command line: */
	bool useUsbDk=false;
	bool autoExposure=false;
	unsigned int gain=128;
	unsigned int exposure=800; // Coarse exposure time in lines
	unsigned int numFrames=5;
	const char* outputPrefix="frame";
	for(int i=1;i<argc;++i)
		{
		if(strcmp(argv[i],"--usbdk")==0)
			useUsbDk=true;
		else if(strcmp(argv[i],"-auto")==0)
			autoExposure=true;
		else if(strcmp(argv[i],"-gain")==0&&i+1<argc)
			gain=atoi(argv[++i]);
		else if(strcmp(argv[i],"-exposure")==0&&i+1<argc)
			exposure=atoi(argv[++i]);
		else if(strcmp(argv[i],"-frames")==0&&i+1<argc)
			numFrames=atoi(argv[++i]);
		else if(strcmp(argv[i],"-o")==0&&i+1<argc)
			outputPrefix=argv[++i];
		else
			{
			printf("Usage: %s [--usbdk] [-auto] [-gain <0-255>] [-exposure <lines>] [-frames <n>] [-o <prefix>]\n",argv[0]);
			return 1;
			}
		}

	libusb_context* usbContext=0;
	libusb_device_handle* device=0;
	int exitCode=0;

	try
		{
		/* Initialize libusb, optionally with the UsbDk backend: */
		int result;
		if(useUsbDk)
			{
			#if defined(_WIN32)&&defined(LIBUSB_API_VERSION)&&LIBUSB_API_VERSION>=0x01000106
			result=libusb_set_option(0,LIBUSB_OPTION_USE_USBDK);
			if(result!=LIBUSB_SUCCESS)
				throwUsbError("libusb_set_option(USE_USBDK)",result);
			printf("Using UsbDk backend\n");
			#else
			throw std::runtime_error("UsbDk backend not supported by this libusb");
			#endif
			}
		result=libusb_init(&usbContext);
		if(result!=LIBUSB_SUCCESS)
			throwUsbError("libusb_init",result);

		/* Find and open the first Oculus Rift CV1 camera (VID 0x2833, PID 0x0211): */
		device=libusb_open_device_with_vid_pid(usbContext,0x2833,0x0211);
		if(device==0)
			throw std::runtime_error("No Oculus Rift CV1 camera found (VID 2833, PID 0211), or access denied. Is the driver set up (WinUSB via Zadig, or UsbDk with --usbdk)?");

		libusb_device* dev=libusb_get_device(device);
		int speed=libusb_get_device_speed(dev);
		const char* speedNames[]={"unknown","low","full","high","super","super+"};
		printf("Device opened; USB speed: %s\n",speed>=0&&speed<=5?speedNames[speed]:"?");

		/* Claim the UVC control interface, detaching any kernel drivers: */
		libusb_set_auto_detach_kernel_driver(device,1);
		result=libusb_claim_interface(device,0);
		if(result!=LIBUSB_SUCCESS)
			throwUsbError("claim interface 0",result);

		/* Query the camera controller's firmware version: */
		ESP770U controller(device);
		unsigned int firmwareVersion=controller.queryFirmwareVersion();
		printf("ESP770U camera controller's firmware version: %u\n",firmwareVersion);

		/* Initialize the camera controller: */
		controller.initController();

		/* Initialize the camera controller's radio component (not required for video; ignore failures): */
		try
			{
			controller.initRadio();
			}
		catch(const std::runtime_error& err)
			{
			printf("Radio init failed (%s); continuing, video does not need it\n",err.what());
			}

		/* Retrieve the camera's calibration data from the controller's non-volatile memory: */
		uint8_t calData[128];
		controller.readMemory(0x1d000U,calData,sizeof(calData));
		float fx,cx,cy;
		memcpy(&fx,calData+0x30,4);
		memcpy(&cx,calData+0x34,4);
		memcpy(&cy,calData+0x38,4);
		printf("Calibration: focal length %f, center %f, %f\n",fx,cx,cy);

		/* Set up the camera's imaging sensor: */
		AR0134 sensor(controller);
		sensor.init();
		sensor.setFrameTimings(true);
		sensor.setGain(gain);
		sensor.setCoarseExposureTime(400);
		sensor.setFineExposureTime(15);
		sensor.setSync(false);

		/* Claim the UVC data interface: */
		result=libusb_claim_interface(device,1);
		if(result!=LIBUSB_SUCCESS)
			throwUsbError("claim interface 1",result);

		/* Assemble a UVC probe control request (26 bytes, little-endian): */
		uint8_t probe[26];
		memset(probe,0,sizeof(probe));
		probe[0]=0x00U; probe[1]=0x00U; // bmHint
		probe[2]=1; // bFormatIndex
		probe[3]=4; // bFrameIndex
		uint32_t frameInterval=192000U; // dwFrameInterval in 100ns units
		memcpy(probe+4,&frameInterval,4);
		uint32_t maxVideoFrameSize=1280U*960U;
		memcpy(probe+18,&maxVideoFrameSize,4);
		uint32_t maxPayloadTransferSize=3072U;
		memcpy(probe+22,&maxPayloadTransferSize,4);

		/* Probe the control (VS_PROBE_CONTROL=1 on interface 1) and read the result: */
		UVC::setCur(device,1,0,1,probe,sizeof(probe));
		uint8_t probeResult[26];
		UVC::getCur(device,1,0,1,probeResult,sizeof(probeResult));

		uint32_t rFrameInterval,rMaxFrameSize,rMaxPayload;
		memcpy(&rFrameInterval,probeResult+4,4);
		memcpy(&rMaxFrameSize,probeResult+18,4);
		memcpy(&rMaxPayload,probeResult+22,4);
		printf("UVC probe result: format %u, frame %u, interval %u, maxFrameSize %u, maxPayload %u\n",probeResult[2],probeResult[3],rFrameInterval,rMaxFrameSize,rMaxPayload);

		/* Commit the probe result (VS_COMMIT_CONTROL=2): */
		UVC::setCur(device,1,0,2,probeResult,sizeof(probeResult));

		/* Request the camera device's alternate setting for video streaming: */
		result=libusb_set_interface_alt_setting(device,1,2);
		if(result!=LIBUSB_SUCCESS)
			throwUsbError("set alt setting 1/2",result);

		/* Set up isochronous transfers on endpoint 0x81: */
		FrameAssembler assembler(numFrames);
		std::atomic<int> activeTransfers(0);
		TransferContext context;
		context.assembler=&assembler;
		context.activeTransfers=&activeTransfers;
		context.stopping=false;

		const int numTransfers=7;
		const int packetsPerTransfer=24;
		const int packetSize=16384;
		std::vector<libusb_transfer*> transfers;
		std::vector<std::vector<uint8_t> > transferBuffers;
		for(int i=0;i<numTransfers;++i)
			{
			libusb_transfer* transfer=libusb_alloc_transfer(packetsPerTransfer);
			if(transfer==0)
				throw std::runtime_error("Cannot allocate USB transfer");
			transferBuffers.push_back(std::vector<uint8_t>(size_t(packetsPerTransfer)*size_t(packetSize)));
			libusb_fill_iso_transfer(transfer,device,0x81,&transferBuffers.back()[0],packetsPerTransfer*packetSize,packetsPerTransfer,transferCallback,&context,1000);
			libusb_set_iso_packet_lengths(transfer,packetSize);
			transfers.push_back(transfer);
			}
		for(int i=0;i<numTransfers;++i)
			{
			result=libusb_submit_transfer(transfers[i]);
			if(result!=LIBUSB_SUCCESS)
				throwUsbError("submit iso transfer",result);
			++activeTransfers;
			}
		printf("Streaming started (%d transfers x %d packets x %d bytes)\n",numTransfers,packetsPerTransfer,packetSize);

		/* After a second of streaming, set the real exposure and gain like the original does: */
		bool exposureSet=false;
		int iterations=0;
		while(!assembler.done()&&activeTransfers>0)
			{
			timeval tv;
			tv.tv_sec=0;
			tv.tv_usec=100000;
			libusb_handle_events_timeout(usbContext,&tv);
			++iterations;
			if(!exposureSet&&iterations>=10)
				{
				if(autoExposure)
					sensor.setAutoExposure(true,true,true);
				else
					{
					sensor.setCoarseExposureTime(exposure);
					sensor.setFineExposureTime(0);
					sensor.setGain(gain);
					}
				exposureSet=true;
				printf("Exposure/gain configured (auto=%d, exposure=%u, gain=%u); capturing...\n",int(autoExposure),exposure,gain);
				}
			if(iterations>300) // 30 second timeout
				{
				printf("Timeout waiting for frames; captured %zu so far\n",assembler.capturedFrames.size());
				break;
				}
			}

		/* Shut down streaming: */
		context.stopping=true;
		for(int i=0;i<numTransfers;++i)
			libusb_cancel_transfer(transfers[i]);
		while(activeTransfers>0)
			{
			timeval tv;
			tv.tv_sec=0;
			tv.tv_usec=100000;
			libusb_handle_events_timeout(usbContext,&tv);
			}
		for(int i=0;i<numTransfers;++i)
			libusb_free_transfer(transfers[i]);

		libusb_set_interface_alt_setting(device,1,0);

		/* Save captured frames as PNG files: */
		printf("Captured %zu frames\n",assembler.capturedFrames.size());
		for(size_t i=0;i<assembler.capturedFrames.size();++i)
			{
			/* Calculate mean brightness for diagnostics: */
			const std::vector<uint8_t>& frame=assembler.capturedFrames[i];
			uint64_t sum=0;
			for(size_t j=0;j<frame.size();++j)
				sum+=frame[j];
			double mean=double(sum)/double(frame.size());

			char fileName[1024];
			snprintf(fileName,sizeof(fileName),"%s%02zu.png",outputPrefix,i);
			if(stbi_write_png(fileName,FrameAssembler::width,FrameAssembler::height,1,&frame[0],FrameAssembler::width)==0)
				printf("Failed to write %s\n",fileName);
			else
				printf("Wrote %s (mean brightness %.1f)\n",fileName,mean);
			}
		if(assembler.capturedFrames.empty())
			{
			printf("ERROR: No frames captured\n");
			exitCode=2;
			}

		libusb_release_interface(device,1);
		libusb_release_interface(device,0);
		}
	catch(const std::exception& err)
		{
		printf("ERROR: %s\n",err.what());
		exitCode=1;
		}

	if(device!=0)
		libusb_close(device);
	if(usbContext!=0)
		libusb_exit(usbContext);
	return exitCode;
	}
