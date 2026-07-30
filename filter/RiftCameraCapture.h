/***********************************************************************
RiftCameraCapture - Self-contained capture engine for the Oculus Rift
CV1 tracking camera. Opens the camera via libusb-1.0 (WinUSB), runs the
ESP770U/AR0134 initialization, streams isochronous video, reassembles
frames, applies software auto-exposure, and publishes the latest frame
as a bottom-up RGBA image for consumption by a DirectShow filter.

Camera protocol based on OculusRiftCV1Camera.cpp, Copyright (c)
2018-2019 Oliver Kreylos, GPL v2+.
***********************************************************************/

#ifndef RIFTCAMERACAPTURE_H
#define RIFTCAMERACAPTURE_H

#include <winsock2.h> // For struct timeval, used by libusb's event API
#include <windows.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <atomic>

#include <libusb-1.0/libusb.h>

namespace RiftCam {

/* UVC class requests on the camera's control/streaming interfaces: */
class UVC
	{
	public:
	static void setCur(libusb_device_handle* device,uint8_t interface,uint8_t entity,uint8_t selector,const uint8_t* data,uint16_t length)
		{
		uint8_t requestType=uint8_t(LIBUSB_REQUEST_TYPE_CLASS)|uint8_t(LIBUSB_RECIPIENT_INTERFACE)|uint8_t(LIBUSB_ENDPOINT_OUT);
		int result=libusb_control_transfer(device,requestType,0x01,uint16_t(selector)<<8,(uint16_t(entity)<<8)|uint16_t(interface),const_cast<uint8_t*>(data),length,1000);
		if(result!=int(length))
			throw std::runtime_error("UVC::setCur failed");
		}
	static void getCur(libusb_device_handle* device,uint8_t interface,uint8_t entity,uint8_t selector,uint8_t* data,uint16_t length)
		{
		uint8_t requestType=uint8_t(LIBUSB_REQUEST_TYPE_CLASS)|uint8_t(LIBUSB_RECIPIENT_INTERFACE)|uint8_t(LIBUSB_ENDPOINT_IN);
		int result=libusb_control_transfer(device,requestType,0x81,uint16_t(selector)<<8,(uint16_t(entity)<<8)|uint16_t(interface),data,length,1000);
		if(result!=int(length))
			throw std::runtime_error("UVC::getCur failed");
		}
	};

/* ESP770U camera controller (UVC extension unit 4): */
class ESP770U
	{
	private:
	enum Entities { ExtensionUnit=4 };
	enum Selectors { I2C=2,Register=3,Counter=10,Control=11,Data=12 };
	libusb_device_handle* device;

	void setGetCur(int selector,uint8_t* buffer,uint16_t length)
		{
		UVC::setCur(device,0,ExtensionUnit,(uint8_t)selector,buffer,length);
		UVC::getCur(device,0,ExtensionUnit,(uint8_t)selector,buffer,length);
		}

	public:
	ESP770U(libusb_device_handle* sDevice)
		:device(sDevice)
		{
		}

	uint8_t readRegister(uint16_t registerIndex)
		{
		uint8_t command[4]={0x82U,uint8_t(registerIndex>>8),uint8_t(registerIndex&0xffU),0x00U};
		setGetCur(Register,command,sizeof(command));
		if(command[0]!=0x82U||command[2]!=0x00U)
			throw std::runtime_error("ESP770U::readRegister: Invalid command buffer");
		return command[1];
		}

	void writeRegister(uint16_t registerIndex,uint8_t value)
		{
		uint8_t command[4]={0x02U,uint8_t(registerIndex>>8),uint8_t(registerIndex&0xffU),value};
		setGetCur(Register,command,sizeof(command));
		if(command[0]!=0x02U)
			throw std::runtime_error("ESP770U::writeRegister: Invalid command buffer");
		}

	void initController(void)
		{
		uint8_t value=readRegister(0xf05aU);
		writeRegister(0xf05aU,0x01U);
		value=readRegister(0xf018U);
		writeRegister(0xf018U,0x0fU);
		value=readRegister(0xf017U);
		writeRegister(0xf017U,value|0x01U);
		writeRegister(0xf017U,value&~0x01U);
		writeRegister(0xf018U,0x0eU);
		}

	uint16_t readI2C(uint8_t address,uint16_t registerIndex)
		{
		uint8_t command[6]={0x86U,address,uint8_t(registerIndex>>8),uint8_t(registerIndex&0xffU),0x00U,0x00U};
		setGetCur(I2C,command,sizeof(command));
		if(command[0]!=0x86U||command[4]!=0x00U||command[5]!=0x00U)
			throw std::runtime_error("ESP770U::readI2C: Invalid return buffer");
		return (uint16_t(command[2])<<8)|uint16_t(command[1]);
		}

	void writeI2C(uint8_t address,uint16_t registerIndex,uint16_t value)
		{
		uint8_t command[6]={0x06U,address,uint8_t(registerIndex>>8),uint8_t(registerIndex&0xffU),uint8_t(value>>8),uint8_t(value&0xffU)};
		setGetCur(I2C,command,sizeof(command));
		if(command[0]!=0x06U||command[1]!=address)
			throw std::runtime_error("ESP770U::writeI2C: Invalid return buffer");
		}
	};

/* AR0134 imaging sensor behind the controller's I2C bus: */
class AR0134
	{
	private:
	enum Addresses { I2CAddress=0x20 };
	enum Registers
		{
		ChipVersionReg=0x3000,YAddrStart=0x3002,XAddrStart=0x3004,YAddrEnd=0x3006,XAddrEnd=0x3008,
		FrameLengthLines=0x300a,LineLengthPck=0x300c,RevisionNumber=0x300e,
		CoarseIntegrationTime=0x3012,FineIntegrationTime=0x3014,ResetRegister=0x301a,
		GlobalGain=0x305e,EmbeddedDataControl=0x3064,DigitalTest=0x30b0,AeCtrlReg=0x3100
		};
	enum ResetRegisterFlags { Stream=0x0004,GpiEn=0x0100,ForcedPllOn=0x0800 };
	enum EmbeddedDataFlags { EmbeddedStatsEn=0x0080,EmbeddedData=0x0100 };
	enum DigitalTestFlags { MonoChrome=0x0080,EnableShortLlpck=0x0400 };

	ESP770U& esp770u;

	uint16_t readRegister(uint16_t r) { return esp770u.readI2C(I2CAddress,r); }
	void writeRegister(uint16_t r,uint16_t v) { esp770u.writeI2C(I2CAddress,r,v); }

	public:
	AR0134(ESP770U& sEsp770u)
		:esp770u(sEsp770u)
		{
		}

	void init(void)
		{
		Sleep(100);
		unsigned int version=readRegister(ChipVersionReg);
		unsigned int revision=readRegister(RevisionNumber);
		if(version!=0x2406U||revision!=0x1300U)
			throw std::runtime_error("AR0134::init: Unsupported chip version");
		if(readRegister(DigitalTest)!=MonoChrome)
			throw std::runtime_error("AR0134: Unexpected camera mode");
		uint16_t edc=readRegister(EmbeddedDataControl);
		writeRegister(EmbeddedDataControl,edc|EmbeddedStatsEn|EmbeddedData);
		}

	void setGain(uint16_t gain) { writeRegister(GlobalGain,gain); }

	void setWindow(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
		{
		writeRegister(YAddrStart,y);
		writeRegister(XAddrStart,x);
		writeRegister(YAddrEnd,y+height-1);
		writeRegister(XAddrEnd,x+width-1);
		}

	void setFrameTimings(bool minBlank)
		{
		setWindow(0,0,1280,960);
		writeRegister(LineLengthPck,1280+(minBlank?108:218));
		uint16_t dt=readRegister(DigitalTest);
		if(minBlank)
			dt=dt|EnableShortLlpck;
		else
			dt=dt&(~EnableShortLlpck);
		writeRegister(DigitalTest,dt);
		writeRegister(FrameLengthLines,960+(minBlank?23:37));
		}

	void setCoarseExposureTime(uint16_t e) { writeRegister(CoarseIntegrationTime,e); }
	void setFineExposureTime(uint16_t e) { writeRegister(FineIntegrationTime,e); }

	void setSync(bool enable)
		{
		uint16_t r=readRegister(ResetRegister);
		r=r&~(Stream|GpiEn|ForcedPllOn);
		if(enable)
			r|=GpiEn|ForcedPllOn;
		else
			r|=Stream;
		writeRegister(ResetRegister,r);
		}
	};

/* The capture engine. Runs a worker thread that owns the USB device and
   publishes bottom-up RGBA frames: */
class Capture
	{
	public:
	static const unsigned int camWidth=1280;
	static const unsigned int camHeight=960;

	private:
	/* Configuration (read from HKLM\SOFTWARE\RiftSensorCam at start): */
	unsigned int aeTarget; // Target mean brightness for software auto-exposure
	int rotate; // 0, 90, 180, or 270 degrees clockwise

	/* Worker thread and lifecycle: */
	HANDLE thread;
	std::atomic<bool> shuttingDown;

	/* Latest published frame (bottom-up RGBA, outWidth x outHeight). The lock
	   is only ever held for a buffer swap or memcpy, never during conversion,
	   so the USB event loop cannot stall on it: */
	CRITICAL_SECTION frameLock;
	HANDLE newFrameEvent; // Auto-reset, signaled per published frame
	std::vector<uint8_t> publishedFrame;
	std::vector<uint8_t> scratchFrame; // Conversion target (worker thread only)
	std::vector<uint8_t> receiveStaging; // Copy-out target (receiver only)
	unsigned int outWidth,outHeight;
	std::atomic<bool> cameraRunning; // True while the camera is streaming

	/* Reassembly state (worker thread only): */
	std::vector<uint8_t> frameBuffer;
	uint8_t frameId;
	size_t framePos;
	bool synced;
	uint64_t completeFrames;

	/* USB state (worker thread only): */
	libusb_context* usbContext;
	libusb_device_handle* device;

	static DWORD WINAPI threadEntry(LPVOID param)
		{
		static_cast<Capture*>(param)->threadMethod();
		return 0;
		}

	void readConfig(void)
		{
		aeTarget=80;
		rotate=0;
		HKEY key;
		if(RegOpenKeyExW(HKEY_LOCAL_MACHINE,L"SOFTWARE\\RiftSensorCam",0,KEY_READ,&key)==ERROR_SUCCESS)
			{
			DWORD value,size=sizeof(value);
			if(RegQueryValueExW(key,L"TargetBrightness",0,0,(LPBYTE)&value,&size)==ERROR_SUCCESS&&value>=16&&value<=250)
				aeTarget=value;
			size=sizeof(value);
			if(RegQueryValueExW(key,L"Rotate",0,0,(LPBYTE)&value,&size)==ERROR_SUCCESS&&(value==0||value==90||value==180||value==270))
				rotate=int(value);
			RegCloseKey(key);
			}
		outWidth=(rotate==90||rotate==270)?camHeight:camWidth;
		outHeight=(rotate==90||rotate==270)?camWidth:camHeight;
		}

	/* Convert the reassembled grayscale frame to bottom-up RGBA and publish it: */
	void publishFrame(void)
		{
		/* Convert into the scratch buffer without holding any lock: */
		const uint8_t* src=&frameBuffer[0];
		for(unsigned int y=0;y<outHeight;++y)
			{
			uint8_t* dst=&scratchFrame[size_t(outHeight-1-y)*outWidth*4];
			for(unsigned int x=0;x<outWidth;++x)
				{
				unsigned int sx,sy;
				switch(rotate)
					{
					case 90:  sx=y;            sy=camHeight-1-x; break;
					case 180: sx=camWidth-1-x; sy=camHeight-1-y; break;
					case 270: sx=camWidth-1-y; sy=x;             break;
					default:  sx=x;            sy=y;             break;
					}
				uint8_t v=src[size_t(sy)*camWidth+sx];
				*dst++=v;
				*dst++=v;
				*dst++=v;
				*dst++=0xffU;
				}
			}

		/* Publish with a constant-time swap: */
		EnterCriticalSection(&frameLock);
		publishedFrame.swap(scratchFrame);
		LeaveCriticalSection(&frameLock);
		++completeFrames;
		SetEvent(newFrameEvent);
		}

	void processPacket(const uint8_t* data,size_t size)
		{
		if(size<12)
			return;
		size_t headerSize=data[0];
		if(headerSize!=12||(data[1]&0x40U)!=0x00U)
			return;

		if(frameId!=(data[1]&0x01U))
			{
			if(synced&&framePos==frameBuffer.size())
				publishFrame();
			if(frameId!=0xffU)
				synced=true;
			frameId=data[1]&0x01U;
			framePos=0;
			}

		size_t payload=size-headerSize;
		if(framePos+payload<=frameBuffer.size())
			{
			memcpy(&frameBuffer[framePos],data+headerSize,payload);
			framePos+=payload;
			}
		else
			framePos=frameBuffer.size()+1;
		}

	struct TransferContext
		{
		Capture* capture;
		std::atomic<int>* activeTransfers;
		bool stopping;
		};

	static void LIBUSB_CALL transferCallback(libusb_transfer* transfer)
		{
		TransferContext* context=static_cast<TransferContext*>(transfer->user_data);
		if(transfer->status==LIBUSB_TRANSFER_COMPLETED||transfer->status==LIBUSB_TRANSFER_ERROR)
			{
			for(int packetIndex=0;packetIndex<transfer->num_iso_packets;++packetIndex)
				{
				const libusb_iso_packet_descriptor& pd=transfer->iso_packet_desc[packetIndex];
				if(pd.status==LIBUSB_TRANSFER_COMPLETED&&pd.actual_length>0)
					{
					const uint8_t* packetData=libusb_get_iso_packet_buffer_simple(transfer,packetIndex);
					context->capture->processPacket(packetData,pd.actual_length);
					}
				}
			}
		if(!context->stopping&&!context->capture->shuttingDown)
			{
			if(libusb_submit_transfer(transfer)==LIBUSB_SUCCESS)
				return;
			}
		--(*context->activeTransfers);
		}

	/* One full open->stream->close session; returns when shutting down or on error: */
	void runCameraSession(void)
		{
		device=libusb_open_device_with_vid_pid(usbContext,0x2833,0x0211);
		if(device==0)
			throw std::runtime_error("Camera not found or not accessible");

		libusb_claim_interface(device,0);

		ESP770U controller(device);
		controller.initController();
		/* Note: the radio init from the original code is skipped; it is only
		   needed to sync exposure to a Rift headset's IR LEDs. */

		AR0134 sensor(controller);
		sensor.init();
		sensor.setFrameTimings(true);
		sensor.setGain(128);
		sensor.setCoarseExposureTime(400);
		sensor.setFineExposureTime(15);
		sensor.setSync(false);

		int result=libusb_claim_interface(device,1);
		if(result!=LIBUSB_SUCCESS)
			throw std::runtime_error("Cannot claim streaming interface");

		/* UVC probe/commit: */
		uint8_t probe[26];
		memset(probe,0,sizeof(probe));
		probe[2]=1; // bFormatIndex
		probe[3]=4; // bFrameIndex
		uint32_t frameInterval=192000U;
		memcpy(probe+4,&frameInterval,4);
		uint32_t maxVideoFrameSize=camWidth*camHeight;
		memcpy(probe+18,&maxVideoFrameSize,4);
		uint32_t maxPayloadTransferSize=3072U;
		memcpy(probe+22,&maxPayloadTransferSize,4);
		UVC::setCur(device,1,0,1,probe,sizeof(probe));
		uint8_t probeResult[26];
		UVC::getCur(device,1,0,1,probeResult,sizeof(probeResult));
		UVC::setCur(device,1,0,2,probeResult,sizeof(probeResult));

		result=libusb_set_interface_alt_setting(device,1,2);
		if(result!=LIBUSB_SUCCESS)
			throw std::runtime_error("Cannot set streaming alt setting");

		/* Reset reassembly state: */
		frameId=0xffU;
		framePos=0;
		synced=false;

		/* Set up isochronous transfers: */
		std::atomic<int> activeTransfers(0);
		TransferContext context;
		context.capture=this;
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
			if(result==LIBUSB_SUCCESS)
				++activeTransfers;
			}
		if(activeTransfers==0)
			throw std::runtime_error("Cannot submit USB transfers");

		cameraRunning=true;

		/* Event/AE/watchdog loop: */
		bool exposureSet=false;
		int iterations=0;
		unsigned int currentExposure=800;
		unsigned int currentGain=128;
		const unsigned int minExposure=20,maxExposure=3000;
		const unsigned int minGain=64,maxGain=240;
		uint64_t lastCompleteFrames=completeFrames;
		ULONGLONG lastAeTime=GetTickCount64();
		ULONGLONG lastProgressTime=GetTickCount64();
		while(!shuttingDown&&activeTransfers>0)
			{
			timeval tv;
			tv.tv_sec=0;
			tv.tv_usec=100000;
			libusb_handle_events_timeout(usbContext,&tv);
			++iterations;
			if(!exposureSet&&iterations>=10)
				{
				sensor.setCoarseExposureTime((uint16_t)currentExposure);
				sensor.setFineExposureTime(0);
				sensor.setGain((uint16_t)currentGain);
				exposureSet=true;
				}

			ULONGLONG now=GetTickCount64();

			/* Software auto-exposure toward the target mean brightness: */
			if(exposureSet&&now-lastAeTime>=500)
				{
				lastAeTime=now;

				/* Sample the current reassembly buffer (whatever is freshest): */
				uint64_t sum=0;
				for(size_t i=0;i<frameBuffer.size();i+=64)
					sum+=frameBuffer[i];
				double mean=double(sum*64)/double(frameBuffer.size());
				double ratio=double(aeTarget)/(mean>1.0?mean:1.0);
				if(ratio<0.87||ratio>1.15)
					{
					double correction=pow(ratio,0.7);
					double newExposure=double(currentExposure)*correction;
					if(newExposure>double(maxExposure)&&currentGain<maxGain)
						{
						currentGain=(unsigned int)(double(currentGain)*correction+0.5);
						if(currentGain>maxGain)
							currentGain=maxGain;
						sensor.setGain((uint16_t)currentGain);
						}
					else if(newExposure<double(minExposure)&&currentGain>minGain)
						{
						currentGain=(unsigned int)(double(currentGain)*correction+0.5);
						if(currentGain<minGain)
							currentGain=minGain;
						sensor.setGain((uint16_t)currentGain);
						}
					else
						{
						if(newExposure>double(maxExposure))
							newExposure=double(maxExposure);
						if(newExposure<double(minExposure))
							newExposure=double(minExposure);
						currentExposure=(unsigned int)(newExposure+0.5);
						sensor.setCoarseExposureTime((uint16_t)currentExposure);
						}
					}
				}

			/* Watchdog: bail out of the session if no frame completed for 5s;
			   the outer loop will tear down and reopen the camera: */
			if(completeFrames!=lastCompleteFrames)
				{
				lastCompleteFrames=completeFrames;
				lastProgressTime=now;
				}
			else if(exposureSet&&now-lastProgressTime>=5000)
				break;
			}

		cameraRunning=false;

		/* Tear down streaming: */
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
		libusb_release_interface(device,1);
		libusb_release_interface(device,0);
		libusb_close(device);
		device=0;
		}

	void threadMethod(void)
		{
		if(libusb_init(&usbContext)!=LIBUSB_SUCCESS)
			return;

		/* Keep (re)opening the camera until shutdown; retry after failures: */
		while(!shuttingDown)
			{
			try
				{
				runCameraSession();
				}
			catch(...)
				{
				if(device!=0)
					{
					libusb_close(device);
					device=0;
					}
				cameraRunning=false;
				}

			/* Wait before reconnecting (camera may be unplugged or busy): */
			for(int i=0;i<30&&!shuttingDown;++i)
				Sleep(100);
			}

		libusb_exit(usbContext);
		usbContext=0;
		}

	public:
	Capture(void)
		:thread(0),shuttingDown(false),
		 outWidth(camWidth),outHeight(camHeight),
		 cameraRunning(false),
		 frameBuffer(size_t(camWidth)*size_t(camHeight),0),
		 frameId(0xffU),framePos(0),synced(false),completeFrames(0),
		 usbContext(0),device(0)
		{
		InitializeCriticalSection(&frameLock);
		newFrameEvent=CreateEventW(0,FALSE,FALSE,0);
		readConfig();
		publishedFrame.resize(size_t(outWidth)*size_t(outHeight)*4,0);
		scratchFrame.resize(size_t(outWidth)*size_t(outHeight)*4,0);
		receiveStaging.resize(size_t(outWidth)*size_t(outHeight)*4,0);
		}

	~Capture(void)
		{
		shuttingDown=true;
		if(thread!=0)
			{
			WaitForSingleObject(thread,10000);
			CloseHandle(thread);
			}
		CloseHandle(newFrameEvent);
		DeleteCriticalSection(&frameLock);
		}

	/* Starts the capture thread (idempotent): */
	void start(void)
		{
		if(thread==0)
			thread=CreateThread(0,0,threadEntry,this,0,0);
		}

	bool isRunning(void) const
		{
		return cameraRunning;
		}

	unsigned int getWidth(void) const { return outWidth; }
	unsigned int getHeight(void) const { return outHeight; }

	/* Waits up to timeoutMs for a fresh frame; returns true if one arrived.
	   Either way, hands the latest published frame to the callback. The frame
	   lock is only held for a memcpy; the callback runs on a private copy: */
	template <class CallbackType>
	bool receive(unsigned int timeoutMs,CallbackType callback)
		{
		bool fresh=(WaitForSingleObject(newFrameEvent,timeoutMs)==WAIT_OBJECT_0);
		EnterCriticalSection(&frameLock);
		memcpy(&receiveStaging[0],&publishedFrame[0],receiveStaging.size());
		LeaveCriticalSection(&frameLock);
		callback(outWidth,outHeight,&receiveStaging[0]);
		return fresh;
		}
	};

}

#endif
