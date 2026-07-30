/* Standalone test host for RiftCameraCapture.h - starts the capture engine
   exactly as the DirectShow filter would and writes received frames as PNG. */
#include "RiftCameraCapture.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"

int main(int argc,char* argv[])
	{
	int seconds=argc>1?atoi(argv[1]):8;

	RiftCam::Capture capture;
	capture.start();
	printf("Capture engine started; running for %d seconds...\n",seconds);

	std::vector<uint8_t> lastFrame;
	unsigned int lastW=0,lastH=0;
	int freshCount=0,staleCount=0;
	ULONGLONG start=GetTickCount64();
	while(GetTickCount64()-start<ULONGLONG(seconds)*1000)
		{
		if(!capture.isRunning())
			{
			Sleep(100);
			continue;
			}
		bool fresh=capture.receive(200,[&](unsigned int w,unsigned int h,uint8_t* buf)
			{
			lastW=w;
			lastH=h;
			lastFrame.assign(buf,buf+size_t(w)*size_t(h)*4);
			});
		if(fresh)
			++freshCount;
		else
			++staleCount;
		}

	printf("Fresh frames: %d, stale polls: %d, running: %d\n",freshCount,staleCount,int(capture.isRunning()));
	if(lastFrame.empty())
		{
		printf("ERROR: No frame received\n");
		return 1;
		}

	/* The published frame is bottom-up RGBA; flip it for the PNG: */
	std::vector<uint8_t> topDown(lastFrame.size());
	for(unsigned int y=0;y<lastH;++y)
		memcpy(&topDown[size_t(y)*lastW*4],&lastFrame[size_t(lastH-1-y)*lastW*4],size_t(lastW)*4);
	uint64_t sum=0;
	for(size_t i=0;i<topDown.size();i+=4)
		sum+=topDown[i];
	printf("Mean brightness: %.1f\n",double(sum)/double(topDown.size()/4));
	if(stbi_write_png("captest.png",lastW,lastH,4,&topDown[0],lastW*4)==0)
		{
		printf("ERROR: Cannot write PNG\n");
		return 1;
		}
	printf("Wrote captest.png (%ux%u)\n",lastW,lastH);
	return 0;
	}
