#ifndef SIMPLE_OPENGL3_APP_H
#define SIMPLE_OPENGL3_APP_H

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "../OpenGLWindow/GLPrimitiveRenderer.h"
#include "../CommonInterfaces/CommonWindowInterface.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"


struct SimpleOpenGL3App : public CommonGraphicsApp
{
	struct SimpleInternalData* m_data;

	class GLPrimitiveRenderer*	m_primRenderer;
	class GLInstancingRenderer* m_instancingRenderer;
	

	SimpleOpenGL3App(const char* title, int width,int height);
	virtual ~SimpleOpenGL3App();

	virtual int	registerCubeShape(float halfExtentsX=1.f,float halfExtentsY=1.f, float halfExtentsZ = 1.f);
	virtual int	registerGraphicsSphereShape(float radius, bool usePointSprites=true, int largeSphereThreshold=100, int mediumSphereThreshold=10);
	virtual void registerGrid(int xres, int yres, float color0[4], float color1[4]);
    void dumpNextFrameToPng(const char* pngFilename);
    void dumpFramesToVideo(const char* mp4Filename);
    
	void drawGrid(DrawGridData data=DrawGridData());
	virtual void setUpAxis(int axis);
	virtual int getUpAxis() const;
	
	virtual void swapBuffer();
	virtual void drawText( const char* txt, int posX, int posY);
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size, float colorR = 1.0f, float colorG = 0.2f, float colorB = 0.2f, float colorAlpha = 1.0f);
	virtual void drawLine3D(float startX, float startY, float startZ, float endX, float endY, float endZ, float colorR = 1.0f, float colorG = 0.2f, float colorB = 0.2f, float colorAlpha = 1.0f, float width = 1.0f);
	virtual void drawPoint3D(float x, float y, float z, float colorR = 1.0f, float colorG = 0.2f, float colorB = 0.2f, float colorAlpha = 1.0f, float size = 1.0f);
	struct sth_stash* getFontStash();


};

#endif //SIMPLE_OPENGL3_APP_H
