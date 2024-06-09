bool initCamera(int width, int height);
extern void captureFrame();
void cameraSecondaryThreadMain();
extern bool isCapturing();
extern uint8_t* framebuffer;
extern bool locked;
bool getMotion();
void resetMotion();