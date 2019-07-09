COMMON=/O2 /MT /EHsc /arch:AVX /I../../mujoco200/include ../../mujoco200/bin/mujoco200.lib /Fe../../mujoco200/bin/

all:
	cl $(COMMON) mjvive.cpp ../../mujoco200/bin/glfw3.lib ../../mujoco200/bin/glew32.lib ../../mujoco200/bin/openvr_api.lib /link OpenGL32.lib
	cl $(COMMON) minivive.cpp ../../mujoco200/bin/glfw3.lib ../../mujoco200/bin/glew32.lib ../../mujoco200/bin/openvr_api.lib /link OpenGL32.lib
	del *.obj