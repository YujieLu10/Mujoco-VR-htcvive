COMMON=/O2 /MT /EHsc /arch:AVX /I../include /Fe../bin/

all:
	cl $(COMMON) testxml.cpp                 ../bin/mujoco200nogl.lib
	cl $(COMMON) testspeed.cpp               ../bin/mujoco200nogl.lib
	cl $(COMMON) compile.cpp                 ../bin/mujoco200nogl.lib
	cl $(COMMON) derivative.cpp /openmp      ../bin/mujoco200nogl.lib
	cl $(COMMON) basic.cpp ../bin/glfw3.lib  ../bin/mujoco200.lib
	cl $(COMMON) record.cpp ../bin/glfw3.lib ../bin/mujoco200.lib
	cl $(COMMON) simulate.cpp ../include/uitools.c ../bin/glfw3.lib ../bin/mujoco200.lib
	del *.obj
