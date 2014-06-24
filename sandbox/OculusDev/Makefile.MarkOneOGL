VRPNPATH = ../vrpn
LIBOVRPATH = ../LibOVR
GLFWPATH = ../glfw-3.0.4
GLEWPATH = ../glew-1.10.0

INCPATH = -I/usr/include
OBJPATH = ./obj
BINPATH = ./bin
SRCPATH = ./src

DEPS = $(VRPNPATH)/vrpn_Tracker.h \
	   $(VRPNPATH)/vrpn_Analog.h \
	   $(VRPNPATH)/vrpn_Button.h \
	   $(LIBOVRPATH)/Include/OVR.h \
	   $(LIBOVRPATH)/Src/OVR_CAPI.h \
	   $(LIBOVRPATH)/Src/OVR_CAPI_GL.h \
	   $(GLFWPATH)/include/GLFW/glfw3.h \
	   $(GLFWPATH)/include/GLFW/glfw3native.h \
	   $(GLEWPATH)/include/GL/glew.h

LIBS = -L$(VRPNPATH)/pc_linux64 \
	   -L$(LIBOVRPATH)/Lib/Linux/Release/x86_64 \
	   -L$(GLFWPATH)/build/src \
	   -L$(GLEWPATH)/lib \
	   -lovr \
	   -lvrpn \
	   -lGL \
	   -lGLU \
	   -lglut \
	   -lglfw3 \
	   -lGLEW \
	   -lXi \
	   -lXrandr \
	   -lXxf86vm \
	   -lX11 \
	   -lm \
	   -lpthread \
	   -ludev

TARGET = $(BINPATH)/MarkOneOGL

$(TARGET): $(OBJPATH)/MarkOneOGL.o
	g++ -Wall -o $(TARGET) $(OBJPATH)/MarkOneOGL.o $(LIBS)

$(OBJPATH)/MarkOneOGL.o: $(SRCPATH)/MarkOneOGL.cpp $(DEPS)
	g++ -Wall -c -o $(OBJPATH)/MarkOneOGL.o $(SRCPATH)/MarkOneOGL.cpp

clean:
	rm -f $(OBJPATH)/*
	rm -f $(BINPATH)/*
