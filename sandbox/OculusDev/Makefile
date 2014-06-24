VRPNPATH = ../vrpn
LIBOVRPATH = ../LibOVR
GLFWPATH = ../glfw-3.0.4
GLEWPATH = ../glew-1.10.0
GLMPATH = ../glm

INCPATH = -I./include \
		  -I/usr/include \
		  -I$(GLMPATH) \
		  -I$(GLEWPATH)/include \
		  -I$(GLFWPATH)/include \
		  -I$(LIBOVRPATH)/Include \
		  -I$(LIBOVRPATH)/Src \
		  -I$(VRPNPATH)

OBJPATH = ./obj
BINPATH = ./bin
SRCPATH = ./src

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

$(TARGET): $(OBJPATH)/MarkOneOGL.o $(OBJPATH)/objloader.o
	g++ -Wall -o $(TARGET) $(OBJPATH)/MarkOneOGL.o $(OBJPATH)/objloader.o $(LIBS)

$(OBJPATH)/MarkOneOGL.o: $(SRCPATH)/MarkOneOGL.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/MarkOneOGL.o $(SRCPATH)/MarkOneOGL.cpp

$(OBJPATH)/objloader.o: $(SRCPATH)/objloader.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/objloader.o $(SRCPATH)/objloader.cpp

clean:
	rm -f $(OBJPATH)/*
	rm -f $(BINPATH)/*
