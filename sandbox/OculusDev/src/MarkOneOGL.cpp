#include <GL/glew.h>
#include <X11/X.h>
#include <X11/extensions/Xrandr.h>
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <OVR.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>

#include <iostream>

#include <objloader.hpp>
#include <texture.hpp>
#include <shader.hpp>

// Use to load .obj in the future
// #include <assimp/Importer.hpp>

// Use to load .DDS
// #include <SOIL.h>

GLFWwindow* window;

ovrHmd l_Hmd;
ovrHmdDesc l_HmdDesc;
ovrFovPort l_EyeFov[2];
ovrGLConfig l_Cfg;
ovrEyeRenderDesc l_EyeRenderDesc[2];

std::vector<glm::vec3> vertices;
std::vector<glm::vec2> uvs;
std::vector<glm::vec3> normals;

vrpn_Tracker_Remote* vrpnTracker;
float wand_trans[3];
float scale = 1.0f;

void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
    wand_trans[0] = (float) t.pos[0] * scale;
    wand_trans[1] = (float) t.pos[1] * scale;
    wand_trans[2] = (float) t.pos[2] * scale;

    std::cout << wand_trans[0] << "," <<  \
        wand_trans[1] << "," << \
        wand_trans[2] << std::endl;
}

static void KeyCallback(GLFWwindow* p_Window, 
        int p_Key, int p_Scancode, int p_Action, int p_Mods)
{
    if (p_Key == GLFW_KEY_ESCAPE && p_Action == GLFW_PRESS) 
        glfwSetWindowShouldClose(p_Window, GL_TRUE);
}

static void WindowSizeCallback(GLFWwindow* p_Window, int p_Width, int p_Height)
{
    l_Cfg.OGL.Header.RTSize.w = p_Width; 
    l_Cfg.OGL.Header.RTSize.h = p_Height;

    int l_DistortionCaps 
        = ovrDistortionCap_Chromatic | ovrDistortionCap_TimeWarp;

    ovrHmd_ConfigureRendering(l_Hmd, 
            &l_Cfg.Config, 
            l_DistortionCaps, 
            l_EyeFov, 
            l_EyeRenderDesc);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

void RenderVertexArrays(void)
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);

    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, 0, &normals[0]);

    // glDrawElements(GL_QUADS, 6*4, GL_UNSIGNED_INT, l_VAIndici);
    glDrawArrays(GL_TRIANGLES, 0, vertices.size());

    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}

static void SetOpenGLState(void)
{
    // Some state...
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Some (stationary) lights...
    GLfloat l_Light0Position[] = { 5.0f, 6.0f, 3.0f, 0.0f };
    GLfloat l_Light0Diffuse[] = { 1.0f, 0.8f, 0.6f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, l_Light0Position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, l_Light0Diffuse);
    glEnable(GL_LIGHT0);

    GLfloat l_Light1Position[] = { -5.0f, -6.0f, 5.0f, 0.0f };
    GLfloat l_Light1Diffuse[] = { 0.6f, 0.8f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, l_Light1Position);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, l_Light1Diffuse);
    glEnable(GL_LIGHT1);

    // Material...
    GLfloat l_MaterialSpecular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat l_MaterialShininess[] = { 10.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, l_MaterialSpecular);
    glMaterialfv(GL_FRONT, GL_SHININESS, l_MaterialShininess);
}

// Initialize our VRPN toolkit to connect to the Optitrack
void initVrpn(void) {
    vrpnTracker = 
        new vrpn_Tracker_Remote("Oculus@158.130.62.126:3883");
    vrpnTracker->register_change_handler(0, handle_tracker);
}

void initOvr(void) {
    // Initialize LibOVR...
    ovr_Initialize();
    l_Hmd = ovrHmd_Create(0);
    if (!l_Hmd) 
        l_Hmd = ovrHmd_CreateDebug(ovrHmd_DK1);
    ovrHmd_GetDesc(l_Hmd, &l_HmdDesc);

    // Start the sensor which provides the Rift’s pose and motion.
    ovrHmd_StartSensor(l_Hmd, 
            ovrSensorCap_Orientation | 
            ovrSensorCap_YawCorrection | 
            ovrSensorCap_Position, 
            ovrSensorCap_Orientation);
}

int main(void)
{
    initVrpn();
    initOvr();

    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize glfw!\n");
        exit(EXIT_FAILURE);
    }

    ovrSizei client_size;
    client_size.w = 640;
    client_size.h = 480;

    window = glfwCreateWindow(client_size.w, 
            client_size.h, 
            "GLFW Oculus Rift Test", 
            NULL, 
            NULL);

    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    /************* GET RID OF THIS JUNK ***********************/

    // Don't forget to initialize Glew, turn glewExperimental on to avoid problem fetching function pointers...
    glewExperimental = GL_TRUE;
    GLenum l_Result = glewInit();
    if (l_Result!=GLEW_OK) {
        printf("glewInit() error.\n");
        exit(EXIT_FAILURE);
    }

    // Print some info about the OpenGL context...
    int l_Major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    int l_Minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    int l_Profile = glfwGetWindowAttrib(window, GLFW_OPENGL_PROFILE);
    printf("OpenGL: %d.%d ", l_Major, l_Minor);

    // Profiles introduced in OpenGL 3.0...
    if (l_Major>=3) {
        if (l_Profile==GLFW_OPENGL_COMPAT_PROFILE) 
            printf("GLFW_OPENGL_COMPAT_PROFILE\n"); 
        else 
            printf("GLFW_OPENGL_CORE_PROFILE\n");
    }
    printf("Vendor: %s\n", (char*)glGetString(GL_VENDOR));
    printf("Renderer: %s\n", (char*)glGetString(GL_RENDERER));

    /*********************************************************/

    // Create some lights, materials, etc...
    SetOpenGLState();

    // We will do some offscreen rendering, setup FBO...
    ovrSizei texture_size_left = ovrHmd_GetFovTextureSize(l_Hmd, 
            ovrEye_Left, 
            l_HmdDesc.DefaultEyeFov[0], 
            1.0f);

    ovrSizei texture_size_right = ovrHmd_GetFovTextureSize(l_Hmd, 
            ovrEye_Right, 
            l_HmdDesc.DefaultEyeFov[1], 
            1.0f);

    ovrSizei texture_size;
    texture_size.w = texture_size_left.w + texture_size_right.w;
    texture_size.h = (texture_size_left.h > texture_size_right.h \
            ? texture_size_left.h : texture_size_right.h);

    // Compile Shaders
    GLuint program_id = LoadShaders("../shaders/StandardShading.vertexshader",
            "../shaders/StandardShading.fragmentshader");

    // Create FBO...
    GLuint l_FBOId;
    glGenFramebuffers(1, &l_FBOId);
    glBindFramebuffer(GL_FRAMEBUFFER, l_FBOId);

    // Load my textures and objects
    bool res = loadOBJ("../assets/suzanne.obj", vertices, uvs, normals);

    // GLuint l_TextureId = SOIL_load_OGL_texture(
    //         "../assets/uvmap.DDS",
    //         SOIL_LOAD_AUTO,
    //         SOIL_CREATE_NEW_ID,
    //         SOIL_FLAG_DDS_LOAD_DIRECT);
    GLuint l_TextureId = loadDDS("../assets/uvmap.DDS");

    //  glTexStorage2D(GL_TEXTURE_2D,
    //          1,
    //          GL_RGBA,
    //          texture_size.w,
    //          texture_size.h);

    //  glTexSubImage2D(GL_TEXTURE_2D,
    //          0,
    //          0, 0,
    //          texture_size.w,
    //          texture_size.h,
    //          GL_RGBA,
    //          GL_FLOAT,
    //          &uvs[0]);


    //  // The texture we're going to render to...
    //  GLuint l_TextureId;
    //  glGenTextures(1, &l_TextureId);
    //  // "Bind" the newly created texture : 
    //  // all future texture functions will modify this texture...
    //  glBindTexture(GL_TEXTURE_2D, l_TextureId);

    // Give an empty image to OpenGL (the last "0")
    glTexImage2D(GL_TEXTURE_2D, 
            0, 
            GL_RGBA, 
            texture_size.w, 
            texture_size.h, 
            0, 
            GL_RGBA, 
            GL_UNSIGNED_BYTE, 
            0);

    // Linear filtering...
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    printf("%d\n\n", (int) l_TextureId);


    // Create Depth Buffer...
    GLuint l_DepthBufferId;
    glGenRenderbuffers(1, &l_DepthBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, l_DepthBufferId);

    glRenderbufferStorage(GL_RENDERBUFFER, 
            GL_DEPTH_COMPONENT, 
            texture_size.w, 
            texture_size.h);

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, 
            GL_DEPTH_ATTACHMENT, 
            GL_RENDERBUFFER, 
            l_DepthBufferId);


    // Set the texture as our colour attachment #0...
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, l_TextureId, 0);

    // Set the list of draw buffers...
    GLenum l_GLDrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, l_GLDrawBuffers); // "1" is the size of DrawBuffers

    // Check if everything is OK...
    GLenum l_Check = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (l_Check!=GL_FRAMEBUFFER_COMPLETE) {
        printf("There is a problem with the FBO.\n");
        exit(EXIT_FAILURE);
    }

    // Unbind...
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Oculus Rift eye configurations...
    l_EyeFov[0] = l_HmdDesc.DefaultEyeFov[0];
    l_EyeFov[1] = l_HmdDesc.DefaultEyeFov[1];

    l_Cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
    l_Cfg.OGL.Header.Multisample = 0;
    l_Cfg.OGL.Header.RTSize.w = client_size.w;
    l_Cfg.OGL.Header.RTSize.h = client_size.h;

    l_Cfg.OGL.Win = glfwGetX11Window(window);
    l_Cfg.OGL.Disp = glfwGetX11Display();

    int l_DistortionCaps = ovrDistortionCap_Chromatic | ovrDistortionCap_TimeWarp;
    ovrHmd_ConfigureRendering(l_Hmd, &l_Cfg.Config, l_DistortionCaps, l_EyeFov, l_EyeRenderDesc);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(0);

    ovrGLTexture l_EyeTexture[2];
    l_EyeTexture[0].OGL.Header.API = ovrRenderAPI_OpenGL;
    l_EyeTexture[0].OGL.Header.TextureSize.w = texture_size.w;
    l_EyeTexture[0].OGL.Header.TextureSize.h = texture_size.h;
    l_EyeTexture[0].OGL.Header.RenderViewport.Pos.x = 0;
    l_EyeTexture[0].OGL.Header.RenderViewport.Pos.y = 0;
    l_EyeTexture[0].OGL.Header.RenderViewport.Size.w = texture_size.w/2;
    l_EyeTexture[0].OGL.Header.RenderViewport.Size.h = texture_size.h;
    l_EyeTexture[0].OGL.TexId = l_TextureId;

    // Right eye the same, except for the x-position in the texture...
    l_EyeTexture[1] = l_EyeTexture[0];
    l_EyeTexture[1].OGL.Header.RenderViewport.Pos.x = (texture_size.w+1)/2;

    glfwSetKeyCallback(window, KeyCallback);
    glfwSetWindowSizeCallback(window, WindowSizeCallback);

    while (!glfwWindowShouldClose(window))
    {
        // spin
        vrpnTracker->mainloop();


        ovrFrameTiming m_HmdFrameTiming = ovrHmd_BeginFrame(l_Hmd, 0);

        // Bind the FBO...
        glBindFramebuffer(GL_FRAMEBUFFER, l_FBOId);

        // Clear...
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (int l_EyeIndex=0; l_EyeIndex<ovrEye_Count; l_EyeIndex++)
        {
            ovrEyeType l_Eye = l_HmdDesc.EyeRenderOrder[l_EyeIndex];
            ovrPosef l_EyePose = ovrHmd_BeginEyeRender(l_Hmd, l_Eye);

            glViewport(l_EyeTexture[l_Eye].OGL.Header.RenderViewport.Pos.x,
                    l_EyeTexture[l_Eye].OGL.Header.RenderViewport.Pos.y,
                    l_EyeTexture[l_Eye].OGL.Header.RenderViewport.Size.w,
                    l_EyeTexture[l_Eye].OGL.Header.RenderViewport.Size.h
                    );

            // Get Projection and ModelView matrici from the device...
            OVR::Matrix4f l_ProjectionMatrix = ovrMatrix4f_Projection(
                    l_EyeRenderDesc[l_Eye].Fov, 0.3f, 100.0f, true);
            OVR::Quatf l_Orientation = OVR::Quatf(l_EyePose.Orientation);
            OVR::Matrix4f l_ModelViewMatrix = OVR::Matrix4f(l_Orientation.Inverted());

            // Pass matrici on to OpenGL...
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glMultMatrixf(&(l_ProjectionMatrix.Transposed().M[0][0]));
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            // Translate for specific eye based on IPD...
            glTranslatef(l_EyeRenderDesc[l_Eye].ViewAdjust.x,
                    l_EyeRenderDesc[l_Eye].ViewAdjust.y,
                    l_EyeRenderDesc[l_Eye].ViewAdjust.z);
            // Multiply with orientation retrieved from sensor...
            glMultMatrixf(&(l_ModelViewMatrix.Transposed().M[0][0]));

            // Move back a bit to show scene in front of us...
            glTranslatef(0.0f, 0.0f, -2.0f);

            // Integrate position data from OptiTrack
            glTranslatef(wand_trans[0], wand_trans[1], wand_trans[2]);

            // Render...
            RenderVertexArrays();

            ovrHmd_EndEyeRender(l_Hmd, l_Eye, l_EyePose, &l_EyeTexture[l_Eye].Texture);
        }

        // Unbind the FBO, back to normal drawing...
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        ovrHmd_EndFrame(l_Hmd);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);

    glfwTerminate();

    ovrHmd_Destroy(l_Hmd);
    ovr_Shutdown();

    exit(EXIT_SUCCESS);
}
