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

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

// Use to load .obj in the future
// #include <assimp/Importer.hpp>

// Use to load .DDS
// #include <SOIL.h>

GLFWwindow* window;

ovrHmd hmd;
ovrHmdDesc hmd_desc;
ovrFovPort eye_fov[2];
ovrGLConfig ovr_gl_config;
ovrEyeRenderDesc eye_render_desc[2];

std::vector<glm::vec3> vertices;
std::vector<glm::vec2> uvs;
std::vector<glm::vec3> normals;

vrpn_Tracker_Remote* vrpn_tracker;
float wand_trans[3];
float scale = 1.0f;

void VRPN_CALLBACK handleTracker(void* userData, const vrpn_TRACKERCB t)
{
    wand_trans[0] = (float) t.pos[0] * scale;
    wand_trans[1] = (float) t.pos[1] * scale;
    wand_trans[2] = (float) t.pos[2] * scale;

    std::cout << wand_trans[0] << "," <<  \
        wand_trans[1] << "," << \
        wand_trans[2] << std::endl;
}

static void windowSizeCallback(GLFWwindow* p_Window, int p_Width, int p_Height)
{
    ovr_gl_config.OGL.Header.RTSize.w = p_Width; 
    ovr_gl_config.OGL.Header.RTSize.h = p_Height;

    int distortion_caps 
        = ovrDistortionCap_Chromatic | ovrDistortionCap_TimeWarp;

    ovrHmd_ConfigureRendering(hmd, 
            &ovr_gl_config.Config, 
            distortion_caps, 
            eye_fov, 
            eye_render_desc);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}


static void setOpenGLState(void)
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
    vrpn_tracker = 
        new vrpn_Tracker_Remote("Oculus@158.130.62.126:3883");
    vrpn_tracker->register_change_handler(0, handleTracker);
}

void initOvr(void) {
    // Initialize LibOVR...
    ovr_Initialize();
    hmd = ovrHmd_Create(0);
    if (!hmd) 
        hmd = ovrHmd_CreateDebug(ovrHmd_DK1);
    ovrHmd_GetDesc(hmd, &hmd_desc);

    // Start the sensor which provides the Rift’s pose and motion.
    ovrHmd_StartSensor(hmd, 
            ovrSensorCap_Orientation | 
            ovrSensorCap_YawCorrection | 
            ovrSensorCap_Position, 
            ovrSensorCap_Orientation);
}

glm::mat4 fromOVRMatrix4f(const OVR::Matrix4f &in) {
    glm::mat4 out;
    memcpy(glm::value_ptr(out), &in, sizeof(in));
    return out;
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

    glewExperimental = GL_TRUE;
    GLenum l_Result = glewInit();
    if (l_Result!=GLEW_OK) {
        printf("glewInit() error.\n");
        exit(EXIT_FAILURE);
    }

    // There is some more old gross code in here that needs to be removed
    // Create some lights, materials, etc...
    setOpenGLState();


    /* 
     * Rendering to a framebuffer requires a bound texture, so we need
     * to get some of the parameters of the texture from OVR.
     */
    ovrSizei texture_size_left = ovrHmd_GetFovTextureSize(hmd, 
            ovrEye_Left, 
            hmd_desc.DefaultEyeFov[0], 
            1.0f);
    ovrSizei texture_size_right = ovrHmd_GetFovTextureSize(hmd, 
            ovrEye_Right, 
            hmd_desc.DefaultEyeFov[1], 
            1.0f);
    ovrSizei texture_size;
    texture_size.w = texture_size_left.w + texture_size_right.w;
    texture_size.h = (texture_size_left.h > texture_size_right.h \
            ? texture_size_left.h : texture_size_right.h);


    /*
     * Run of the mill framebuffer initialization
     */
    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    GLuint color_texture_id;
    glGenTextures(1, &color_texture_id);
    glBindTexture(GL_TEXTURE_2D, color_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 
            0, 
            GL_RGBA, 
            texture_size.w, 
            texture_size.h, 
            0, 
            GL_RGBA, 
            GL_UNSIGNED_BYTE, 
            0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    GLuint depth_buffer_id;
    glGenRenderbuffers(1, &depth_buffer_id);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_id);
    glRenderbufferStorage(GL_RENDERBUFFER, 
            GL_DEPTH_COMPONENT, 
            texture_size.w, 
            texture_size.h);

    glFramebufferTexture(GL_FRAMEBUFFER, 
            GL_COLOR_ATTACHMENT0, 
            color_texture_id, 
            0);

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, 
            GL_DEPTH_ATTACHMENT, 
            GL_RENDERBUFFER, 
            depth_buffer_id);

    static const GLenum draw_buffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, draw_buffers);

    GLenum fbo_check = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (fbo_check != GL_FRAMEBUFFER_COMPLETE) {
        printf("There is a problem with the FBO.\n");
        exit(EXIT_FAILURE);
    }

    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    /*
     * Some oculus configurations
     */

    eye_fov[0] = hmd_desc.DefaultEyeFov[0];
    eye_fov[1] = hmd_desc.DefaultEyeFov[1];

    ovr_gl_config.OGL.Header.API = ovrRenderAPI_OpenGL;
    ovr_gl_config.OGL.Header.Multisample = 0;
    ovr_gl_config.OGL.Header.RTSize.w = client_size.w;
    ovr_gl_config.OGL.Header.RTSize.h = client_size.h;
    ovr_gl_config.OGL.Win = glfwGetX11Window(window);
    ovr_gl_config.OGL.Disp = glfwGetX11Display();

    int distortion_caps = ovrDistortionCap_Chromatic | 
        ovrDistortionCap_TimeWarp;

    ovrHmd_ConfigureRendering(hmd, 
            &ovr_gl_config.Config, 
            distortion_caps, 
            eye_fov, 
            eye_render_desc);

    ovrGLTexture eye_texture[2];
    eye_texture[0].OGL.Header.API = ovrRenderAPI_OpenGL;
    eye_texture[0].OGL.Header.TextureSize.w = texture_size.w;
    eye_texture[0].OGL.Header.TextureSize.h = texture_size.h;
    eye_texture[0].OGL.Header.RenderViewport.Pos.x = 0;
    eye_texture[0].OGL.Header.RenderViewport.Pos.y = 0;
    eye_texture[0].OGL.Header.RenderViewport.Size.w = texture_size.w/2;
    eye_texture[0].OGL.Header.RenderViewport.Size.h = texture_size.h;
    eye_texture[0].OGL.TexId = color_texture_id;
    // Right eye the same, except for the x-position in the texture...
    eye_texture[1] = eye_texture[0];
    eye_texture[1].OGL.Header.RenderViewport.Pos.x = (texture_size.w+1)/2;



    GLuint program_id = LoadShaders("../shaders/StandardShading.vertexshader",
            "../shaders/StandardShading.fragmentshader");

    bool res = loadOBJ("../assets/suzanne.obj", vertices, uvs, normals);

    GLuint vertex_array_id;
    glGenVertexArrays(1, &vertex_array_id);
    glBindVertexArray(vertex_array_id);

    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER,
            vertices.size() * sizeof(glm::vec3),
            &vertices[0],
            GL_STATIC_DRAW);



    GLuint mvp_id = glGetUniformLocation(program_id, "MVP");

    //      glfwSetWindowSizeCallback(window, windowSizeCallback);

    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && \
            !glfwWindowShouldClose(window))
    {
        vrpn_tracker->mainloop();
        ovrFrameTiming m_HmdFrameTiming = ovrHmd_BeginFrame(hmd, 0);

        glUseProgram(0);
        //      glUseProgram(program_id);


        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (int eye_idx = 0; eye_idx < ovrEye_Count; eye_idx++)
        {
            ovrEyeType eye = hmd_desc.EyeRenderOrder[eye_idx];
            ovrPosef eye_pose = ovrHmd_BeginEyeRender(hmd, eye);

            glViewport(eye_texture[eye].OGL.Header.RenderViewport.Pos.x,
                    eye_texture[eye].OGL.Header.RenderViewport.Pos.y,
                    eye_texture[eye].OGL.Header.RenderViewport.Size.w,
                    eye_texture[eye].OGL.Header.RenderViewport.Size.h
                    );

            // Projection
            OVR::Matrix4f projection_matrix = ovrMatrix4f_Projection(
                    eye_render_desc[eye].Fov, 
                    0.3f, 
                    100.0f, 
                    true);
            
            // Model
            OVR::Quatf orientation = OVR::Quatf(eye_pose.Orientation);
            OVR::Matrix4f model_matrix = 
                OVR::Matrix4f(orientation.Inverted());

            //      // View
            //      glm::vec3 translation = glm::vec3(
            //              eye_render_desc[eye].ViewAdjust.x,
            //              eye_render_desc[eye].ViewAdjust.y,
            //              eye_render_desc[eye].ViewAdjust.z - 2.0f);
            //      glm::mat4 view_matrix = glm::translate(glm::mat4(1.f), 
            //              translation);

            //      // MVP 
            //      glm::mat4 mvp = fromOVRMatrix4f(projection_matrix) *
            //          view_matrix *
            //          fromOVRMatrix4f(model_matrix);


            //      glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &mvp[0][0]);


            // Gross old stuff, we want what is just above to work
            // Pass matrici on to OpenGL...
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glMultMatrixf(&(projection_matrix.Transposed().M[0][0]));
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            // Translate for specific eye based on IPD...
            glTranslatef(eye_render_desc[eye].ViewAdjust.x,
                    eye_render_desc[eye].ViewAdjust.y,
                    eye_render_desc[eye].ViewAdjust.z);
            // Multiply with orientation retrieved from sensor...
            glMultMatrixf(&(model_matrix.Transposed().M[0][0]));
            // Move back a bit to show scene in front of us...
            glTranslatef(0.0f, 0.0f, -2.0f);
            // Integrate position data from OptiTrack
            glTranslatef(wand_trans[0], wand_trans[1], wand_trans[2]);



            // Render...
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
            glDrawArrays(GL_LINES, 0, vertices.size());
            glDisableVertexAttribArray(0);

            ovrHmd_EndEyeRender(hmd, eye, eye_pose, &eye_texture[eye].Texture);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        ovrHmd_EndFrame(hmd);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glfwPollEvents();
    }

    glDeleteBuffers(1, &vertex_buffer);
    glDeleteProgram(program_id);

    glfwDestroyWindow(window);
    glfwTerminate();

    ovrHmd_Destroy(hmd);
    ovr_Shutdown();

    exit(EXIT_SUCCESS);
}
