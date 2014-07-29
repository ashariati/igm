#include <ros/ros.h>
#include <signal.h>
#include <math.h>

#include <GL/glew.h>

#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <OVR.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <boost/thread/mutex.hpp>

#include <oculus/shader.hpp>
#include <oculus/objloader.hpp>
#include <oculus/simple_shape.h>
#include <oculus/transform_stack.h>

#include <vicon_msgs/Names.h>
#include <vicon_msgs/Values.h>

// Oculus
ovrHmd hmd;
ovrHmdDesc hmd_desc;
ovrFovPort eye_fov[2];
ovrGLConfig ovr_gl_config;
ovrEyeRenderDesc eye_render_desc[2];
ovrSizei client_size;

// GLFW
GLFWwindow* window;

// OpenGL
GLuint program_id;
GLuint vertex_buffer;
std::vector<glm::vec3> vertices;
std::vector<glm::vec2> uvs;
std::vector<glm::vec3> normals;

// Transforms
boost::mutex tf_mutex;
glm::mat4 view_mask;
glm::mat4 world_mask;
glm::mat4 view_ovr;
glm::mat4 world_ovr;
glm::mat4 world_model;

// Params
bool fullscreen;
std::string shaders_path;
std::string assets_path;

// Localization
bool locInit;
int rootIndex;

// My Shapes


const float trapezoid_w = 500.0f;
const float trapezoid_h = 2000.0f;
std::vector<glm::vec3> model_vertices;

void valuesCallback(const vicon_msgs::Values::ConstPtr& msg) {

    float ax = msg->values[rootIndex];
    float ay = msg->values[rootIndex + 1];
    float az = msg->values[rootIndex + 2];
    
    float xt = msg->values[rootIndex + 3];
    float yt = msg->values[rootIndex + 4];
    float zt = msg->values[rootIndex + 5];

    float theta = sqrt(ax*ax + ay*ay + az*az);
    float scale = 1.0;

    if (theta > 0.0) scale = sin(theta/2) / theta;

    glm::quat q = glm::normalize(glm::quat(cos(theta/2), scale*ax, scale*ay, scale*az));
    glm::mat4 M = 
        glm::translate(glm::mat4(1.0f), glm::vec3(xt, yt, zt)) *
        glm::mat4_cast(q);

    // ROS_INFO("%f, %f, %f, %f", theta, q[0], q[1], q[2]);
    // ROS_INFO("%f, %f, %f", xt, yt, zt);

    {
        boost::mutex::scoped_lock lock(tf_mutex);
        world_mask = M;
    }

}

void namesCallback(const vicon_msgs::Names::ConstPtr& msg) {

    std::string key = std::string("Oculus:Root <A-X>");

    for(int i = 0; i < msg->names.size(); i++) {
        if(!(key.compare(msg->names[i]))) {
            rootIndex = i;
            ROS_ERROR("%d", i);
        }
    }

    locInit = true;
}

glm::mat4 fromOVRMatrix4f(const OVR::Matrix4f &in) 
{
    glm::mat4 out;
    memcpy(glm::value_ptr(out), &in, sizeof(in));
    return out;
}

void cleanup(int sig) {
    
    // Delete buffers
    glDeleteBuffers(1, &vertex_buffer);

    // Delete compiled shaders
    glDeleteProgram(program_id);

    // Destroy objects
    glfwDestroyWindow(window);
    ovrHmd_Destroy(hmd);

    // Shutdown systems
    glfwTerminate();
    ovr_Shutdown();
    ros::requestShutdown();
}

void initGlew() {

    glewExperimental = GL_TRUE;
    GLenum result = glewInit();
    if(result != GLEW_OK) {
        ROS_ERROR("Failed to initialize glew");
        cleanup(1);
    }

}

void initGlfw() {

    if(!glfwInit()) {
        ROS_ERROR("Failed to initialize glfw");
        cleanup(1);
    }

    if (fullscreen) {
        int count;
        GLFWmonitor** monitors = glfwGetMonitors(&count);

        client_size.w = hmd_desc.Resolution.w;
        client_size.h = hmd_desc.Resolution.h;

        window = glfwCreateWindow(
                client_size.w,
                client_size.h,
                "Oculus View",
                monitors[1],
                NULL
                );
    } else {
        client_size.w = 640;
        client_size.h = 480;

        window = glfwCreateWindow(
                client_size.w,
                client_size.h,
                "Oculus View",
                NULL,
                NULL
                );
    }

    if(!window) {
        glfwTerminate();
        ROS_ERROR("Failed to create window");
        cleanup(1);
    }

    glfwMakeContextCurrent(window);

}

void initOvr() {

    ovr_Initialize();

    hmd = ovrHmd_Create(0);
    if(!hmd) {
        hmd = ovrHmd_CreateDebug(ovrHmd_DK1);
        ROS_ERROR("Oculus not connected properly");
        cleanup(1);
    }

    ovrHmd_GetDesc(hmd, &hmd_desc);

    ovrHmd_StartSensor(
            hmd,
            ovrSensorCap_Orientation |
            ovrSensorCap_YawCorrection |
            ovrSensorCap_Position,
            ovrSensorCap_Orientation
            );

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "oculus_node");
    signal(SIGINT, cleanup);

    ros::NodeHandle nh("~");

    std::string names_topic;
    ros::Subscriber names_sub;

    std::string values_topic;
    ros::Subscriber values_sub;

    ///////////////// ROS Parameters //////////////////

    nh.param("fullscreen", fullscreen, true);

    nh.param(
            "shaders_path", 
            shaders_path, 
            std::string("/home/vrwall/REU_Summer_2014/hydro_ws/src/igm/oculus/shaders")
            );

    nh.param(
            "assets_path",
            assets_path,
            std::string("/home/vrwall/REU_Summer_2014/hydro_ws/src/igm/oculus/assets")
            );

    nh.param("names_topic", names_topic, std::string("/vicon/names"));
    nh.param("values_topic", values_topic, std::string("/vicon/values"));

    ////////////////// Tracking Initialization ////////////////////

    locInit = false;
    names_sub = nh.subscribe(names_topic, 10, namesCallback);
    while(!locInit && ros::ok())
        ros::spinOnce();
    names_sub.shutdown();

    ///////////// Initialization Routines ////////////////

    initOvr();
    initGlfw();
    initGlew();

    ///////////////// Set OpenGL States //////////////
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);

    //////////// Texture parameters from OVR //////////////
    
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

    //////////// Framebuffer initialization ///////////////
    
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

    //////////// Oculus rendering configuration ///////////
    
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

    ////////////////////// Load Shaders ///////////////////
    
    std::string vertex_shader = 
        shaders_path + std::string("/StandardShading.vertexshader");

    std::string fragment_shader = 
        shaders_path + std::string("/StandardShading.fragmentshader");

    program_id = LoadShaders(
            vertex_shader.c_str(),
            fragment_shader.c_str()
            );

    /////////////////////// Load Assets ////////////////////////

    //      std::string test_object =
    //          assets_path + std::string("/suzanne.obj");
    //      if (!loadOBJ(test_object.c_str(), vertices, uvs, normals)) {
    //          ROS_ERROR("Cannot load object");
    //      }


    simple_shape::Pyramid pyramid;

    ////////////////// Buffer Initialization ///////////////
    
    // VAO
    GLuint vertex_array_id;
    glGenVertexArrays(1, &vertex_array_id);
    glBindVertexArray(vertex_array_id);

    ////////////////// Shader Uniforms /////////////////
    
    GLuint mvp_id = glGetUniformLocation(program_id, "MVP");

    //////////////// Initialize Transforms ///////////////

    // Column Wise Initialization
    view_mask = 
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -20.0f, -80.0f)) *
        glm::mat4(
            0, 0, -1, 0,
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1
            );

    world_mask = glm::mat4(1.0f);
    view_ovr = glm::mat4(1.0f);
    world_ovr = glm::mat4(1.0f);
    world_model = glm::mat4(1.0f);

    ////////////////// Subscribers ////////////////////////

    values_sub = nh.subscribe(values_topic, 10, valuesCallback);

    ros::Rate loop_rate(60);
    while(ros::ok()) {
        ros::spinOnce();
        
        // Must be called at the beginning of Oculus rendering
        ovrFrameTiming m_HmdFrameTiming = ovrHmd_BeginFrame(hmd, 0);

        // Shaders
        glUseProgram(program_id);

        // Bind framebuffer for OVR
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        // Background
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render twice, for each eye
        for (int eye_idx = 0; eye_idx < ovrEye_Count; eye_idx++)
        {
            ovrEyeType eye = hmd_desc.EyeRenderOrder[eye_idx];

            // Begin rendering for this eye
            ovrPosef eye_pose = ovrHmd_BeginEyeRender(hmd, eye);

            glViewport(eye_texture[eye].OGL.Header.RenderViewport.Pos.x,
                    eye_texture[eye].OGL.Header.RenderViewport.Pos.y,
                    eye_texture[eye].OGL.Header.RenderViewport.Size.w,
                    eye_texture[eye].OGL.Header.RenderViewport.Size.h
                    );
            
            /////////// Projection /////////////

            glm::mat4 projection_matrix = 
                fromOVRMatrix4f(
                        OVR::Matrix4f(
                            ovrMatrix4f_Projection(
                                eye_render_desc[eye].Fov, 
                                50.0f, 
                                5000.0f, 
                                true)).Transposed()
                        );
            
            TransformStack::getInstance().push(projection_matrix);

            ///////////// View /////////////////


            // Get data from the OVR
            //      glm::mat4 vo =
            //          fromOVRMatrix4f(
            //                  OVR::Matrix4f(
            //                      OVR::Quatf(eye_pose.Orientation)
            //                      )
            //                  );


            // Extract translational component of the transformation
            glm::vec3 view_adjust = 
                glm::vec3(
                        eye_render_desc[eye].ViewAdjust.x,
                        eye_render_desc[eye].ViewAdjust.y,
                        eye_render_desc[eye].ViewAdjust.z
                        );

            glm::mat4 vo_t = 
                glm::translate(
                        glm::mat4(1.f),
                        view_adjust
                        );

            TransformStack::getInstance().push(vo_t);
            TransformStack::getInstance().push(view_mask);
            TransformStack::getInstance().push(glm::inverse(world_mask));
            

            //////////////// Model ////////////


            // glm::mat4 model_matrix = 
            //     glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -5.0f, -20.0f));

            TransformStack::getInstance().push(
                    glm::scale(glm::mat4(1.0f), glm::vec3(1000.0f))
                        );

            //////////// Cumulative Transform /////////


            glm::mat4 mvp = TransformStack::getInstance().computeTransform();
            TransformStack::getInstance().clear();


            ///////////// Passing to shader pipeline //////////

            // Transform
            glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &mvp[0][0]);

            // Draw shape
            pyramid.draw();

            // End rendering for this eye
            ovrHmd_EndEyeRender(hmd, eye, eye_pose, &eye_texture[eye].Texture);
        }

        // Unbind vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // Unbind the framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Must call before rendering the next frame for the Oculus
        ovrHmd_EndFrame(hmd);

        // For any callbacks
        glfwPollEvents();

        loop_rate.sleep();
    }

}
