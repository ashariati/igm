#include <ros/ros.h>

#include <GL/glew.h>

#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#include <GLFW/glfw3.h>

#include <OVR.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

// Oculus
ovrHmd hmd;
ovrHmdDesc hmd_desc;
ovrFovPort eye_fov[2];
ovrGLConfig ovr_gl_config;
ovrEyeRenderDesc eye_render_desc[2];
ovrSizei client_size;

// GLFW
GLFWwindow* window;
bool fullscreen;

void initGlew() {

    glewExperimental = GL_TRUE;
    GLenum result = glewInit();
    if(result != GLEW_OK) {
        ROS_ERROR("Failed to initialize glew");
        ros::shutdown();
    }
}

void initGlfw() {

    if(!glfwInit()) {
        ROS_ERROR("Failed to initialize glfw");
        ros::shutdown();
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
        ros::shutdown();
    }

    glfwMakeContextCurrent(window);

}

void initOvr() {

    ovr_Initialize();

    hmd = ovrHmd_Create(0);
    if(!hmd) {
        hmd = ovrHmd_CreateDebug(ovrHmd_DK1);
        ROS_ERROR("Oculus not connected properly");
        ros::shutdown();
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
    ros::NodeHandle nh("~");

    nh.param("fullscreen", fullscreen, true);

    initOvr();
    initGlfw();
    initGlew();

    ros::Rate loop_rate(60);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

}
