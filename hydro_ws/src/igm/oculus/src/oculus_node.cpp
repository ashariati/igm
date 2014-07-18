#include <ros/ros.h>

#include <OVR.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

ovrHmd hmd;
ovrHmdDesc hmd_desc;
ovrFovPort eye_fov[2];
ovrGLConfig ovr_gl_config;
ovrEyeRenderDesc eye_render_desc[2];

int main(int argc, char** argv) {

    ros::init(argc, argv, "oculus_node");
    ros::NodeHandle nh("~");

    ovr_Initialize();
    hmd = ovrHmd_Create(0);
    
    if(!hmd) {
        hmd = ovrHmd_CreateDebug(ovrHmd_DK1);
        ROS_ERROR("Oculus not connected properly");
    }

    ovrHmd_StartSensor(
            hmd,
            ovrSensorCap_Orientation |
            ovrSensorCap_YawCorrection |
            ovrSensorCap_Position,
            ovrSensorCap_Orientation
            );

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
