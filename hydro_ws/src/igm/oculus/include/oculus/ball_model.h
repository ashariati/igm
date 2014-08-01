#ifndef BALL_MODEL
#define BALL_MODEL

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class BallModel {
    public:
        BallModel(glm::mat4 p, float r);

        glm::mat4 update(glm::mat4 paddle, float deltaTime);

        bool inContact(glm::mat4 paddle);

        void setPaddleDim(float x, float y, float z);
        void setRoomDim(float x, float y, float z);

    private:

        // Private Methods
        float distanceToPlane(glm::vec3 n, glm::vec3 p, glm::vec3 x);
        glm::vec3 velocityFromPaddle(glm::mat4 paddle);
        glm::vec3 velocityFromWall();

        // Private Members
        glm::mat4 pose;
        glm::vec3 vel;
        float radius;

        float paddle_x;
        float paddle_y;
        float paddle_z;

        glm::vec3 paddle_n;

        glm::vec3 room_front_n;
        glm::vec3 room_back_n;
        glm::vec3 room_left_n;
        glm::vec3 room_right_n;
        glm::vec3 room_top_n;
        glm::vec3 room_bottom_n;

        glm::vec3 room_front_p;
        glm::vec3 room_back_p;
        glm::vec3 room_left_p;
        glm::vec3 room_right_p;
        glm::vec3 room_top_p;
        glm::vec3 room_bottom_p;



};

#endif
