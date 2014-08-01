#include <oculus/ball_model.h>

BallModel::BallModel(glm::mat4 p, float r) {

    pose = p;
    vel = glm::vec3(0.0f, 0.0f, 0.0f);
    radius = r;

    setPaddleDim(0.0f, 0.0f, 0.0f);
    setRoomDim(0.0f, 0.0f, 0.0f);

}

glm::mat4 BallModel::update(glm::mat4 paddle, float deltaTime) {

    vel = velocityFromPaddle(paddle) + velocityFromWall();

    pose = glm::translate(
            glm::mat4(1.0f),
            glm::vec3(
                vel[0] * deltaTime,
                vel[1] * deltaTime,
                vel[2] * deltaTime
                )
            );

    return pose;

}

float BallModel::distanceToPlane(glm::vec3 n, glm::vec3 p, glm::vec3 x) {

    glm::vec3 v = x - p;
    float d = glm::dot(n, v);

    return d;
}

glm::vec3 BallModel::velocityFromPaddle(glm::mat4 paddle) {
    glm::vec4 center = glm::vec4(pose[3]);
    glm::vec3 r = glm::vec3(glm::inverse(paddle) * center);

    bool inBounds_x = (paddle_x / 2) + radius > abs(r[0]);
    bool inBounds_y = (paddle_y / 2) > abs(r[1]);
    bool inBounds_z = (paddle_z / 2) + radius > abs(r[2]);

    bool inBounds = inBounds_x && inBounds_y && inBounds_z;

    if (inBounds) {
        float D = distanceToPlane(paddle_n, glm::vec3(0.0f), r);
        if (D > 0.0f)
            return glm::reflect(vel, paddle_n);
        else
            return glm::reflect(vel, -paddle_n);

    } else {
        return glm::vec3(0.0f);
    }

}

glm::vec3 BallModel::velocityFromWall() {
    glm::vec3 r = glm::vec3(pose[3]);

    float D_front = distanceToPlane(room_front_n, room_front_p, r);
    float D_back = distanceToPlane(room_back_n, room_back_p, r);
    float D_left= distanceToPlane(room_left_n, room_left_p, r);
    float D_right = distanceToPlane(room_right_n, room_right_p, r);
    float D_top = distanceToPlane(room_top_n, room_top_p, r);
    float D_bottom = distanceToPlane(room_bottom_n, room_bottom_p, r);

    bool inFront = D_front > 0.0f;
    bool inBack = D_back > 0.0f;
    bool inLeft = D_left > 0.0f;
    bool inRight = D_right > 0.0f;
    bool inTop = D_top > 0.0f;
    bool inBottom = D_bottom > 0.0f;

    bool inRoom_x = inFront && inBack;
    bool inRoom_y = inLeft && inRight;
    bool inRoom_z = inTop && inBottom;
    bool inRoom = inRoom_x && inRoom_y && inRoom_z;

    if (inRoom) 

        return glm::vec3(0.0f);

    else 
        if (!inRoom_x) {

            if (!inFront)
                return glm::reflect(vel, room_front_n);
            else if (!inBack)
                return glm::reflect(vel, room_back_n);

        } else if (!inRoom_y) {

            if (!inLeft) 
                return glm::reflect(vel, room_left_n);
            else if (!inRight) 
                return glm::reflect(vel, room_right_n);

        } else if (!inRoom_z) {

            if (!inTop)
                return glm::reflect(vel, room_top_n);
            else if (!inBottom)
                return glm::reflect(vel, room_bottom_n);

        }
}


bool BallModel::inContact(glm::mat4 paddle) {

    glm::vec4 center = glm::vec4(pose[3]);
    glm::vec4 v = glm::inverse(paddle) * center;

    bool inBounds_x = (paddle_x / 2)  + radius > abs(v[0]);
    bool inBounds_y = (paddle_y / 2)  + radius > abs(v[1]);
    bool inBounds_z = (paddle_z / 2)  + radius > abs(v[2]);

    bool inBounds = inBounds_x && inBounds_y && inBounds_z;

    return inBounds ? true : false;

}

void BallModel::setPaddleDim(float x, float y, float z) {
    paddle_x = x; paddle_y = y; paddle_z = z;

    paddle_n = glm::vec3(0.0f, 1.0f, 0.0f);

}

void BallModel::setRoomDim(float x, float y, float z) {

    room_front_n = glm::vec3(-1.0f, 0.0f, 0.0f);
    room_back_n = glm::vec3(1.0f, 0.0f, 0.0f);
    room_left_n = glm::vec3(0.0f, -1.0f, 0.0f);
    room_right_n = glm::vec3(0.0f, 1.0f, 0.0f);
    room_top_n = glm::vec3(0.0f, 0.0f, -1.0f);
    room_bottom_n = glm::vec3(0.0f, 0.0f, 1.0f);

    room_front_p = glm::vec3(x/2, 0.0f, z/2);
    room_back_p = glm::vec3(-x/2, 0.0f, z/2);
    room_left_p = glm::vec3(0.0f, y/2, z/2);
    room_right_p = glm::vec3(0.0f, -y/2, z/2);
    room_top_p = glm::vec3(0.0f, 0.0f, z);
    room_bottom_p = glm::vec3(0.0f, 0.0f, 0.0f);

}
