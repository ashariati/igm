#include <oculus/contact_model.h>

bool hitPaddle(glm::mat4 paddle, glm::vec3 dim, glm::mat4 sphere, float r) {

    glm::vec3 bounds = dim / 2.0f;

    glm::vec4 center = glm::vec4(sphere[3]);
    glm::vec4 v = glm::inverse(paddle) * center;

    bool inBounds_x = bounds[0] + r > abs(v[0]);
    bool inBounds_y = bounds[1] + r > abs(v[1]);
    bool inBounds_z = bounds[2] + r > abs(v[2]);
    bool inBounds = inBounds_x && inBounds_y && inBounds_z;

    if (inBounds) {
        return true;
    } else {
        return false;
    }

}

float distanceToPlane(glm::vec3 n, glm::vec3 p, glm::vec3 x) {

    glm::vec3 v = x - p;
    float d = glm::dot(n, v);

    return d;
}

