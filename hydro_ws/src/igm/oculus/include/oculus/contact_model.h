#ifndef CONTACT_MODEL
#define CONTACT_MODEL

#define GLM_FORCE_RADIANS

#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>


bool hitPaddle(glm::mat4 paddle, glm::vec3 dim, glm::mat4 sphere, float r);

#endif
