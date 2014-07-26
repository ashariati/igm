#include <playground/transform_stack.h>


void TransformStack::push(glm::mat4 M) {
    T.push(M);
}

glm::mat4 TransformStack::pop() {
    glm::mat4 top = T.top();
    T.pop();
    return top;
}

glm::mat4 TransformStack::computeTransform() {

    glm::mat4 M = glm::mat4(1.0f);

    for (int i = 0; i < T.size(); i++) {
        glm::mat4 top = T.top();
        std::cout << top << std::endl;
        M = top * M;
        T.pop();
    }

    return M;

}

