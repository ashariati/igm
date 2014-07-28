#include <oculus/transform_stack.h>


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
    int len = T.size();

    std::stack<glm::mat4> T_copy;

    for (int i = 0; i < len; i++) {
        // look at the top of the stack
        glm::mat4 top = T.top();
        // aggregate the transform
        M = top * M;
        // push a copy to the copy stack
        T_copy.push(top);
        // pop the element off the stack
        T.pop();
    }

    // place elements back in the stack
    for (int i = 0; i < len; i++) {
        glm::mat4 top = T_copy.top();
        T.push(top);
        T_copy.pop();
    }

    return M;

}

