#include <GL/glew.h>

#include <vector>
#include <glm/glm.hpp>

#include <oculus/transform_stack.h>

namespace simple_shape {
    
    static const GLfloat test_triangle[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
    };

    static const GLfloat pyramid_vertices[] = {
        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, 1.0f,
        -1.0f, -1.0f, 1.0f,
        1.0f, -1.0f, 1.0f,
        1.0f, -1.0f, 1.0f,
        1.0f, -1.0f, -1.0f,
        1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
        0.0f, 1.0f, 0.0f,
        -1.0f, -1.0f, 1.0f,
        0.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 1.0f,
        0.0f, 1.0f, 0.0f,
        1.0f, -1.0f, -1.0f,
        0.0f, 1.0f, 0.0f,
    };

    class SimpleShape {
        public:
            SimpleShape();
            ~SimpleShape();
            void draw();
        protected:
            std::vector<GLfloat> vertices;
            GLuint vertex_buffer;
    };

    class Pyramid : public SimpleShape {
        public:
            Pyramid();
        private:
            int width;
            int height;

    };
};
