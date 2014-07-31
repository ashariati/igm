#ifndef SIMPLE_SHAPE
#define SIMPLE_SHAPE

#include <GL/glew.h>

#include <math.h>
#include <vector>
#include <glm/glm.hpp>

// Sphere macros
#define X .525731112119133606
#define Z .850650808352039932

namespace simple_shape {

    class SimpleShape {
        public:
            SimpleShape();
            ~SimpleShape();
            virtual void draw();
        protected:
            std::vector<glm::vec3> model_vertices;
            std::vector<glm::vec3> vertex_buffer_data;
            std::vector<glm::vec3> color_buffer_data;
            GLuint vertex_buffer;
            GLuint color_buffer;

            void bufferData();
    };

    class Plane : public SimpleShape {
        public:
            Plane(float x=1000.0f, float y=1000.0f, int res=100);
    };

    class Pyramid : public SimpleShape {
        public:
            Pyramid(float x=1.0f, float z=1.0f);

    };

    class Box : public SimpleShape {
        public:
            Box(float x=1.0f, float y=1.0f, float z=1.0f);
            void draw();
    };

    namespace sphere {

        static GLfloat vdata[12][3] = {
            {-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z},
            {0.0, Z, X}, {0.0, Z, -X}, {0.0, -Z, X}, {0.0,- Z, -X},
            {Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0}
        };

        static GLuint tindices[20][3] = {
            {0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},
            {8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},
            {7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6},
            {6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11}
        };


        class Sphere : public SimpleShape {
            public:
                Sphere(int ndiv, float r=1.0);
                void draw();
            private:

                void drawtri(
                        GLfloat *a, GLfloat *b, GLfloat *c, 
                        int div, float r
                        );

                void normalize(GLfloat *a);
        };
    };

};

#endif
