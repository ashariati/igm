#include <oculus/simple_shape.h>

namespace simple_shape {

    SimpleShape::SimpleShape() {
        glGenBuffers(1, &vertex_buffer);
        glGenBuffers(1, &color_buffer);
    }

    SimpleShape::~SimpleShape() {
        glDeleteBuffers(1, &vertex_buffer);
        glDeleteBuffers(1, &color_buffer);
    }

    void SimpleShape::draw() { 
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINES, 0, vertex_buffer_data.size() * sizeof(glm::vec3));
        glDisableVertexAttribArray(0);
    }

    void SimpleShape::bufferData() {
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glBufferData(
                GL_ARRAY_BUFFER, 
                vertex_buffer_data.size() * sizeof(glm::vec3),
                &vertex_buffer_data[0],
                GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, color_buffer);
        glBufferData(
                GL_ARRAY_BUFFER, 
                color_buffer_data.size() * sizeof(glm::vec3),
                &color_buffer_data[0],
                GL_STATIC_DRAW);
    }

    Pyramid::Pyramid(float x, float z) {

        float d_x = x / 2;

        model_vertices.push_back(glm::vec3(d_x, d_x, 0.0f));
        model_vertices.push_back(glm::vec3(d_x, -d_x, 0.0f));
        model_vertices.push_back(glm::vec3(-d_x, -d_x, 0.0f));
        model_vertices.push_back(glm::vec3(-d_x, d_x, 0.0f));
        model_vertices.push_back(glm::vec3(0.0f, 0.0f, z));

        vertex_buffer_data.push_back(model_vertices[0]);
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[2]);
        vertex_buffer_data.push_back(model_vertices[2]);
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[0]);
        vertex_buffer_data.push_back(model_vertices[0]);
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[2]);
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[4]);

        bufferData();

    }

    Box::Box(float x, float y, float z) {

        float d_x = x / 2;
        float d_y = y / 2;
        float d_z = z / 2;

        model_vertices.push_back(glm::vec3(d_x, d_y, -d_z));
        model_vertices.push_back(glm::vec3(d_x, -d_y, -d_z));
        model_vertices.push_back(glm::vec3(-d_x, -d_y, -d_z));
        model_vertices.push_back(glm::vec3(-d_x, d_y, -d_z));
        model_vertices.push_back(glm::vec3(d_x, d_y, d_z));
        model_vertices.push_back(glm::vec3(d_x, -d_y, d_z));
        model_vertices.push_back(glm::vec3(-d_x, -d_y, d_z));
        model_vertices.push_back(glm::vec3(-d_x, d_y, d_z));

        vertex_buffer_data.push_back(model_vertices[0]); // 1
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[5]);
        vertex_buffer_data.push_back(model_vertices[2]); // 2
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[5]);
        vertex_buffer_data.push_back(model_vertices[2]); // 3
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[7]);
        vertex_buffer_data.push_back(model_vertices[0]); // 4
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[7]);
        vertex_buffer_data.push_back(model_vertices[0]); // 5
        vertex_buffer_data.push_back(model_vertices[1]);
        vertex_buffer_data.push_back(model_vertices[2]);
        vertex_buffer_data.push_back(model_vertices[2]); // 6
        vertex_buffer_data.push_back(model_vertices[3]);
        vertex_buffer_data.push_back(model_vertices[0]);
        vertex_buffer_data.push_back(model_vertices[6]); // 7
        vertex_buffer_data.push_back(model_vertices[5]);
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[4]); // 8
        vertex_buffer_data.push_back(model_vertices[7]);
        vertex_buffer_data.push_back(model_vertices[6]);
        vertex_buffer_data.push_back(model_vertices[2]); // 9
        vertex_buffer_data.push_back(model_vertices[6]);
        vertex_buffer_data.push_back(model_vertices[5]);
        vertex_buffer_data.push_back(model_vertices[0]); // 10
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[7]);
        vertex_buffer_data.push_back(model_vertices[2]); // 11
        vertex_buffer_data.push_back(model_vertices[6]);
        vertex_buffer_data.push_back(model_vertices[7]);
        vertex_buffer_data.push_back(model_vertices[0]); // 12
        vertex_buffer_data.push_back(model_vertices[4]);
        vertex_buffer_data.push_back(model_vertices[5]);

        for (int i = 0; i < vertex_buffer_data.size(); i++) {
            glm::vec3 v = vertex_buffer_data[i] + glm::vec3(d_x, d_y, d_z);
            color_buffer_data.push_back(glm::normalize(v));
        }

        bufferData();

    }

    void Box::draw() {
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, color_buffer);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glDrawArrays(GL_TRIANGLES, 0, vertex_buffer_data.size());
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }

    namespace sphere {

        Sphere::Sphere(int ndiv, float r) {
            for (int i = 0; i < 20; i++) {
                drawtri(
                        vdata[tindices[i][0]], 
                        vdata[tindices[i][1]], 
                        vdata[tindices[i][2]],
                        ndiv, 
                        r
                       );
            }

            for (int i = 0; i < vertex_buffer_data.size(); i++) {
                glm::vec3 v = vertex_buffer_data[i] + glm::vec3(r / 2);
                color_buffer_data.push_back(glm::normalize(v));
            }

            bufferData();
        }

        void Sphere::normalize(GLfloat *a) {
            GLfloat d = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            a[0] /= d; a[1] /= d; a[2] /= d;
        }

        void Sphere::drawtri(GLfloat *a, GLfloat *b, GLfloat *c, int div, float r) {
            if (div <= 0) {
                vertex_buffer_data.push_back(glm::vec3(a[0]*r, a[1]*r, a[2]*r));
                vertex_buffer_data.push_back(glm::vec3(b[0]*r, b[1]*r, b[2]*r));
                vertex_buffer_data.push_back(glm::vec3(c[0]*r, c[1]*r, c[2]*r));
            } else {
                GLfloat ab[3], ac[3], bc[3];
                for (int i = 0; i < 3; i++) {
                    ab[i] = (a[i] + b[i]) / 2;
                    ac[i] = (a[i] + c[i]) / 2;
                    bc[i] = (b[i] + c[i]) / 2;
                }
                normalize(ab); normalize(ac); normalize(bc);
                drawtri(a, ab, ac, div-1, r);
                drawtri(b, bc, ab, div-1, r);
                drawtri(c, ac, bc, div-1, r);
                drawtri(ab, bc, ac, div-1, r);
            }
        }

    void Sphere::draw() {
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, color_buffer);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glDrawArrays(GL_TRIANGLES, 0, vertex_buffer_data.size());
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }

    }

};
