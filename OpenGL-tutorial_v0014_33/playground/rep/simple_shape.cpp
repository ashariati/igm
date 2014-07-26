#include <playground/simple_shape.h>

namespace simple_shape {

    SimpleShape::SimpleShape() {
        glGenBuffers(1, &vertex_buffer);
    }

    SimpleShape::~SimpleShape() {
        glDeleteBuffers(1, &vertex_buffer);
    }

    void SimpleShape::draw() { 
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINES, 0, vertices.size());
        glDisableVertexAttribArray(0);
    }

    Pyramid::Pyramid() {

        for (int i = 0; i < sizeof(pyramid_vertices); i++)
            vertices.push_back(pyramid_vertices[i]);

        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glBufferData(
                GL_ARRAY_BUFFER, 
                vertices.size(),
                &vertices[0],
                GL_STATIC_DRAW);


    }

};
