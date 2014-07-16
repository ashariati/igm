#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <GL/glew.h>

#include <glfw3.h>
GLFWwindow* window;

#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>

#include <stdlib.h>

#include <common/shader.hpp>
#include <common/controls.hpp>
#include <common/objloader.hpp>
#include <common/texture.hpp>
#include <common/vboindexer.hpp>

using namespace glm;

int main(){
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize glfw!\n");
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(1024, 768, "BEAR PLAYGROUND", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "Failed to open GLFW window!\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glewExperimental=true;
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize glew!\n");
        return -1;
    }

    // Set GL State
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
	glEnable(GL_CULL_FACE);
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // Vertex Array Object (VAO)
    GLuint vertex_array_id;
    glGenVertexArrays(1, &vertex_array_id);
    glBindVertexArray(vertex_array_id);


    GLuint program_id = LoadShaders("StandardShading.vertexshader",
            "StandardShading.fragmentshader");

    // load texture
    GLuint texture = loadDDS("uvmap.DDS");



    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;

    std::vector<unsigned short> indices;

    // bool res = loadOBJ("suzanne.obj", vertices, uvs, normals);
    bool res = loadAssImp("suzanne.obj", indices, vertices, uvs, normals);

    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, 
            vertices.size() * sizeof(glm::vec3),
            &vertices[0],
            GL_STATIC_DRAW);

    GLuint element_buffer;
    glGenBuffers(1, &element_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer);
    glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            indices.size() * sizeof(unsigned int),
            &indices[0],
            GL_STATIC_DRAW
            );

    GLuint uv_buffer;
    glGenBuffers(1, &uv_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, uv_buffer);
    glBufferData(GL_ARRAY_BUFFER, 
            uvs.size() * sizeof(glm::vec2),
            &uvs[0],
            GL_STATIC_DRAW);

    GLuint normals_buffer;
    glGenBuffers(1, &normals_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
    glBufferData(GL_ARRAY_BUFFER,
            normals.size() * sizeof(glm::vec3),
            &normals[0],
            GL_STATIC_DRAW);


    glUseProgram(program_id);

    // get uniform handle
    GLuint mvp_id = glGetUniformLocation(program_id, "MVP");
    GLuint view_matrix_id = glGetUniformLocation(program_id, "V");
    GLuint model_matrix_id = glGetUniformLocation(program_id, "M");
    GLuint texture_id = glGetUniformLocation(program_id, "myTextureSampler");
    GLuint light_id = glGetUniformLocation(program_id, "LightPosition_worldspace");

    glm::mat4 projection_matrix = glm::perspective(
            45.0f,
            4.0f / 3.0f, 
            0.1f,
            100.0f);

    glm::mat4 view_matrix = glm::lookAt(
            glm::vec3(4, 3, 3),
            glm::vec3(0, 0, 0),
            glm::vec3(0, 1, 0));


    glm::mat4 model_matrix = glm::mat4(1.0);
    glm::mat4 mvp = projection_matrix * view_matrix * model_matrix;

    

    double last_time = glfwGetTime();
    double last_frame_time = last_time;
    int nb_frames = 0;

    vec3 orient;
    double y_rot = 0.0f;
    orient.y = y_rot;

    vec4 light_pos;
    light_pos.x = 4.0f;
    light_pos.y = 4.0f;
    light_pos.z = 4.0f;
    light_pos.w = 1.0f;

    do {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program_id);

        double current_time = glfwGetTime();
        float delta_time = (float)(current_time - last_frame_time);
        last_frame_time = current_time;
        nb_frames++;

        if (current_time - last_time >= 1.0) {
            nb_frames = 0;
            last_time += 1.0;
        }
        
        y_rot += 3.14159f/2.0f * delta_time;
        orient.y = y_rot;
        
        glm::mat4 rot_mat = eulerAngleYXZ(orient.y, orient.x, orient.z);

        // keyboard input
        // computeMatricesFromInputs();
        // glm::mat4 projection_matrix = getProjectionMatrix();
        // glm::mat4 view_matrix = getViewMatrix();
        glm::mat4 model_matrix = glm::mat4(1.0);
        glm::mat4 mvp = projection_matrix * view_matrix * model_matrix;
        // glm::mat4 mvp = projection_matrix * view_matrix * rot_mat * model_matrix;


        // send transform to currently bound shader
        glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &mvp[0][0]);
        glUniformMatrix4fv(model_matrix_id, 1, GL_FALSE, &model_matrix[0][0]);
        glUniformMatrix4fv(view_matrix_id, 1, GL_FALSE, &view_matrix[0][0]);

        // GlUniform3f(light_id, 
        //         light_pos.x, 
        //         light_pos.y, 
        //         light_pos.z);

        mat4 light_rot_mat = eulerAngleYXZ(-1*orient.y, orient.x, orient.z); 
        // vec4 light_pos_trans = light_rot_mat * light_pos;
        vec4 light_pos_trans = light_pos;
        glUniform3f(light_id, 
                light_pos_trans.x, 
                light_pos_trans.y, 
                light_pos_trans.z);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glUniform1i(texture_id, 0);

        // primary attribute
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // secondary attribute
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uv_buffer);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // tertiary attribute
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // glDrawArrays(GL_TRIANGLES, 0, vertices.size());
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer);

        glDrawElements(
                GL_TRIANGLES,
                indices.size(),
                GL_UNSIGNED_SHORT,
                (void*)0
                );


        ///////////// Second Object ///////////////

        glm::mat4 model_matrix2 = glm::mat4(1.0);
        model_matrix2 = glm::translate(
                model_matrix2, 
                glm::vec3(2.0f, 0.0f, 0.0f)
                );
        glm::mat4 mvp2 = projection_matrix * view_matrix * model_matrix2;

        glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &mvp2[0][0]);
        glUniformMatrix4fv(model_matrix_id, 1, GL_FALSE, &model_matrix2[0][0]);


        // primary attribute
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // secondary attribute
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uv_buffer);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // tertiary attribute
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // glDrawArrays(GL_TRIANGLES, 0, vertices.size());
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer);

        glDrawElements(
                GL_TRIANGLES,
                indices.size(),
                GL_UNSIGNED_SHORT,
                (void*)0
                );

        ///////////////////////////////////////////

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);


        glfwSwapBuffers(window);
        glfwPollEvents();

    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && \
            glfwWindowShouldClose(window) == 0);

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertex_buffer);
	glDeleteBuffers(1, &uv_buffer);
	glDeleteBuffers(1, &normals_buffer);
	glDeleteProgram(program_id);
	glDeleteTextures(1, &texture);
	glDeleteVertexArrays(1, &vertex_array_id);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

    return 0;

}
