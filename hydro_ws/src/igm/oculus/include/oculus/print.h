#ifndef MAT_PRINTER
#define MAT_PRINTER

#include <glm/glm.hpp>
#include <iostream>

inline std::ostream& operator << (std::ostream& os, const glm::vec3 &v) {

    os << v[0] << " " << v[1] <<  " " << v[2];

}

inline std::ostream& operator << (std::ostream& os, const glm::mat4 &M) {
    // glm::mat4 is column major... M[col][row]
    os << 
        M[0][0] << " " << M[1][0] <<  " " << M[2][0] << " " << M[3][0]
        << std::endl <<
        M[0][1] << " " << M[1][1] <<  " " << M[2][1] << " " << M[3][1]
        << std::endl <<
        M[0][2] << " " << M[1][2] <<  " " << M[2][2] << " " << M[3][2]
        << std::endl <<
        M[0][3] << " " << M[1][3] <<  " " << M[2][3] << " " << M[3][3]
        << std::endl;

}

#endif
