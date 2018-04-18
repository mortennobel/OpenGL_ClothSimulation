#define _USE_MATH_DEFINES
#include <iostream>
#ifdef __WIN32
#   include <GL/glew.h>
#   include <GL/gl.h>
#else
#   include <OpenGL/gl3.h>
#   include <glut.h>
#endif
#include "cloth_1_5_glm/cloth_1_5_glm.h"
#include "cloth_1_5_starting_point/cloth_1_5_starting_point.h"
#include "cloth_2_1_shaders/cloth_2_1_shaders.h"
#include "cloth_3_2_core_profile/cloth_3_2_core_profile.h"
#include "cloth_3_2_triangle_strips/cloth_3_2_triangle_strips.h"
using namespace std;


int main(int argc, char* argv[]) {
	cout << "Cloth simulation:" << endl;
	cout << "0. OpenGL 1.5 (Fixed function pipeline)" << endl;
	cout << "1. OpenGL 1.5 (Fixed function pipeline + GLM)" << endl;
	cout << "2. OpenGL 2.1 (Shaders)" << endl;
	cout << "3. OpenGL 3.2 (Core profile)" << endl;
	cout << "4. OpenGL 3.2 (Triangle strips)" << endl;
	int menuChoice;
	cin >> menuChoice;
	switch (menuChoice) {
	case 0:
		cloth_1_5_starting_point::main(argc, argv);
		break;
	case 1:
		cloth_1_5_glm::main(argc, argv);
		break;
	case 2:
		cloth_2_1_shaders::main(argc, argv);
		break;
	case 3:
		cloth_3_2_core_profile::main(argc, argv);
		break;
	case 4:
		cloth_3_2_triangle_strips::main(argc, argv);
		break;
	default:
		cout << "Invalid option" << endl;
		break;
	}
}