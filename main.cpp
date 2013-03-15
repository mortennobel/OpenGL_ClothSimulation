#include <iostream>

using namespace std;

int main_1_5_starting_point ( int argc, char** argv );
int main_1_5_glm(int &argc, char** argv);
int main_2_1_shaders(int &argc, char** argv);
int main_3_2_core_profile(int &argc, char** argv);
int main_3_2_ping_pong(int &argc, char** argv);

int main(int argc, char* argv[]) {
	cout << "Cloth simulation:" << endl;
	cout << "0. OpenGL 1.5 (Fixed function pipeline)" << endl;
	cout << "1. OpenGL 1.5 (Fixed function pipeline + GLM)" << endl;
	cout << "2. OpenGL 2.1 (Shaders)" << endl;
	cout << "3. OpenGL 3.2 (Core profile)" << endl;
	cout << "4. OpenGL 3.2 (Ping pong textures)" << endl;
	int menuChoice;
	cin >> menuChoice;
	switch (menuChoice) {
	case 0:
		main_1_5_starting_point(argc, argv);
		break;
	case 1:
		main_1_5_glm(argc, argv);
		break;
	case 2:
		main_2_1_shaders(argc, argv);
		break;
	case 3:
		main_3_2_core_profile(argc, argv);
		break;
	case 4:
		main_3_2_ping_pong(argc, argv);
	default:
		cout << "Invalid option" << endl;
		break;
	}
}