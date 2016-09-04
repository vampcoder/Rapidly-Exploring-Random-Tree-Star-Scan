#include "RRT-star-Implementation.h"


int main() {
	ifstream inp;
	inp.open("Input/obstacle.txt");
	int x1, y1, x2, y2;
	string str = "Output/obstacle_type5_input";
	int J = 1;
	while (inp >> x1 >> y1 >> x2 >> y2) {
		cout <<endl<< x1 << " " << y1 << " " << x2 << " " << y2 << endl << endl;
		if (J != 1) {
			J++;
			continue;
		}
		stringstream s;

		string str1 =".txt";
		s << str << J << str1;
		ofstream file;
		file.open(s.str());
		for (int I = 0; I < 1; I++) {

			clock_t start = clock();
			srand(start);
			workspace w = workspace();
			//w.getsourceandgoal();
			clock_t begin = clock();
			w.getSnG(x1, y1, x2, y2);
			w.start();
			clock_t end = clock() - begin;
			long double cost = w.printFinalPath();
			file << end << " " << cost << "\n";
			cout << I << " " << cost << " " << end << "\n";
			
		}


		file.close();
		
		J++;
	}
	inp.close();

}