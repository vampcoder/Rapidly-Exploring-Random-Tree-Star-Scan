#include<bits/stdc++.h>

using namespace std;

int main()
{
    for(int i = 1; i < 2; i++){
        ifstream file;
        stringstream s1, s2, s3;
        string str1 = "obstacle_type5_input";
        string str2 = ".txt";
        s1 << str1 << i << str2;
        //cout << s1.str() << endl;
        file.open(s1.str().c_str());
        double x, y;
        ofstream file1, file2;
        string str3 = "input";
        string str4 = "_type5_1.txt";
        string str5 = "_type5_2.txt";
        s2 << str3 <<i << str4;
        s3 << str3 <<i << str5;

        file1.open(s2.str().c_str() , ios_base::app);
        file2.open(s3.str().c_str() , ios_base::app);
        while(file >> x >> y){
            file1 << x << endl;
            file2 << y << endl;
        }
        file.close();
        file1.close();
        file2.close();
    }
}
