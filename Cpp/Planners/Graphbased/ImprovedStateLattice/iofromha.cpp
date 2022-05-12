#include <iostream>
using namespace std;
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
//将文本文件中得数据读入vector中，并返回一个vector。
vector<double> InputData_To_Vector(string filename ="D://x.txt")
{
    //vector<double>* p = new vector<double>;
    //ifstream infile("D://x.txt");
    //double number;
    //while (!infile.eof())
    //{
    //    infile >> number;
    //    p->push_back(number);
    //    cout << "number:" << number << endl;
    //}
    //p->pop_back();  //此处要将最后一个数字弹出，是因为上述循环将最后一个数字读取了两次
    //return p;
    vector<double> p;
    ifstream infile(filename);
    double number;
    while (!infile.eof())
    {
        infile >> number;
        p.push_back(number);
        //cout << "number:" << number << endl;
    }
    p.pop_back();  //此处要将最后一个数字弹出，是因为上述循环将最后一个数字读取了两次
    return p;
}

inline int Num_Square(int n)
{
    return n * n;
}

int Sum_Of_Num_Square(vector<int>* p)
{
    int Sum2 = 0;
    vector<int>::iterator it;
    for (it = p->begin(); it != p->end(); it++)
    {
        Sum2 += Num_Square(*it);
    }
    return Sum2;
};


int loaddata()
{
    //vector<double> location_x, location_y, location_z;
    ///* Read data from point_right.csv */
    //ifstream fin("D:\\x.csv");                        // 打开文件流操作
    //string line;
    //while (getline(fin, line))                           // 整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
    //{
    //    cout << "原始字符串: " << line << endl;           // 整行输出
    //    istringstream sin(line);                         // 将整行字符串line读入到字符串流istringstream中 
    //    vector<string> Waypoints;                        // 声明一个字符串向量
    //    string info;
    //    while (getline(sin, info, ',')) {                // 将字符串流sin中的字符读入到Waypoints字符串中，以逗号为分隔符
    //        Waypoints.push_back(info);                   // 将刚刚读取的字符串添加到向量Waypoints中
    //    }
    //    // Get x,y,z of points and transform to double
    //    string x_str = Waypoints[3];
    //    string y_str = Waypoints[4];
    //    string z_str = Waypoints[5];

    //    //cout << "x= " << x << "  " << "y= " << y << "  " << "z= " << z << endl;
    //    cout << "Read data done!" << endl;
    //    // Get x,y,z of points and transform to double

    //    double x, y, z;
    //    stringstream sx, sy, sz;
    //    sx << x_str;
    //    sy << y_str;
    //    sz << z_str;
    //    sx >> x;
    //    sy >> y;
    //    sz >> z;
    //    //cout << "x= " << x << "  " << "y= " << y << "  " << "z= " << z << endl;
    //    cout << "Read data done!" << endl;

    //    location_x.push_back(x);
    //    location_y.push_back(y);
    //    location_z.push_back(z);
    //}
    return 0;
};