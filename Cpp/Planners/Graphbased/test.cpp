#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
using namespace std;
using std::vector;
class Figure
{
public:
	virtual void outprint(std::ostream &out) = 0;
	virtual void input_data(std::istream &is) = 0;
	virtual void draw(BlackBoard &board) = 0;
	//virtual Figure *create() = 0;
};
class Line:public Figure
{
	double _x1,_y1,_x2,_y2;
public:
	Line():_x1(0),_y1(0),_x2(0),_y2(0){}
	//virtual Figure *create(){return new Line;}
	void input_data(std::istream &is)
	{
		//std::cout<<"Point1_X: ";
		is>>_x1;
		//std::cout<<"Point1_Y: ";
		is>>_y1;
		//std::cout<<"Point2_X: ";
		is>>_x2;
		//std::cout<<"Point2_Y: ";
		is>>_y2;

	}
	virtual void draw(BlackBoard &board)
	{
		board.DrawLine(_x1,_y1,_x2,_y2);
	}
	virtual void outprint(std::ostream &out)
	{
		std::cout<<"Line{"
			<<"point1_X: "<<_x1<<", "
			<<"point1_Y: "<<_y1<<", "
			<<"point2_X: "<<_x2<<", "
			<<"point2_Y: "<<_y2<<"}"
			<<std::endl;
	}
};

class Rectangle:public Figure
{
	double _left,_top,_right,_bottom;
public:
	//virtual Figure *create(){return new Rectangle;}
	void input_data(std::istream &is)
	{
		//std::cout<<"Left: ";
		is>>_left;
		//std::cout<<"Top: ";
		is>>_top;
		//std::cout<<"Right: ";
		is>>_right;
		//std::cout<<"Bottom: ";
		is>>_bottom;
	}
	virtual void draw(BlackBoard &board)
	{
		board.DrawLine(_left,_top,_right,_top);
		board.DrawLine(_right,_top,_right,_bottom);
		board.DrawLine(_left,_bottom,_right,_bottom);
		board.DrawLine(_left,_top,_left,_bottom);
	}
	virtual void outprint(std::ostream &out)
	{
		std::cout<<"Rectangle{"
			<<"left: "<<_left<<", "
			<<"top: "<<_top<<", "
			<<"right: "<<_right<<", "
			<<"bottom: "<<_bottom<<"}"
			<<std::endl;
	}
};

class Circle:public Figure
{
	double _x,_y,_r;
public:
	//virtual Figure *create(){return new Circle;}
	void input_data(std::istream &is)
	{
		//std::cout<<"Center X: ";
		is>>_x;
		//std::cout<<"Center Y: ";
		is>>_y;
		//std::cout<<"Radius: ";
		is>>_r;
	}
	virtual void draw(BlackBoard &board)
	{
		board.DrawCircle(_x,_y,_r);
	}
	virtual void outprint(std::ostream &out)
	{
		std::cout<<"Circle{"
			<<"center_X: "<<_x<<", "
			<<"center_Y: "<<_y<<", "
			<<"radius: "<<_r<<"}"
			<<std::endl;
	}
};

class FigureManager
{
public:
    static FigureManager &handle()
    {
        static FigureManager manager;
        return manager;
    }
    // FigureManager类析构函数
    virtual ~FigureManager() {}
    // FigureManager类接口.
	vector<Figure *> _figures;
public:
    void input(std::istream &in);
    void display(BlackBoard &board);
    void print(std::ostream &out);
}; // class FigureManager类定义结束.


void FigureManager::input(std::istream &in)
{
	int count=0;
	for(count=0;count<100;count++)
	{
		int shape;
		do{
			//cout<<"请输入图形的种类（1：圆， 2：线（段）， 3：矩形， 0：退出）: ";
			in>>shape;
		}while(shape<0 && shape>3);
		if(shape==0) break;
		Figure *figure;
		switch(shape)
		{
		case 2:
			figure = new Line;
			break;
		case 3:
			figure = new Rectangle;
			break;
		case 1:
			figure = new Circle;
			break;
		}
		figure->input_data(in);
		_figures.push_back(figure);
	}
}

void FigureManager::display(BlackBoard &board)
{
	vector<Figure *>::iterator k;
	for(k=_figures.begin();k!=_figures.end();++k)
	{
		(*k)->draw(board);
	}
}

void FigureManager::print(std::ostream &out)
{
	vector<Figure *>::iterator k;
	for(k=_figures.begin();k!=_figures.end();++k)
	{
		(*k)->outprint(out);
	}
}
// 如果你的实现需要一些必要的初始化，可放在这个函数中，main函数会调用
// 如果没有，则忽略
void InitiateFigureManager()
{
}
