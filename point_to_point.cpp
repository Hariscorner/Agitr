#include <iomanip>
#include <iostream>

int main()
{
	double x_goal;
	double y_goal;
	
	std::cout<<"SETTING GOAL POSITION"<<std::endl;
	std::cout<<"Please enter the x cordinate (hint: a value between 0.5 to 10.5): ";
	std::cin>>x_goal;
	std::cout<<"Please enter the y cordinate (hint: a value between 0.5 to 10.5): ";
	std::cin>>y_goal;	
	
	std::cout<<"The goal position is set to ("<< x_goal << "," << y_goal << ")."<<std::endl;

}