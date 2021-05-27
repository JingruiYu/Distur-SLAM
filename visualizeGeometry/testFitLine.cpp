#include <iostream>
#include <vector>

int main ( int argc, char** argv )
{
    std::vector<float> data_x = {1,2,3,4,5,6,7,8,9,10};
    std::vector<float> data_y = {45,67,79,98,106,128,145,162,179,196};

    float A = 0.0;
    float B = 0.0;
    float C = 0.0;
    float D = 0.0;
    float E = 0.0;
    float F = 0.0;
    float data_n = 10.0;

    for (size_t i = 0; i < data_x.size(); i++)
    {
        A += data_x[i] * data_x[i];
        B += data_x[i];
        C += data_x[i] * data_y[i];
        D += data_y[i];
    }
    
    float a, b, temp = 0;
    if (temp = (data_n*A - B*B))
    {
        a = (data_n*C - B*D) / temp;
        b = (A*D - B*C) / temp;
    }
    else
    {
        a = 1;
        b = 0;
    }

    std::cout << "a: " << a << ", b: " << b << std::endl;
    
}
