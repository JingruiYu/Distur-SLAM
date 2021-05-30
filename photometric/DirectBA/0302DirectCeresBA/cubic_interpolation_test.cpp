#include <ceres/cubic_interpolation.h>
#include <memory>
#include <iostream>

using namespace std;
using namespace ceres;

void test01();
void test02();
void test03();
void test04();
void test05();
void test06();
void test07();
void test08();
void test09();
void test10();

int main(int argc, char **argv)
{
    cout << argv[1] << endl;
    int i;
    i = atoi(argv[1]);
    
    switch(i)
    {
        case 1:
            test01();
            break;
        case 2:
            test02();
            break;
        case 3:
            test03();
            break;
        case 4:
            test04();
            break;
        case 5:
            test05();
            break;
        case 6:
            test06();
            break;
        case 7:
            test07();
            break;
        case 8:
            test08();
            break;
        case 9:
            test09();
            break;
        case 10:
            test10();
            break;
    }
    
    return 0;
}

void test01()
{
    int x[] = {1, 2, 3};
    Grid1D<int, 1> grid(x, 0, 3);
    for (int i = 0; i < 3; ++i) {
        double value;
        grid.GetValue(i, &value);
        cout << i << ": " << value << endl;
    }
}

void test02()
{
    int x[] = {1, 2, 3};
    Grid1D<int, 1> grid(x, 0, 3);
    double value;
    grid.GetValue(-1, &value);
    cout << "-1: " << value << endl;
    grid.GetValue(-2, &value);
    cout << "-2: " << value << endl;
    grid.GetValue(3, &value);
    cout << "3: " << value << endl;
    grid.GetValue(4, &value);
    cout << "4: " << value << endl;
}

void test03()
{
    int x[] = {1, 5,
                2, 6,
                3, 7};

    Grid1D<int, 2, true> grid(x, 0, 3);
    for (int i = 0; i < 3; ++i)
    {
        double value[2];
        grid.GetValue(i, value);
        cout << i << ": " << value[0] << ", " << value[1] << endl;
    }
}


void test04()
{
    int x[] = {1, 2, 3,
                5, 6, 7};

    Grid1D<int, 2, false> grid(x, 0, 3);
    for (int i = 0; i < 3; ++i)
    {
        double value[2];
        grid.GetValue(i, value);
        cout << i << ": " << value[0] << ", " << value[1] << endl;
    }
}

void test05()
{
    int x[] = {1, 2, 3,
                2, 3, 4};
    Grid2D<int, 1, true, true> grid(x, 0, 2, 0, 3);
    for (int r = 0; r < 2; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double value;
            grid.GetValue(r, c, &value);
            cout << r << "," << c << ": " << value << endl;
        }
    }
}

void test06()
{
    int x[] = {1, 2, 3,
                2, 3, 4};
    Grid2D<int, 1, true, true> grid(x, 0, 2, 0, 3);
    double value;
    grid.GetValue(-1, -1, &value);
    cout << "-1,-1: " << value << endl;
    grid.GetValue(-1, 0, &value);
    cout << "-1,0: " << value << endl;
    grid.GetValue(-1, 1, &value);
    cout << "-1,1: " << value << endl;
    grid.GetValue(-1, 2, &value);
    cout << "-1,2: " << value << endl;
    grid.GetValue(-1, 3, &value);
    cout << "-1,3: " << value << endl;
    grid.GetValue(0, 3, &value);
    cout << "0,3: " << value << endl;
    grid.GetValue(1, 3, &value);
    cout << "1,3: " << value << endl;
    grid.GetValue(2, 3, &value);
    cout << "2,3: " << value << endl;
    grid.GetValue(2, 2, &value);
    cout << "2,2: " << value << endl;
    grid.GetValue(2, 1, &value);
    cout << "2,1: " << value << endl;
    grid.GetValue(2, 0, &value);
    cout << "2,0: " << value << endl;
    grid.GetValue(2, -1, &value);
    cout << "2,-1: " << value << endl;
    grid.GetValue(1, -1, &value);
    cout << "1,-1: " << value << endl;
    grid.GetValue(0, -1, &value);
    cout << "0,-1: " << value << endl;
}

void test07()
{
    int x[] = {1, 4, 2, 8, 3, 12,
                2, 8, 3, 12, 4, 16};
    Grid2D<int, 2, true, true> grid(x, 0, 2, 0, 3);
    for (int r = 0; r < 2; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double value[2];
            grid.GetValue(r, c, value);
            cout << r << "," << c << ": " << value[0] << ", " << value[1] << endl;
        }
    }
}

void test08()
{
    int x[] = {1,  2,  3,
                2,  3,  4,
                4,  8, 12,
                8, 12, 16};
    Grid2D<int, 2, true, false> grid(x, 0, 2, 0, 3);
    for (int r = 0; r < 2; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double value[2];
            grid.GetValue(r, c, value);
            cout << r << "," << c << ": " << value[0] << ", " << value[1] << endl;
        }
    }
}

void test09()
{
    int x[] = { 1,  4, 2,  8,
                2,  8, 3, 12,
                3, 12, 4, 16};
    Grid2D<int, 2, false, true> grid(x, 0, 2, 0, 3);
    for (int r = 0; r < 2; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double value[2];
            grid.GetValue(r, c, value);
            cout << r << "," << c << ": " << value[0] << ", " << value[1] << endl;
        }
    }
}

void test10()
{
    int x[] = {1,   2,
                2,   3,
                3,   4,
                4,   8,
                8,  12,
                12, 16};
    Grid2D<int, 2, false, false> grid(x, 0, 2, 0, 3);
    
    for (int r = 0; r < 2; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double value[2];
            grid.GetValue(r, c, value);
            cout << r << "," << c << ": " << value[0] << ", " << value[1] << endl;
        }
    }
}
