#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>


int main(int argc, char const *argv[])
{
    cv::Mat img = cv::imread("/Users/yujingrui/Downloads/tst/13.png");
    int a = 9;
    std::cout << "img type is: " << img.type() << std::endl;
    std::cout << "img size is: " << img.size() << std::endl;
    std::cout << "hello world" << std::endl;

    cv::Mat img_gray;
    cv::cvtColor(img,img_gray,CV_RGB2GRAY);

    cv::imshow("123",img_gray);

    cv::Mat imc = img_gray.clone();
    std::cout << "img_gray type is: " << img_gray.type() << std::endl;
    std::cout << "img_gray size is: " << img_gray.size() << std::endl;

    for (int r = 0; r < img_gray.rows; r++)
    {
        for (int c = 0; c < img_gray.cols; c++)
        {   
            if (std::abs(r-640) <= 20 && std::abs(c-380) <= 20)
            {
                imc.at<uchar>(r,c) = 1.2 * imc.at<uchar>(r,c) + 10;
            }
        }
    }
    
    cv::imshow("12",imc);
    cv::waitKey(0);

    return 0;
}



