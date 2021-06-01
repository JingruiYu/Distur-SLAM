#include "line.h"

bool line::CalculateMajorLine(const Frame* pF)
{
    const float th = cos(M_PI / 4);
    const float th2 = cos(M_PI / 10);
    const float thDist = 0.05;
    const float cx = 0.5 * pF->cols;
    const float cy = 0.5 * pF->rows;

    cv::Mat imageRaw = pF->img.clone();
    
    // // extract lsd lines
    LineExtractorPtr mpLineExtractor = std::make_shared<LineExtractor>();
    std::vector<KeyLine> vKeyLines;
    mpLineExtractor->extractLines(imageRaw, vKeyLines);
    std::cout << "vKeyLines: " << vKeyLines.size() << std::endl;

    return true;
}