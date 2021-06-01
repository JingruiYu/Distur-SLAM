#include "line.h"

bool line::CalculateMajorLine(const Frame* pF)
{
    const float th = cos(M_PI / 4);
    const float th2 = cos(M_PI / 10);
    const float thDist = 0.05;
    const float cx = 0.5 * pF->cols;
    const float cy = 0.5 * pF->rows;

    const cv::Mat& imageRaw = pF->img.clone();

    LSDDetector::LSDOptions opts;
    opts.min_length = 0.025 * std::max(pF->rows, pF->cols);

    std::vector<KeyLine> keylines_;
    std::vector<KeyLine> keylines;

    cv::Ptr<LSDDetector> lsd = LSDDetector::createLSDDetectorC();

    lsd->detect(imageRaw,keylines_,1,1,opts);

    keylines.clear();
    for(auto & keyline : keylines_)
    {
        if( keyline.octave == 0 && keyline.lineLength >= 20)
        {
            keylines.push_back(keyline);
        }
    }

    // // extract lsd lines
    // LineExtractorPtr mpLineExtractor = std::make_shared<LineExtractor>();
    // LineExtractor* mpLineExtractor = new LineExtractor();
    // std::vector<KeyLine> vKeyLines;
    // mpLineExtractor->extractLines(imageRaw, vKeyLines);

    return true;
}