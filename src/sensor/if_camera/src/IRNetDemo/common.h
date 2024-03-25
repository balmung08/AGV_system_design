#ifndef COMMON_H
#define COMMON_H

#include <QList>
#include <QPoint>

enum AnalysisType{
    Point,
    Line,
    Rect,
    Cire,
    Poly,
    Max,
    Min
};

struct AnalysisInfo{

    AnalysisType analysisType;
    QList<QPoint> points;
    int tag;


};

struct TempInfo{

    double max_temp;
    double min_temp;
    double avg_temp;
    QPoint max_point = QPoint(200,200);
    QPoint min_point = QPoint(100,200);
    int tag;
};



#endif // COMMON_H
