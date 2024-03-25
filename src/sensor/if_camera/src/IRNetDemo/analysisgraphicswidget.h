#ifndef ANALYSISGRAPHICSWIDGET_H
#define ANALYSISGRAPHICSWIDGET_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMap>
#include "common.h"

class AnalysisGraphicsItem;

class AnalysisGraphicsWidget : public QGraphicsView
{
    Q_OBJECT
public:
    explicit AnalysisGraphicsWidget(QGraphicsView *parent = 0);
    ~AnalysisGraphicsWidget();

    void resizeEvent(QResizeEvent *event);

    void AddPointAnalysis(QList<QPoint> points,int tag);
    void AddLineAnalysis(QList<QPoint> points,int tag);
    void AddRectAnalysis(QList<QPoint> points,int tag);
    void AddCireAnalysis(QList<QPoint> points,int tag);
    void AddPolyAnalysis(QList<QPoint> points,int tag);

    void ClearAnalysis();
    void RemoveAnalysis(QString key);

    void SetFollowMaxTemp(bool is_follow);
    void SetFollowMinTemp(bool is_follow);

    void ShowMachineVideo(bool is_show);

    void GetAnalysisInfo(QList<AnalysisInfo> &list);

    void UpdateTempInfo(QList<TempInfo> list);



signals:


public slots:
    void graphics_scene_itemchange();

    void onAnalysisItemSelect(AnalysisGraphicsItem* item);


private:
     QGraphicsScene *graphics_scene;
     AnalysisGraphicsItem* m_select_item = nullptr;

     QMap<QString,AnalysisGraphicsItem*> analysis_map;
     int point_number = 0;
     int line_number = 0;
     int rect_number = 0;
     int cire_number = 0;
     int poly_number = 0;



      void AddAnalysis(AnalysisType type,QList<QPoint> points,int tag);

};

#endif // ANALYSISGRAPHICSWIDGET_H
