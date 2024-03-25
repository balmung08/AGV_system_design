#include "analysisgraphicswidget.h"
#include "analysisgraphicsitem.h"
#include <QDebug>

#define KEY_MAX_TEMP "max"
#define KEY_MIN_TEMP "min"

AnalysisGraphicsWidget::AnalysisGraphicsWidget(QGraphicsView *parent) : QGraphicsView(parent)
{
    graphics_scene = new QGraphicsScene();
    connect(graphics_scene,SIGNAL(selectionChanged()),this,SLOT(graphics_scene_itemchange()));
    graphics_scene->setSceneRect(0,0,width(),height());

    setStyleSheet("background: transparent");


    setScene(graphics_scene);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);


}

AnalysisGraphicsWidget::~AnalysisGraphicsWidget(){
   ClearAnalysis();
}




void AnalysisGraphicsWidget::SetFollowMaxTemp(bool is_follow){
    if(is_follow && !analysis_map.contains(KEY_MAX_TEMP)){
       AddAnalysis(AnalysisType::Max,QList<QPoint>(),-1);
    }else if(!is_follow && analysis_map.contains(KEY_MAX_TEMP)){
       RemoveAnalysis(KEY_MAX_TEMP);
    }
}

void AnalysisGraphicsWidget::SetFollowMinTemp(bool is_follow){
    if(is_follow && !analysis_map.contains(KEY_MIN_TEMP)){
       AddAnalysis(AnalysisType::Min,QList<QPoint>(),-2);
    }else if(!is_follow && analysis_map.contains(KEY_MIN_TEMP)){
       RemoveAnalysis(KEY_MIN_TEMP);
    }
}

void AnalysisGraphicsWidget::ShowMachineVideo(bool is_show){

}



void AnalysisGraphicsWidget::GetAnalysisInfo(QList<AnalysisInfo> &list){

    foreach(AnalysisGraphicsItem* item,analysis_map.values()){
        AnalysisInfo info;
        item->GetPoints(info.points);
        info.tag = item->GetTag();
        info.analysisType = item->GetAnalysisType();
        list.append(info);
    }

}

void AnalysisGraphicsWidget::UpdateTempInfo(QList<TempInfo> list){

    foreach (TempInfo info, list) {

        foreach (AnalysisGraphicsItem *item, analysis_map.values()) {
            if(item->GetTag() == info.tag){
                item->UpdateTempInfo(info);
                break;
            }
        }
    }
    viewport()->update();
}


void AnalysisGraphicsWidget::AddPointAnalysis(QList<QPoint> points,int tag){

    AddAnalysis(AnalysisType::Point,points,tag);

}

void AnalysisGraphicsWidget::AddLineAnalysis(QList<QPoint> points,int tag){

    AddAnalysis(AnalysisType::Line,points,tag);

}

void AnalysisGraphicsWidget::AddRectAnalysis(QList<QPoint> points,int tag){

    AddAnalysis(AnalysisType::Rect,points,tag);
}

void AnalysisGraphicsWidget::AddCireAnalysis(QList<QPoint> points,int tag){

    AddAnalysis(AnalysisType::Cire,points,tag);
}

void AnalysisGraphicsWidget::AddPolyAnalysis(QList<QPoint> points, int tag)
{
   AddAnalysis(AnalysisType::Poly,points,tag);
}

void AnalysisGraphicsWidget::AddAnalysis(AnalysisType type, QList<QPoint> points,int tag){

    QString title;

    switch (type) {
    case Point:
    {
        point_number++;
        title = QString("Po%1").arg(point_number);
    }
        break;
    case Line:
    {
        line_number++;
        title = QString("Li%1").arg(line_number);
    }
        break;
    case Rect:
    {
        rect_number++;
        title = QString("Re%1").arg(rect_number);
    }
        break;
    case Cire:
    {
        cire_number++;
        title = QString("Ci%1").arg(cire_number);
    }
        break;
    case Poly:
    {
        poly_number++;
        title = QString("Pl%1").arg(poly_number);
    }
        break;
    case Max:
        title = KEY_MAX_TEMP;
        break;
    case Min:
        title = KEY_MIN_TEMP;
        break;
    default:break;
    }

    AnalysisGraphicsItem *item = new AnalysisGraphicsItem(type,points);
    connect(item,SIGNAL(sgnAnalysisItemSelect(AnalysisGraphicsItem*)),this,SLOT(onAnalysisItemSelect(AnalysisGraphicsItem*)));
    item->SetViewSize(QSize(width(),height()));
    item->SetTitle(title);
    item->SetTag(tag);
    analysis_map.insert(title,item);
    graphics_scene->addItem(item);

}

void AnalysisGraphicsWidget::RemoveAnalysis(QString key){
    if(analysis_map.contains(key)){
         AnalysisGraphicsItem* item = analysis_map.value(key);
         graphics_scene->removeItem(item);
         delete item;
         analysis_map.remove(key);
    }
}

void AnalysisGraphicsWidget::ClearAnalysis(){
    m_select_item = nullptr;
    foreach(AnalysisGraphicsItem* item, analysis_map.values()){
        graphics_scene->removeItem(item);
        delete item;
    }
    analysis_map.clear();

}


void AnalysisGraphicsWidget::onAnalysisItemSelect(AnalysisGraphicsItem* item){

    if(m_select_item){
        m_select_item->setSelected(false);
    }

    m_select_item = item;

}

void AnalysisGraphicsWidget::graphics_scene_itemchange(){

    qDebug()<<"graphics_scene_itemchange";

}

void AnalysisGraphicsWidget::resizeEvent(QResizeEvent *event){


   int w = width();
   int h = height();
    graphics_scene->setSceneRect(0,0,w-1,h-1);
    QList<QGraphicsItem *> items = graphics_scene->items();
    foreach(QGraphicsItem* item,items){
        AnalysisGraphicsItem* analysis = (AnalysisGraphicsItem*)item;
        analysis->SetViewSize(QSize(width(),height()));
    }
    qDebug()<<"resizeEvent";

}
