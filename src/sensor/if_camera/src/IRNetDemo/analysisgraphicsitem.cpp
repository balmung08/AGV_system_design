#include "analysisgraphicsitem.h"

#include <QPainter>
#include <QDebug>
#include <QStyleOptionGraphicsItem>
#include <qmath.h>
#include <QGraphicsScene>
#include <QPolygon>

#define CURSOR_LEN          10
#define PI                  3.14159265
#define RESIZERECTWIDTH     5

AnalysisGraphicsItem::AnalysisGraphicsItem(AnalysisType type,QList<QPoint> points){

    analysis_type = type;

    foreach (QPoint p, points) {
        QPoint *point = new QPoint(p.x(),p.y());
        analysis_points.append(point);
    }

    if(analysis_type == Cire){
        analysis_cire_radius = (points[1].x() - points[0].x())/2;
        analysis_cire_center = QPoint(points[0].x() +analysis_cire_radius,points[0].y());
    }

    setFlags(QGraphicsItem::ItemIsSelectable |
             QGraphicsItem::ItemIsMovable |
             QGraphicsItem::ItemSendsGeometryChanges |
             QGraphicsItem::ItemIsFocusable);

}

AnalysisGraphicsItem::~AnalysisGraphicsItem(){

    qDeleteAll(analysis_points);

}

void AnalysisGraphicsItem::SetTitle(QString title){
    analysis_title = title;
}

void AnalysisGraphicsItem::SetViewSize(QSize size){
    view_size = size;
}

void AnalysisGraphicsItem::UpdateTempInfo(TempInfo info){
    temp_info = info;
}

void AnalysisGraphicsItem::GetPoints(QList<QPoint> &points){

    foreach(QPoint *p,analysis_points){
        int x = pos().x() + p->x();
        int y = pos().y() + p->y();
        points.append(QPoint(x,y));
    }
}

QRectF AnalysisGraphicsItem::boundingRect() const
{

    int x = -1 * view_size.width();
    int y = -1*view_size.height();
    int w = 3*view_size.width();
    int h = 3*view_size.height();
    return QRect(x,y,w,h);
}

//设置可拖动的面积
QPainterPath AnalysisGraphicsItem::shape() const
{
    //qDebug()<<"shape";

    QPainterPath path;

    if(analysis_type == Point){

        int line_width = 20;

        QPoint* p = analysis_points.at(0);
        QRect rect = QRect(p->x()-line_width/2-line_width,p->y()-line_width/2-line_width,
                           line_width+2*line_width,line_width+2*line_width);
        path.addRect(rect);
    }else if(analysis_type == Line || analysis_type == Rect || analysis_type == Cire){

        int offset = 20;

        QPoint* p1 = analysis_points.at(0);
        QPoint* p2 = analysis_points.at(1);
        int x = p1->x() < p2->x() ? p1->x() : p2->x();
        int y = p1->y() < p2->y() ? p1->y() : p2->y();
        int w = abs(p1->x() - p2->x())+1;
        int h = abs(p1->y() - p2->y())+1;

        if(analysis_type == Cire){

            y = y - w/2;
            h = w;
            QRect rect(x - offset, y - offset, w+2*offset, h+2*offset);
            path.addEllipse(rect);
        }else {
            QRect rect(x - offset, y - offset, w+2*offset, h+2*offset);
            path.addRect(rect);
        }

    }else if(analysis_type == Poly){

        QPolygon ploy;
        for(int i=0;i<analysis_points.size();i++){
            QPoint p = *analysis_points.at(i);
            ploy.append(p);
        }
        path.addPolygon(ploy);
    }

    return path;
}

void AnalysisGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{

    int TextWidth = 240;
    int TextHeight = 20;

    QColor normal_color(0,125,255);
    painter->setPen(QPen(normal_color,3));
    painter->setBrush(normal_color);

    painter->setRenderHint(QPainter::Antialiasing,true);

    if(option->state & QStyle::State_Selected)//工具被选中
    {

        QColor color(255,0,0);
        painter->setPen(QPen(color,1));
        int offset = RESIZERECTWIDTH;

        switch (analysis_type) {
        case Point:
        {
            painter->setBrush(Qt::NoBrush);
            QPoint* p = analysis_points.at(0);
            QRect rect(p->x()-offset,p->y()-offset,2*offset,2*offset);
            painter->drawRect(rect);
        }
            break;

        case Line:
        {

            painter->setBrush(Qt::NoBrush);
            for(int i=0;i<analysis_points.size();i++){
                QPoint* p = analysis_points.at(i);
                QRect rect = QRect(p->x()-offset,p->y()-offset,2*offset,2*offset);
                painter->drawRect(rect);
            }
        }
            break;

        case Rect:
        {
            painter->setBrush(Qt::NoBrush);
            QList<QPoint> points;
            QPoint p1 = *analysis_points.at(0);
            QPoint p4 = *analysis_points.at(1);
            QPoint p2 = QPoint(p4.x(),p1.y());
            QPoint p3 = QPoint(p1.x(),p4.y());
            points.append(p1);
            points.append(p2);
            points.append(p3);
            points.append(p4);

            for(int i=0;i<points.size();i++){
                QPoint p = points.at(i);
                QRect rect = QRect(p.x()-offset,p.y()-offset,2*offset,2*offset);
                painter->drawRect(rect);
            }
        }

            break;

        case Cire:
        {
            painter->setBrush(Qt::NoBrush);
            QList<QPoint> points;
            QPoint p1 = *analysis_points.at(0);
            QPoint p4 = *analysis_points.at(1);
            int radius = (p4.x() - p1.x())/2;
            QPoint p2 = QPoint(p1.x() + radius ,p1.y() - radius);
            QPoint p3 = QPoint(p1.x() + radius ,p4.y() + radius);
            points.append(p1);
            points.append(p2);
            points.append(p3);
            points.append(p4);
            for(int i=0;i<points.size();i++){
                QPoint p = points.at(i);
                QRect rect = QRect(p.x()-offset,p.y()-offset,2*offset,2*offset);
                painter->drawRect(rect);
            }
        }
            break;
        case Poly:
        {
            painter->setBrush(Qt::NoBrush);
            for(int i=0;i<analysis_points.size();i++){
                QPoint p = *analysis_points.at(i);
                QRect rect = QRect(p.x()-offset,p.y()-offset,2*offset,2*offset);
                painter->drawRect(rect);
            }
        }
            break;
        default:break;
        }

    }

    QString max_string = QString::number(temp_info.max_temp,'f', 1);
    QString min_string = QString::number(temp_info.min_temp,'f', 1);
    QString avg_string = QString::number(temp_info.avg_temp,'f', 1);

    if(analysis_type == Point){
        painter->setPen(QPen(normal_color,2));
        QPoint* p = analysis_points.at(0);
        painter->drawLine(p->x()-CURSOR_LEN/2,p->y(),p->x()+CURSOR_LEN/2,p->y());
        painter->drawLine(p->x(),p->y()-CURSOR_LEN/2,p->x(),p->y()+CURSOR_LEN/2);
        QRectF text_rect(p->x(),p->y()-TextHeight,TextWidth,TextHeight);
        QString info = QString("%0 temp:%1").arg(analysis_title).arg(max_string);
        painter->drawText(text_rect,info);

    }else if(analysis_type == Line || analysis_type == Rect || analysis_type == Cire){

        QPoint* p1 = analysis_points.at(0);
        QPoint* p2 = analysis_points.at(1);
        int x = p1->x() < p2->x() ? p1->x() : p2->x();
        int y = p1->y() < p2->y() ? p1->y() : p2->y();
        int w = abs(p1->x() - p2->x());
        int h = abs(p1->y() - p2->y());

        if(analysis_type == Line){
            painter->setPen(QPen(normal_color,2));
            painter->drawLine(*p1,*p2);
        }
        else if(analysis_type == Cire){
            painter->setPen(QPen(normal_color,2));
            painter->setBrush(Qt::NoBrush);
            y = y - w/2;
            h = w;
            QRect rect(x,y,w,h);
            painter->drawEllipse(rect);
        }else {

            painter->setPen(QPen(normal_color,2));
            painter->setBrush(Qt::NoBrush);
            QRect rect(x,y,w,h);
            painter->drawRect(rect);
        }

        QString info = QString("%0 max:%1 min:%2 avg:%3").arg(analysis_title).arg(max_string).arg(min_string).arg(avg_string);
        QRectF text_rect(p1->x(),p1->y()-TextHeight,TextWidth,TextHeight);
        if(analysis_type == Cire){
            QPoint p = QPoint(p1->x(),p1->y()-w/2);
            text_rect = QRectF(p.x(),p.y()-TextHeight,TextWidth,TextHeight);
        }
        painter->drawText(text_rect,info);
        int line_width = 5;
        int pen_width = 3;
        painter->setPen(QPen(Qt::red,pen_width));
        int pole_x = temp_info.max_point.x() - pos().x();
        int pole_y = temp_info.max_point.y() - pos().y();
        painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
        painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);

        painter->setPen(QPen(Qt::blue,pen_width));
        pole_x = temp_info.min_point.x() - pos().x();
        pole_y = temp_info.min_point.y() - pos().y();
        painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
        painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);



    }else if(analysis_type == Poly){

        painter->setPen(QPen(normal_color,2));
        painter->setBrush(Qt::NoBrush);
        QPolygon ploy;
        for(int i=0;i<analysis_points.size();i++){
            QPoint p = *analysis_points.at(i);
            ploy.append(p);
        }
        painter->drawPolygon(ploy);

        QPoint* p = analysis_points.at(0);
        painter->drawLine(p->x()-CURSOR_LEN/2,p->y(),p->x()+CURSOR_LEN/2,p->y());
        painter->drawLine(p->x(),p->y()-CURSOR_LEN/2,p->x(),p->y()+CURSOR_LEN/2);
        QRectF text_rect(p->x(),p->y()-TextHeight,TextWidth,TextHeight);
        QString info = QString("%0 temp:%1").arg(analysis_title).arg(max_string);
        painter->drawText(text_rect,info);

        int line_width = 5;
        int pen_width = 3;
        painter->setPen(QPen(Qt::red,pen_width));
        int pole_x = temp_info.max_point.x() - pos().x();
        int pole_y = temp_info.max_point.y() - pos().y();
        painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
        painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);

        painter->setPen(QPen(Qt::blue,pen_width));
        pole_x = temp_info.min_point.x() - pos().x();
        pole_y = temp_info.min_point.y() - pos().y();
        painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
        painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);

    }else if(analysis_type == Max || analysis_type == Min){

        int line_width = 5;
        int pen_width = 1;

        if(analysis_type == Max){
            int pole_x = temp_info.max_point.x() - pos().x();
            int pole_y = temp_info.max_point.y() - pos().y();
            QString info = QString("max:%1").arg(max_string);

            painter->setPen(QPen(Qt::red,pen_width));
            QRectF text_rect(pole_x,pole_y-TextHeight,TextWidth,TextHeight);
            painter->drawText(text_rect,info);


            painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
            painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);

        }else if(analysis_type == Min){
            int pole_x = temp_info.min_point.x() - pos().x();
            int pole_y = temp_info.min_point.y() - pos().y();
            QString info = QString("min:%1").arg(min_string);

            painter->setPen(QPen(Qt::blue,pen_width));
            QRectF text_rect = QRectF(pole_x,pole_y-TextHeight,TextWidth,TextHeight);
            painter->drawText(text_rect,info);

            painter->drawLine(pole_x-line_width,pole_y,pole_x+line_width,pole_y);
            painter->drawLine(pole_x,pole_y-line_width,pole_x,pole_y+line_width);
        }
    }

}

//碰撞检测
QVariant AnalysisGraphicsItem::itemChange(GraphicsItemChange change, const QVariant &value){

    //qDebug()<<"value:"<<value.toPointF();


    if(change==ItemPositionChange && scene())
    {



        QPointF left_top;
        QPointF right_bottom;

        switch (analysis_type) {//switch
        case Point:
        {

            QPoint* p = analysis_points.at(0);

            int x = value.toPointF().x()+ p->x();
            int y = value.toPointF().y()+ p->y();
            left_top = QPointF(x,y);
            right_bottom = QPointF(x,y);

            break;
        }
        case Line:
        {
            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);
            int x1 = qMin(p1->x(),p2->x());
            int y1 = qMin(p1->y(),p2->y());
            int x2 = qMax(p1->x(),p2->x());
            int y2 = qMax(p1->y(),p2->y());

            left_top = QPointF(value.toPointF().x()+x1,value.toPointF().y()+y1);
            right_bottom = QPointF(value.toPointF().x()+x2,value.toPointF().y()+y2);

            break;
        }
        case Rect:
        {

            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);
            left_top = QPointF(value.toPointF().x()+p1->x(),value.toPointF().y()+p1->y());
            right_bottom = QPointF(value.toPointF().x()+p2->x(),value.toPointF().y()+p2->y());

            break;
        }
        case Cire:
        {
            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);

            QPoint c1 = *p1;
            QPoint c2 = *p2;

            int radius = (p2->x() - p1->x())/2;
            left_top = QPoint(value.toPointF().x()+p1->x(),value.toPointF().y() + p1->y() - radius);
            right_bottom = QPoint(value.toPointF().x()+p2->x(),value.toPointF().y() + p2->y()+radius);

            break;
        }
        case Poly:
        {
            int min_x = 65535;
            int min_y = 65535;
            int max_x = 0;
            int max_y = 0;

            for(int i=0;i<analysis_points.size();++i){
                QPoint p = *analysis_points.at(i);
                min_x = fmin(p.x(),min_x);
                min_y = fmin(p.y(),min_y);
                max_x = fmax(p.x(),max_x);
                max_y = fmax(p.y(),max_y);
            }
            left_top = QPoint(value.toPointF().x()+min_x,value.toPointF().y()+min_y);
            right_bottom = QPoint(value.toPointF().x()+max_x,value.toPointF().y()+max_y);

        }
            break;
        default:
            break;
        }

        int w = right_bottom.x() - left_top.x();
        int h = right_bottom.y() - left_top.y();

        QRectF contains_rect = QRectF(scene()->sceneRect().topLeft(),QPointF(scene()->sceneRect().right()-w,scene()->sceneRect().bottom()-h));

        //qDebug()<<"contains_rect:"<<contains_rect;

        if(!contains_rect.contains(left_top))
        {

            int x_min = left_top.x() - value.toPointF().x();
            int y_min = left_top.y() - value.toPointF().y();

            int x = qMin(contains_rect.right() ,qMax(left_top.x(),contains_rect.left()))-x_min;
            int y = qMin(contains_rect.bottom(),qMax(left_top.y(),contains_rect.top()))-y_min;
            left_top.setX(x);
            left_top.setY(y);
            return left_top;
        }
    }
    return QGraphicsItem::itemChange(change,value);
}


void AnalysisGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
    //qDebug()<<"mousePressEvent: "<<e->pos();

    emit sgnAnalysisItemSelect(this);

    setSelected(true);

    int pressX = e->pos().x();
    int pressY = e->pos().y();

    int offset = RESIZERECTWIDTH;

    if(analysis_type == Line){

        for(int i=0;i<analysis_points.size();i++){
            QPoint* p = analysis_points.at(i);
            bool flag_1 = pressX<(p->x()+offset) && pressX > (p->x() - offset);
            bool flag_2 = pressY<(p->y()+offset) && pressY > (p->y() - offset);
            if( flag_1 && flag_2){

                resize_type = (ReSizeType)i;
                setCursor(Qt::SizeAllCursor);
            }

        }
    }else if(analysis_type == Rect){

        QList<QPoint> points;
        QPoint p1 = *analysis_points.at(0);
        QPoint p3 = *analysis_points.at(1);
        QPoint p2 = QPoint(p3.x(),p1.y());
        QPoint p4 = QPoint(p1.x(),p3.y());
        points.append(p1);
        points.append(p2);
        points.append(p3);
        points.append(p4);

        for(int i=0;i<points.size();i++){
            QPoint p = points.at(i);
            bool flag_1 = pressX<(p.x()+offset) && pressX > (p.x() - offset);
            bool flag_2 = pressY<(p.y()+offset) && pressY > (p.y() - offset);
            if( flag_1 && flag_2){
                resize_type = (ReSizeType)i;
                qDebug()<<"resize_type:"<<resize_type;
                setCursor(Qt::SizeAllCursor);
            }

        }


    }else if(analysis_type == Cire){

        QList<QPoint> points;
        QPoint p1 = *analysis_points.at(0);
        QPoint p3 = *analysis_points.at(1);
        int radius = (p3.x() - p1.x())/2;
        QPoint p2 = QPoint(p1.x() + radius ,p1.y() - radius);
        QPoint p4 = QPoint(p1.x() + radius ,p3.y() + radius);
        points.append(p1);
        points.append(p2);
        points.append(p3);
        points.append(p4);
        for(int i=0;i<points.size();i++){
            QPoint p = points.at(i);
            bool flag_1 = pressX<(p.x()+offset) && pressX > (p.x() - offset);
            bool flag_2 = pressY<(p.y()+offset) && pressY > (p.y() - offset);
            if( flag_1 && flag_2){
                resize_type = (ReSizeType)i;
                setCursor(Qt::SizeAllCursor);
            }

        }
    }else if(analysis_type == Poly){

        for(int i=0;i<analysis_points.size();i++){
            QPoint p = *analysis_points.at(i);
            bool flag_1 = pressX<(p.x()+offset) && pressX > (p.x() - offset);
            bool flag_2 = pressY<(p.y()+offset) && pressY > (p.y() - offset);
            if( flag_1 && flag_2){
                resize_type = SizePoint;
                qDebug()<<"resize_type:"<<resize_type;
                setCursor(Qt::SizeAllCursor);
                poly_sel_point = analysis_points.at(i);
                break;
            }

        }

        if(resize_type  == SizeNone){

            offset = 12;

            for(int i=0;i<analysis_points.size();i++){

                QPoint p1 = *analysis_points[i];

                QPoint p2;

                if(i == analysis_points.size() -1){
                    p2 = *analysis_points[0];
                }else{
                   p2 = *analysis_points[i+1];
                }


                double angle = atan2(p2.y() - p1.y(), p2.x() - p1.x()) + PI;

                int mx1,my1,mx2,my2,mx3,my3,mx4,my4;
                int x1 = p1.x() + sqrt(pow(offset/2,2)*2) * cos(angle + PI/4);
                int y1 = p1.y() + sqrt(pow(offset/2,2)*2) * sin(angle + PI/4);
                int x2 = p1.x() + sqrt(pow(offset/2,2)*2) * cos(angle - PI/4);
                int y2 = p1.y() + sqrt(pow(offset/2,2)*2) * sin(angle - PI/4);

                int x3 = p2.x() - sqrt(pow(offset/2,2)*2) * cos(angle + PI/4);
                int y3 = p2.y() - sqrt(pow(offset/2,2)*2) * sin(angle + PI/4);
                int x4 = p2.x() - sqrt(pow(offset/2,2)*2) * cos(angle - PI/4);
                int y4 = p2.y() - sqrt(pow(offset/2,2)*2) * sin(angle - PI/4);

                QPolygon  polygon;
                polygon<<QPoint(x1,y1)<<QPoint(x2,y2)<<QPoint(x3,y3)<<QPoint(x4,y4);

                if(polygon.containsPoint(QPoint(pressX,pressY),Qt::FillRule::OddEvenFill)){

                    resize_type = SizeAddPoint;
                    qDebug()<<"resize_type:"<<resize_type;
                    setCursor(Qt::SizeAllCursor);
                    poly_add_index = i+1;
                    break;
                }

            }
        }



    }

    if(resize_type == ReSizeType::SizeNone){
        QGraphicsItem::mousePressEvent(e);
    }
}

void AnalysisGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
    //e->pos().x()+pos().x()<0)? -pos().x()

    //qDebug()<<"mouseMoveEvent e->pos():"<<e->pos();
    //qDebug()<<"mouseMoveEvent pos():"<< pos();

    qDebug()<<"mouseMoveEvent resize_type:"<< resize_type;

    int posX = e->pos().x();
    int posY = e->pos().y();

    switch (analysis_type) {
    case Line:
    {
        prepareGeometryChange();
        switch (resize_type) {
        case SizePoint1:
        {
            QPoint* p = analysis_points.at(0);
            p->setX(e->pos().x());
            p->setY(e->pos().y());
        }
            break;
        case SizePoint2:
        {
            QPoint* p = analysis_points.at(1);
            p->setX(e->pos().x());
            p->setY(e->pos().y());
        }
            break;
        default:break;
        }
    }
        break;
    case Rect:
    {
        prepareGeometryChange();
        switch (resize_type) {
        case SizePoint1:
        {

            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);
            if(p2->x() - posX > 5){
                p1->setX(posX);
            }
            if(p2->y() - posY > 5){
                p1->setY(posY);
            }
        }
            break;
        case SizePoint2:
        {
            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);

            if(posX - p1->x() > 5){
                p2->setX(posX);
            }

            if(p2->y() - posY >5){
                p1->setY(posY);
            }
        }
            break;
        case SizePoint3:
        {
            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);

            if(posX - p1->x() > 5){
                p2->setX(posX);
            }

            if(posY - p1->y() > 5){
                p2->setY(posY);
            }

        }
            break;
        case SizePoint4:
        {
            QPoint* p1 = analysis_points.at(0);
            QPoint* p2 = analysis_points.at(1);

            if(p2->x() - posX > 5){
                p1->setX(posX);
            }

            if(posY - p1->y() > 5){
                p2->setY(posY);
            }
        }
            break;
        default:
            break;
        }
    }
        break;
    case Cire:
    {
        prepareGeometryChange();

        QPoint* p1 = analysis_points.at(0);
        QPoint* p2 = analysis_points.at(1);
        int old_radius = (p2->x() - p1->x())/2;
        QPoint center_point = QPoint(p1->x()+old_radius,p1->y());

        switch (resize_type) {
        case SizePoint1:
        {
            if(center_point.x() - posX > 5){
                int new_radius =  center_point.x() - posX;
                p1->setX(center_point.x() - new_radius);
                p2->setX(center_point.x() + new_radius);
            }

        }
            break;
        case SizePoint2:
        {
            if(center_point.y() - posY > 5){
                int new_radius =  center_point.y() - posY;
                p1->setX(center_point.x() - new_radius);
                p2->setX(center_point.x() + new_radius);
            }
        }
            break;
        case SizePoint3:
        {
            if(posX - center_point.x() > 5){
                int new_radius = posX - center_point.x();
                p1->setX(center_point.x() - new_radius);
                p2->setX(center_point.x() + new_radius);
            }
        }
            break;
        case SizePoint4:
        {
            if(posY - center_point.y() > 5){
                int new_radius = posY - center_point.y();
                p1->setX(center_point.x() - new_radius);
                p2->setX(center_point.x() + new_radius);
            }
        }
            break;
        default:
            break;
        }

    }
        break;
    case Poly:
        prepareGeometryChange();
        if(resize_type == SizePoint){
            poly_sel_point->setX(e->pos().x());
            poly_sel_point->setY(e->pos().y());
        }

        if(resize_type == SizeAddPoint){
            if(!poly_add_point){
                poly_add_point = new QPoint(e->pos().x(),e->pos().y());
                analysis_points.insert(poly_add_index,poly_add_point);
            }
            poly_add_point->setX(e->pos().x());
            poly_add_point->setY(e->pos().y());
        }


        break;
    default:
        break;
    }

    if(resize_type == ReSizeType::SizeNone){
        QGraphicsItem::mouseMoveEvent(e);
    }
}

void AnalysisGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
    qDebug()<<"mouseReleaseEvent";

    poly_add_point = nullptr;
    poly_add_index = -1;

    resize_type = ReSizeType::SizeNone;

    setCursor(Qt::ArrowCursor);

    QGraphicsItem::mouseReleaseEvent(e);
}
