#ifndef ANALYSISGRAPHICSITEM_H
#define ANALYSISGRAPHICSITEM_H

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include "common.h"

class AnalysisGraphicsItem :public QObject, public QGraphicsItem
{
    enum ReSizeType{
        SizePoint1 = 0,
        SizePoint2,
        SizePoint3,
        SizePoint4,
        SizePoint,
        SizeAddPoint,
        SizeNone
    };

    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:

    explicit AnalysisGraphicsItem(AnalysisType type,QList<QPoint> points);

    ~AnalysisGraphicsItem();

    void SetTitle(QString title);
    void SetViewSize(QSize size);
    void GetPoints(QList<QPoint> &points);
    void UpdateTempInfo(TempInfo info);

    void SetTag(int tag){this->tag = tag;}
    int GetTag(){return tag;}
    AnalysisType GetAnalysisType(){return analysis_type;}

signals:
    void sgnAnalysisItemSelect(AnalysisGraphicsItem* item);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *e);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *e);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *e);

    virtual QRectF boundingRect() const;
    virtual QPainterPath shape() const;
    virtual void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget *widget);
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

public slots:



private:

    AnalysisType analysis_type;
    QList<QPoint*> analysis_points;
    QPoint *poly_sel_point = nullptr;
    int poly_add_index = -1;
    QPoint *poly_add_point = nullptr;

    int analysis_cire_radius;
    QPoint analysis_cire_center;
    QString analysis_title;
    QSize view_size;
    ReSizeType resize_type = ReSizeType::SizeNone;
    int tag;
    TempInfo temp_info;
};

#endif // ANALYSISGRAPHICSITEM_H
