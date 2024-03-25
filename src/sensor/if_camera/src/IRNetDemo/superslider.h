#ifndef SUPERSLIDER_H
#define SUPERSLIDER_H


#include "qslider.h"
#include "qlabel.h"


/*
*  Super sick nasty awesome double handled slider!
*
*   @author Steve
*/
class SuperSliderHandle;

class SuperSlider: public QSlider
{
    Q_OBJECT
public:
    SuperSlider(QWidget *parent = 0);
    SuperSliderHandle *alt_handle;
    void mouseReleaseEvent(QMouseEvent *event);
    int alt_value();
    void alt_setValue(int value);
    void Reset();
    void alt_update();
signals:
    void alt_valueChanged(int);
};

class SuperEventFilter : public QObject
{
public:
    SuperEventFilter(SuperSlider *_grandParent)
    {grandParent = _grandParent;};

protected:
    bool eventFilter(QObject* obj, QEvent* event);

private:
    SuperSlider *grandParent;
};

class SuperSliderHandle: public QLabel
{
    Q_OBJECT
public:
    SuperSliderHandle(SuperSlider *parent = 0);
    void mousePressEvent(QMouseEvent *event);
    int value();
    int mapValue();
    SuperSlider *parent;
    bool handleActivated;
private:
    SuperEventFilter *filter;
public slots:
    void setValue(double value);
};

#endif // SUPERSLIDER_H
