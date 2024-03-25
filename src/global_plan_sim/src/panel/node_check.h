#ifndef NODE_CHECK_H
#define NODE_CHECK_H

#include <ros/ros.h>
#include <QDialog>
#include <QTimer>
#include "ui_nodecheck.h"

using namespace std;

class dialog_node_check : public QDialog
{
Q_OBJECT
// 内部槽.
protected Q_SLOTS:
    void qtmrfunc();

protected:
    ros::NodeHandle *nh;

public:
    explicit dialog_node_check(QWidget *parent = nullptr);
    ~dialog_node_check();

    void update_rate(string param, QLabel *lab);

private:
    Ui_Dialog_NodeCheck *ui;
    QTimer qtmr;
};

#endif 
