/********************************************************************************
** Form generated from reading UI file 'global_plan_sim.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GLOBAL_PLAN_SIM_H
#define UI_GLOBAL_PLAN_SIM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Panel_Global_Plan_Sim
{
public:
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_agvid;
    QLabel *lab1;
    QLabel *lab2;
    QLabel *lab3;
    QLabel *lab4;
    QLabel *lab5;
    QLabel *lab6;
    QLabel *lab7;
    QLabel *lab8;
    QHBoxLayout *horizontalLayout_2;
    QLabel *lab4_2;
    QComboBox *cb_load;
    QHBoxLayout *horizontalLayout;
    QPushButton *btn_enable;
    QPushButton *btn_load;
    QPushButton *btn_obs;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btn_cleartrack;
    QPushButton *btn_stop;
    QPushButton *btn_syscheck;
    QPushButton *btn_save;

    void setupUi(QWidget *Panel_Global_Plan_Sim)
    {
        if (Panel_Global_Plan_Sim->objectName().isEmpty())
            Panel_Global_Plan_Sim->setObjectName(QString::fromUtf8("Panel_Global_Plan_Sim"));
        Panel_Global_Plan_Sim->resize(446, 418);
        verticalLayout = new QVBoxLayout(Panel_Global_Plan_Sim);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_agvid = new QLabel(Panel_Global_Plan_Sim);
        label_agvid->setObjectName(QString::fromUtf8("label_agvid"));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font.setPointSize(15);
        font.setBold(true);
        font.setWeight(75);
        label_agvid->setFont(font);
        label_agvid->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_agvid);

        lab1 = new QLabel(Panel_Global_Plan_Sim);
        lab1->setObjectName(QString::fromUtf8("lab1"));
        QFont font1;
        font1.setPointSize(14);
        lab1->setFont(font1);

        verticalLayout_2->addWidget(lab1);

        lab2 = new QLabel(Panel_Global_Plan_Sim);
        lab2->setObjectName(QString::fromUtf8("lab2"));
        lab2->setFont(font1);

        verticalLayout_2->addWidget(lab2);

        lab3 = new QLabel(Panel_Global_Plan_Sim);
        lab3->setObjectName(QString::fromUtf8("lab3"));
        lab3->setFont(font1);

        verticalLayout_2->addWidget(lab3);

        lab4 = new QLabel(Panel_Global_Plan_Sim);
        lab4->setObjectName(QString::fromUtf8("lab4"));
        lab4->setFont(font1);

        verticalLayout_2->addWidget(lab4);

        lab5 = new QLabel(Panel_Global_Plan_Sim);
        lab5->setObjectName(QString::fromUtf8("lab5"));
        lab5->setFont(font1);

        verticalLayout_2->addWidget(lab5);

        lab6 = new QLabel(Panel_Global_Plan_Sim);
        lab6->setObjectName(QString::fromUtf8("lab6"));
        lab6->setFont(font1);

        verticalLayout_2->addWidget(lab6);

        lab7 = new QLabel(Panel_Global_Plan_Sim);
        lab7->setObjectName(QString::fromUtf8("lab7"));
        lab7->setFont(font1);

        verticalLayout_2->addWidget(lab7);

        lab8 = new QLabel(Panel_Global_Plan_Sim);
        lab8->setObjectName(QString::fromUtf8("lab8"));
        lab8->setFont(font1);

        verticalLayout_2->addWidget(lab8);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        lab4_2 = new QLabel(Panel_Global_Plan_Sim);
        lab4_2->setObjectName(QString::fromUtf8("lab4_2"));
        lab4_2->setFont(font1);

        horizontalLayout_2->addWidget(lab4_2);

        cb_load = new QComboBox(Panel_Global_Plan_Sim);
        cb_load->addItem(QString());
        cb_load->addItem(QString());
        cb_load->setObjectName(QString::fromUtf8("cb_load"));
        QFont font2;
        font2.setPointSize(16);
        cb_load->setFont(font2);

        horizontalLayout_2->addWidget(cb_load);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        btn_enable = new QPushButton(Panel_Global_Plan_Sim);
        btn_enable->setObjectName(QString::fromUtf8("btn_enable"));
        btn_enable->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout->addWidget(btn_enable);

        btn_load = new QPushButton(Panel_Global_Plan_Sim);
        btn_load->setObjectName(QString::fromUtf8("btn_load"));
        btn_load->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout->addWidget(btn_load);

        btn_obs = new QPushButton(Panel_Global_Plan_Sim);
        btn_obs->setObjectName(QString::fromUtf8("btn_obs"));
        btn_obs->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout->addWidget(btn_obs);


        verticalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        btn_cleartrack = new QPushButton(Panel_Global_Plan_Sim);
        btn_cleartrack->setObjectName(QString::fromUtf8("btn_cleartrack"));
        btn_cleartrack->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout_3->addWidget(btn_cleartrack);

        btn_stop = new QPushButton(Panel_Global_Plan_Sim);
        btn_stop->setObjectName(QString::fromUtf8("btn_stop"));
        btn_stop->setStyleSheet(QString::fromUtf8("font: 16pt \"Ubuntu\";\n"
"background-color: rgb(239, 41, 41);"));

        horizontalLayout_3->addWidget(btn_stop);

        btn_syscheck = new QPushButton(Panel_Global_Plan_Sim);
        btn_syscheck->setObjectName(QString::fromUtf8("btn_syscheck"));
        btn_syscheck->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout_3->addWidget(btn_syscheck);

        btn_save = new QPushButton(Panel_Global_Plan_Sim);
        btn_save->setObjectName(QString::fromUtf8("btn_save"));
        btn_save->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));

        horizontalLayout_3->addWidget(btn_save);


        verticalLayout_2->addLayout(horizontalLayout_3);


        verticalLayout->addLayout(verticalLayout_2);


        retranslateUi(Panel_Global_Plan_Sim);

        QMetaObject::connectSlotsByName(Panel_Global_Plan_Sim);
    } // setupUi

    void retranslateUi(QWidget *Panel_Global_Plan_Sim)
    {
        Panel_Global_Plan_Sim->setWindowTitle(QApplication::translate("Panel_Global_Plan_Sim", "\344\274\272\346\234\215\350\275\254\345\217\260", nullptr));
        label_agvid->setText(QApplication::translate("Panel_Global_Plan_Sim", "AGV\347\233\221\346\216\247", nullptr));
        lab1->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab1", nullptr));
        lab2->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab2", nullptr));
        lab3->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab3", nullptr));
        lab4->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab4", nullptr));
        lab5->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab5", nullptr));
        lab6->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab6", nullptr));
        lab7->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab7", nullptr));
        lab8->setText(QApplication::translate("Panel_Global_Plan_Sim", "lab8", nullptr));
        lab4_2->setText(QApplication::translate("Panel_Global_Plan_Sim", "\350\267\257\345\276\204\346\226\207\344\273\266", nullptr));
        cb_load->setItemText(0, QApplication::translate("Panel_Global_Plan_Sim", "1", nullptr));
        cb_load->setItemText(1, QApplication::translate("Panel_Global_Plan_Sim", "2", nullptr));

        btn_enable->setText(QApplication::translate("Panel_Global_Plan_Sim", "\344\275\277\350\203\275", nullptr));
        btn_load->setText(QApplication::translate("Panel_Global_Plan_Sim", "\350\257\273\345\217\226", nullptr));
        btn_obs->setText(QApplication::translate("Panel_Global_Plan_Sim", "\351\201\277\351\232\234", nullptr));
        btn_cleartrack->setText(QApplication::translate("Panel_Global_Plan_Sim", "\346\270\205\351\231\244", nullptr));
        btn_stop->setText(QApplication::translate("Panel_Global_Plan_Sim", "\345\201\234\346\255\242", nullptr));
        btn_syscheck->setText(QApplication::translate("Panel_Global_Plan_Sim", "\347\212\266\346\200\201", nullptr));
        btn_save->setText(QApplication::translate("Panel_Global_Plan_Sim", "\344\277\235\345\255\230", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Panel_Global_Plan_Sim: public Ui_Panel_Global_Plan_Sim {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GLOBAL_PLAN_SIM_H
