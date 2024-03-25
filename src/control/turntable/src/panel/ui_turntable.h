/********************************************************************************
** Form generated from reading UI file 'turntable.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TURNTABLE_H
#define UI_TURNTABLE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Panel_TurnTable
{
public:
    QLabel *Label_Pitch_Angle;
    QPushButton *Btn_Pitch_Zero;
    QPushButton *Btn_Pitch_Enable;
    QPushButton *Btn_Pitch_Disable;
    QPushButton *Btn_Pitch_Check;
    QPushButton *Btn_Azimuth_Check;
    QPushButton *Btn_Azimuth_Zero;
    QPushButton *Btn_Azimuth_Enable;
    QLabel *Label_Azimuth_Angle;
    QPushButton *Btn_Azimuth_Disable;
    QLabel *label;
    QPushButton *Btn_Pitch_Stop;
    QPushButton *Btn_Azimuth_Stop;
    QPushButton *Btn_Azimuth_Move;
    QLineEdit *edt_azimuth_pos;
    QPushButton *Btn_Pitch_Move;
    QLineEdit *edt_pitch_pos;

    void setupUi(QWidget *Panel_TurnTable)
    {
        if (Panel_TurnTable->objectName().isEmpty())
            Panel_TurnTable->setObjectName(QString::fromUtf8("Panel_TurnTable"));
        Panel_TurnTable->resize(329, 306);
        Label_Pitch_Angle = new QLabel(Panel_TurnTable);
        Label_Pitch_Angle->setObjectName(QString::fromUtf8("Label_Pitch_Angle"));
        Label_Pitch_Angle->setGeometry(QRect(10, 40, 331, 40));
        QFont font;
        font.setPointSize(14);
        Label_Pitch_Angle->setFont(font);
        Btn_Pitch_Zero = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Zero->setObjectName(QString::fromUtf8("Btn_Pitch_Zero"));
        Btn_Pitch_Zero->setGeometry(QRect(220, 130, 70, 40));
        Btn_Pitch_Zero->setMaximumSize(QSize(70, 16777215));
        Btn_Pitch_Zero->setFont(font);
        Btn_Pitch_Enable = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Enable->setObjectName(QString::fromUtf8("Btn_Pitch_Enable"));
        Btn_Pitch_Enable->setGeometry(QRect(0, 80, 70, 40));
        Btn_Pitch_Enable->setMaximumSize(QSize(70, 16777215));
        Btn_Pitch_Enable->setFont(font);
        Btn_Pitch_Disable = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Disable->setObjectName(QString::fromUtf8("Btn_Pitch_Disable"));
        Btn_Pitch_Disable->setGeometry(QRect(80, 80, 70, 40));
        Btn_Pitch_Disable->setMaximumSize(QSize(70, 16777215));
        Btn_Pitch_Disable->setFont(font);
        Btn_Pitch_Check = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Check->setObjectName(QString::fromUtf8("Btn_Pitch_Check"));
        Btn_Pitch_Check->setGeometry(QRect(170, 80, 70, 40));
        Btn_Pitch_Check->setMaximumSize(QSize(70, 16777215));
        Btn_Pitch_Check->setFont(font);
        Btn_Azimuth_Check = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Check->setObjectName(QString::fromUtf8("Btn_Azimuth_Check"));
        Btn_Azimuth_Check->setGeometry(QRect(160, 210, 80, 40));
        Btn_Azimuth_Check->setFont(font);
        Btn_Azimuth_Zero = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Zero->setObjectName(QString::fromUtf8("Btn_Azimuth_Zero"));
        Btn_Azimuth_Zero->setGeometry(QRect(210, 260, 81, 40));
        Btn_Azimuth_Zero->setFont(font);
        Btn_Azimuth_Enable = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Enable->setObjectName(QString::fromUtf8("Btn_Azimuth_Enable"));
        Btn_Azimuth_Enable->setGeometry(QRect(0, 210, 80, 40));
        Btn_Azimuth_Enable->setFont(font);
        Label_Azimuth_Angle = new QLabel(Panel_TurnTable);
        Label_Azimuth_Angle->setObjectName(QString::fromUtf8("Label_Azimuth_Angle"));
        Label_Azimuth_Angle->setGeometry(QRect(10, 170, 321, 40));
        Label_Azimuth_Angle->setFont(font);
        Btn_Azimuth_Disable = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Disable->setObjectName(QString::fromUtf8("Btn_Azimuth_Disable"));
        Btn_Azimuth_Disable->setGeometry(QRect(80, 210, 80, 40));
        Btn_Azimuth_Disable->setFont(font);
        label = new QLabel(Panel_TurnTable);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 10, 261, 20));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font1.setPointSize(15);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);
        Btn_Pitch_Stop = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Stop->setObjectName(QString::fromUtf8("Btn_Pitch_Stop"));
        Btn_Pitch_Stop->setGeometry(QRect(250, 80, 70, 40));
        Btn_Pitch_Stop->setMaximumSize(QSize(70, 16777215));
        Btn_Pitch_Stop->setFont(font);
        Btn_Azimuth_Stop = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Stop->setObjectName(QString::fromUtf8("Btn_Azimuth_Stop"));
        Btn_Azimuth_Stop->setGeometry(QRect(240, 210, 80, 40));
        Btn_Azimuth_Stop->setFont(font);
        Btn_Azimuth_Move = new QPushButton(Panel_TurnTable);
        Btn_Azimuth_Move->setObjectName(QString::fromUtf8("Btn_Azimuth_Move"));
        Btn_Azimuth_Move->setGeometry(QRect(0, 260, 91, 40));
        Btn_Azimuth_Move->setFont(font);
        edt_azimuth_pos = new QLineEdit(Panel_TurnTable);
        edt_azimuth_pos->setObjectName(QString::fromUtf8("edt_azimuth_pos"));
        edt_azimuth_pos->setGeometry(QRect(110, 260, 51, 41));
        QFont font2;
        font2.setPointSize(16);
        edt_azimuth_pos->setFont(font2);
        Btn_Pitch_Move = new QPushButton(Panel_TurnTable);
        Btn_Pitch_Move->setObjectName(QString::fromUtf8("Btn_Pitch_Move"));
        Btn_Pitch_Move->setGeometry(QRect(0, 130, 91, 40));
        Btn_Pitch_Move->setFont(font);
        edt_pitch_pos = new QLineEdit(Panel_TurnTable);
        edt_pitch_pos->setObjectName(QString::fromUtf8("edt_pitch_pos"));
        edt_pitch_pos->setGeometry(QRect(110, 130, 61, 41));
        edt_pitch_pos->setFont(font2);
        edt_pitch_pos->setStyleSheet(QString::fromUtf8(""));

        retranslateUi(Panel_TurnTable);

        QMetaObject::connectSlotsByName(Panel_TurnTable);
    } // setupUi

    void retranslateUi(QWidget *Panel_TurnTable)
    {
        Panel_TurnTable->setWindowTitle(QApplication::translate("Panel_TurnTable", "\344\274\272\346\234\215\350\275\254\345\217\260", nullptr));
        Label_Pitch_Angle->setText(QApplication::translate("Panel_TurnTable", "\344\277\257\344\273\260\350\247\222", nullptr));
        Btn_Pitch_Zero->setText(QApplication::translate("Panel_TurnTable", "\345\275\222\351\233\266", nullptr));
        Btn_Pitch_Enable->setText(QApplication::translate("Panel_TurnTable", "Enable", nullptr));
        Btn_Pitch_Disable->setText(QApplication::translate("Panel_TurnTable", "Disable", nullptr));
        Btn_Pitch_Check->setText(QApplication::translate("Panel_TurnTable", "\345\267\241\350\210\252", nullptr));
        Btn_Azimuth_Check->setText(QApplication::translate("Panel_TurnTable", "\345\267\241\350\210\252", nullptr));
        Btn_Azimuth_Zero->setText(QApplication::translate("Panel_TurnTable", "\345\275\222\351\233\266", nullptr));
        Btn_Azimuth_Enable->setText(QApplication::translate("Panel_TurnTable", "Enable", nullptr));
        Label_Azimuth_Angle->setText(QApplication::translate("Panel_TurnTable", "\346\226\271\344\275\215\345\220\221", nullptr));
        Btn_Azimuth_Disable->setText(QApplication::translate("Panel_TurnTable", "Disable", nullptr));
        label->setText(QApplication::translate("Panel_TurnTable", "\344\270\244\347\273\264\344\274\272\346\234\215\344\274\272\346\234\215\350\275\254\345\217\260", nullptr));
        Btn_Pitch_Stop->setText(QApplication::translate("Panel_TurnTable", "\345\201\234\346\255\242", nullptr));
        Btn_Azimuth_Stop->setText(QApplication::translate("Panel_TurnTable", "\345\201\234\346\255\242", nullptr));
        Btn_Azimuth_Move->setText(QApplication::translate("Panel_TurnTable", "\350\277\220\345\212\250\350\207\263", nullptr));
        edt_azimuth_pos->setText(QApplication::translate("Panel_TurnTable", "0", nullptr));
        Btn_Pitch_Move->setText(QApplication::translate("Panel_TurnTable", "\350\277\220\345\212\250\350\207\263", nullptr));
        edt_pitch_pos->setText(QApplication::translate("Panel_TurnTable", "0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Panel_TurnTable: public Ui_Panel_TurnTable {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TURNTABLE_H
