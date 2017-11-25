/********************************************************************************
** Form generated from reading UI file 'rgbsegmentation.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RGBSEGMENTATION_H
#define UI_RGBSEGMENTATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_RGBSegmentation
{
public:
    QWidget *centralWidget;
    QSpinBox *spinBox_1;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QSpinBox *spinBox_2;
    QLabel *label_4;
    QSpinBox *spinBox_3;
    QLabel *label_5;
    QSpinBox *spinBox_4;
    QVTKWidget *qvtkWidget;
    QPushButton *pushButton_load;
    QPushButton *pushButton_save;

    void setupUi(QMainWindow *RGBSegmentation)
    {
        if (RGBSegmentation->objectName().isEmpty())
            RGBSegmentation->setObjectName(QStringLiteral("RGBSegmentation"));
        RGBSegmentation->resize(1400, 700);
        centralWidget = new QWidget(RGBSegmentation);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        spinBox_1 = new QSpinBox(centralWidget);
        spinBox_1->setObjectName(QStringLiteral("spinBox_1"));
        spinBox_1->setGeometry(QRect(220, 100, 61, 26));
        spinBox_1->setFocusPolicy(Qt::ClickFocus);
        spinBox_1->setMaximum(1000);
        spinBox_1->setValue(25);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(40, 100, 131, 17));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(90, 20, 151, 17));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(40, 180, 151, 17));
        spinBox_2 = new QSpinBox(centralWidget);
        spinBox_2->setObjectName(QStringLiteral("spinBox_2"));
        spinBox_2->setGeometry(QRect(220, 180, 61, 26));
        spinBox_2->setFocusPolicy(Qt::ClickFocus);
        spinBox_2->setMaximum(1000);
        spinBox_2->setValue(5);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(40, 260, 161, 17));
        spinBox_3 = new QSpinBox(centralWidget);
        spinBox_3->setObjectName(QStringLiteral("spinBox_3"));
        spinBox_3->setGeometry(QRect(220, 260, 61, 26));
        spinBox_3->setFocusPolicy(Qt::ClickFocus);
        spinBox_3->setMaximum(1000);
        spinBox_3->setValue(25);
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(40, 350, 131, 17));
        spinBox_4 = new QSpinBox(centralWidget);
        spinBox_4->setObjectName(QStringLiteral("spinBox_4"));
        spinBox_4->setGeometry(QRect(220, 340, 61, 26));
        spinBox_4->setFocusPolicy(Qt::ClickFocus);
        spinBox_4->setMaximum(1000);
        spinBox_4->setValue(150);
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(340, 10, 980, 640));
        pushButton_load = new QPushButton(centralWidget);
        pushButton_load->setObjectName(QStringLiteral("pushButton_load"));
        pushButton_load->setGeometry(QRect(40, 480, 89, 25));
        pushButton_save = new QPushButton(centralWidget);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));
        pushButton_save->setGeometry(QRect(180, 480, 89, 25));
        RGBSegmentation->setCentralWidget(centralWidget);

        retranslateUi(RGBSegmentation);

        QMetaObject::connectSlotsByName(RGBSegmentation);
    } // setupUi

    void retranslateUi(QMainWindow *RGBSegmentation)
    {
        RGBSegmentation->setWindowTitle(QApplication::translate("RGBSegmentation", "RGBSegmentation", Q_NULLPTR));
        label->setText(QApplication::translate("RGBSegmentation", "DistanceThreshold", Q_NULLPTR));
        label_2->setText(QApplication::translate("RGBSegmentation", "RGB-Segmentation", Q_NULLPTR));
        label_3->setText(QApplication::translate("RGBSegmentation", "PointColorThreshold", Q_NULLPTR));
        label_4->setText(QApplication::translate("RGBSegmentation", "RegionColorThreshold", Q_NULLPTR));
        label_5->setText(QApplication::translate("RGBSegmentation", "MinClusterSize", Q_NULLPTR));
        pushButton_load->setText(QApplication::translate("RGBSegmentation", "Load", Q_NULLPTR));
        pushButton_save->setText(QApplication::translate("RGBSegmentation", "Save", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RGBSegmentation: public Ui_RGBSegmentation {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RGBSEGMENTATION_H
