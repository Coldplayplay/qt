/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QSlider *horizontalSlider_x1;
    QSlider *horizontalSlider_y1;
    QSlider *horizontalSlider_z1;
    QLabel *label_7;
    QLabel *label_9;
    QLabel *label_11;
    QPushButton *pushButton_save;
    QSlider *horizontalSlider_x2;
    QLabel *label_12;
    QLabel *label_10;
    QSlider *horizontalSlider_y2;
    QLabel *label_8;
    QSlider *horizontalSlider_z2;
    QLabel *label_14;
    QLabel *label_13;
    QSlider *horizontalSlider_z1_2;
    QSpinBox *spinBox_x1;
    QSpinBox *spinBox_y1;
    QSpinBox *spinBox_z1;
    QSpinBox *spinBox_z2;
    QSpinBox *spinBox_y2;
    QSpinBox *spinBox_x2;
    QLabel *label_1;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_4;
    QPushButton *pushButton_load;
    QRadioButton *radioButton;
    QCheckBox *checkBox;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->resize(1980, 800);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(670, 20, 1100, 640));
        horizontalSlider_x1 = new QSlider(centralwidget);
        horizontalSlider_x1->setObjectName(QStringLiteral("horizontalSlider_x1"));
        horizontalSlider_x1->setGeometry(QRect(30, 170, 160, 29));
        horizontalSlider_x1->setMinimum(-200);
        horizontalSlider_x1->setMaximum(200);
        horizontalSlider_x1->setSingleStep(1);
        horizontalSlider_x1->setValue(-50);
        horizontalSlider_x1->setOrientation(Qt::Horizontal);
        horizontalSlider_y1 = new QSlider(centralwidget);
        horizontalSlider_y1->setObjectName(QStringLiteral("horizontalSlider_y1"));
        horizontalSlider_y1->setGeometry(QRect(30, 250, 160, 29));
        horizontalSlider_y1->setMinimum(-200);
        horizontalSlider_y1->setMaximum(200);
        horizontalSlider_y1->setValue(-40);
        horizontalSlider_y1->setOrientation(Qt::Horizontal);
        horizontalSlider_z1 = new QSlider(centralwidget);
        horizontalSlider_z1->setObjectName(QStringLiteral("horizontalSlider_z1"));
        horizontalSlider_z1->setGeometry(QRect(30, 330, 160, 29));
        horizontalSlider_z1->setMinimum(-200);
        horizontalSlider_z1->setMaximum(200);
        horizontalSlider_z1->setValue(0);
        horizontalSlider_z1->setOrientation(Qt::Horizontal);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(30, 130, 191, 31));
        QFont font;
        font.setPointSize(16);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label_7->setFont(font);
        label_9 = new QLabel(centralwidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(30, 210, 191, 31));
        label_9->setFont(font);
        label_11 = new QLabel(centralwidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(30, 300, 191, 31));
        label_11->setFont(font);
        pushButton_save = new QPushButton(centralwidget);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));
        pushButton_save->setGeometry(QRect(390, 490, 181, 61));
        horizontalSlider_x2 = new QSlider(centralwidget);
        horizontalSlider_x2->setObjectName(QStringLiteral("horizontalSlider_x2"));
        horizontalSlider_x2->setGeometry(QRect(350, 170, 160, 29));
        horizontalSlider_x2->setMinimum(-200);
        horizontalSlider_x2->setMaximum(200);
        horizontalSlider_x2->setValue(50);
        horizontalSlider_x2->setOrientation(Qt::Horizontal);
        label_12 = new QLabel(centralwidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(350, 300, 191, 31));
        label_12->setFont(font);
        label_10 = new QLabel(centralwidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(350, 210, 191, 31));
        label_10->setFont(font);
        horizontalSlider_y2 = new QSlider(centralwidget);
        horizontalSlider_y2->setObjectName(QStringLiteral("horizontalSlider_y2"));
        horizontalSlider_y2->setGeometry(QRect(350, 250, 160, 29));
        horizontalSlider_y2->setMinimum(-200);
        horizontalSlider_y2->setMaximum(200);
        horizontalSlider_y2->setValue(25);
        horizontalSlider_y2->setOrientation(Qt::Horizontal);
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(350, 130, 191, 31));
        label_8->setFont(font);
        horizontalSlider_z2 = new QSlider(centralwidget);
        horizontalSlider_z2->setObjectName(QStringLiteral("horizontalSlider_z2"));
        horizontalSlider_z2->setGeometry(QRect(350, 330, 160, 29));
        horizontalSlider_z2->setMinimum(-200);
        horizontalSlider_z2->setMaximum(200);
        horizontalSlider_z2->setValue(150);
        horizontalSlider_z2->setOrientation(Qt::Horizontal);
        label_14 = new QLabel(centralwidget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(160, 50, 271, 31));
        label_14->setFont(font);
        label_13 = new QLabel(centralwidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(30, 380, 191, 31));
        label_13->setFont(font);
        horizontalSlider_z1_2 = new QSlider(centralwidget);
        horizontalSlider_z1_2->setObjectName(QStringLiteral("horizontalSlider_z1_2"));
        horizontalSlider_z1_2->setGeometry(QRect(30, 410, 160, 29));
        horizontalSlider_z1_2->setMaximum(255);
        horizontalSlider_z1_2->setValue(128);
        horizontalSlider_z1_2->setOrientation(Qt::Horizontal);
        spinBox_x1 = new QSpinBox(centralwidget);
        spinBox_x1->setObjectName(QStringLiteral("spinBox_x1"));
        spinBox_x1->setGeometry(QRect(200, 170, 48, 27));
        spinBox_x1->setMinimum(-200);
        spinBox_x1->setMaximum(200);
        spinBox_x1->setSingleStep(1);
        spinBox_x1->setValue(-50);
        spinBox_y1 = new QSpinBox(centralwidget);
        spinBox_y1->setObjectName(QStringLiteral("spinBox_y1"));
        spinBox_y1->setGeometry(QRect(200, 250, 48, 27));
        spinBox_y1->setMinimum(-200);
        spinBox_y1->setMaximum(200);
        spinBox_y1->setValue(-40);
        spinBox_z1 = new QSpinBox(centralwidget);
        spinBox_z1->setObjectName(QStringLiteral("spinBox_z1"));
        spinBox_z1->setGeometry(QRect(200, 330, 48, 27));
        spinBox_z1->setMinimum(-200);
        spinBox_z1->setMaximum(200);
        spinBox_z2 = new QSpinBox(centralwidget);
        spinBox_z2->setObjectName(QStringLiteral("spinBox_z2"));
        spinBox_z2->setGeometry(QRect(530, 330, 48, 27));
        spinBox_z2->setMinimum(-200);
        spinBox_z2->setMaximum(200);
        spinBox_z2->setValue(150);
        spinBox_y2 = new QSpinBox(centralwidget);
        spinBox_y2->setObjectName(QStringLiteral("spinBox_y2"));
        spinBox_y2->setGeometry(QRect(530, 250, 48, 27));
        spinBox_y2->setMinimum(-200);
        spinBox_y2->setMaximum(200);
        spinBox_y2->setValue(25);
        spinBox_x2 = new QSpinBox(centralwidget);
        spinBox_x2->setObjectName(QStringLiteral("spinBox_x2"));
        spinBox_x2->setGeometry(QRect(530, 170, 48, 27));
        spinBox_x2->setMinimum(-200);
        spinBox_x2->setMaximum(200);
        spinBox_x2->setValue(50);
        label_1 = new QLabel(centralwidget);
        label_1->setObjectName(QStringLiteral("label_1"));
        label_1->setGeometry(QRect(270, 170, 41, 17));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(270, 250, 41, 17));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(600, 180, 41, 17));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(270, 340, 31, 17));
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(600, 340, 41, 17));
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(600, 260, 41, 17));
        pushButton_load = new QPushButton(centralwidget);
        pushButton_load->setObjectName(QStringLiteral("pushButton_load"));
        pushButton_load->setGeometry(QRect(70, 490, 181, 61));
        radioButton = new QRadioButton(centralwidget);
        radioButton->setObjectName(QStringLiteral("radioButton"));
        radioButton->setGeometry(QRect(90, 600, 117, 22));
        checkBox = new QCheckBox(centralwidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(260, 590, 97, 22));
        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);
        QObject::connect(horizontalSlider_x1, SIGNAL(valueChanged(int)), spinBox_x1, SLOT(setValue(int)));
        QObject::connect(spinBox_x1, SIGNAL(valueChanged(int)), horizontalSlider_x1, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_y1, SIGNAL(sliderMoved(int)), spinBox_y1, SLOT(setValue(int)));
        QObject::connect(spinBox_y1, SIGNAL(valueChanged(int)), horizontalSlider_y1, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_z1, SIGNAL(sliderMoved(int)), spinBox_z1, SLOT(setValue(int)));
        QObject::connect(spinBox_z1, SIGNAL(valueChanged(int)), horizontalSlider_z1, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_x2, SIGNAL(sliderMoved(int)), spinBox_x2, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_y2, SIGNAL(sliderMoved(int)), spinBox_y2, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_z2, SIGNAL(sliderMoved(int)), spinBox_z2, SLOT(setValue(int)));
        QObject::connect(spinBox_x2, SIGNAL(valueChanged(int)), horizontalSlider_x2, SLOT(setValue(int)));
        QObject::connect(spinBox_y2, SIGNAL(valueChanged(int)), horizontalSlider_y2, SLOT(setValue(int)));
        QObject::connect(spinBox_z2, SIGNAL(valueChanged(int)), horizontalSlider_z2, SLOT(setValue(int)));

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", Q_NULLPTR));
        label_7->setText(QApplication::translate("PCLViewer", "x bottom", Q_NULLPTR));
        label_9->setText(QApplication::translate("PCLViewer", "y bottom", Q_NULLPTR));
        label_11->setText(QApplication::translate("PCLViewer", "z bottom", Q_NULLPTR));
        pushButton_save->setText(QApplication::translate("PCLViewer", "save", Q_NULLPTR));
        label_12->setText(QApplication::translate("PCLViewer", "z upper", Q_NULLPTR));
        label_10->setText(QApplication::translate("PCLViewer", "y upper", Q_NULLPTR));
        label_8->setText(QApplication::translate("PCLViewer", "x upper", Q_NULLPTR));
        label_14->setText(QApplication::translate("PCLViewer", "pointcloud conditional filter", Q_NULLPTR));
        label_13->setText(QApplication::translate("PCLViewer", "voxel-leafsize", Q_NULLPTR));
        label_1->setText(QApplication::translate("PCLViewer", "-0.5", Q_NULLPTR));
        label_3->setText(QApplication::translate("PCLViewer", "-0.4", Q_NULLPTR));
        label_2->setText(QApplication::translate("PCLViewer", "0.5", Q_NULLPTR));
        label_5->setText(QApplication::translate("PCLViewer", "0", Q_NULLPTR));
        label_6->setText(QApplication::translate("PCLViewer", "1.5", Q_NULLPTR));
        label_4->setText(QApplication::translate("PCLViewer", "0.25", Q_NULLPTR));
        pushButton_load->setText(QApplication::translate("PCLViewer", "load file", Q_NULLPTR));
        radioButton->setText(QApplication::translate("PCLViewer", "RadioButton", Q_NULLPTR));
        checkBox->setText(QApplication::translate("PCLViewer", "CheckBox", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
