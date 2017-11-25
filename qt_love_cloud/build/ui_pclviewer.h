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
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_Axis;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *radioButton_x;
    QRadioButton *radioButton_y;
    QRadioButton *radioButton_z;
    QSpacerItem *verticalSpacer;
    QGroupBox *groupBox_ColorMode;
    QVBoxLayout *verticalLayout;
    QRadioButton *radioButton_BlueRed;
    QRadioButton *radioButton_GreenMagenta;
    QRadioButton *radioButton_WhiteRed;
    QRadioButton *radioButton_GreyRed;
    QRadioButton *radioButton_Rainbow;
    QSpacerItem *verticalSpacer_3;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_load;
    QPushButton *pushButton_save;
    QVTKWidget *qvtkWidget;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(896, 498);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_Axis = new QGroupBox(centralwidget);
        groupBox_Axis->setObjectName(QStringLiteral("groupBox_Axis"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_Axis->sizePolicy().hasHeightForWidth());
        groupBox_Axis->setSizePolicy(sizePolicy);
        groupBox_Axis->setMinimumSize(QSize(180, 60));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setWeight(75);
        groupBox_Axis->setFont(font);
        horizontalLayout_2 = new QHBoxLayout(groupBox_Axis);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        radioButton_x = new QRadioButton(groupBox_Axis);
        radioButton_x->setObjectName(QStringLiteral("radioButton_x"));
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(false);
        font1.setWeight(50);
        radioButton_x->setFont(font1);
        radioButton_x->setChecked(false);

        horizontalLayout_2->addWidget(radioButton_x);

        radioButton_y = new QRadioButton(groupBox_Axis);
        radioButton_y->setObjectName(QStringLiteral("radioButton_y"));
        radioButton_y->setFont(font1);
        radioButton_y->setChecked(true);

        horizontalLayout_2->addWidget(radioButton_y);

        radioButton_z = new QRadioButton(groupBox_Axis);
        radioButton_z->setObjectName(QStringLiteral("radioButton_z"));
        radioButton_z->setFont(font1);
        radioButton_z->setChecked(false);

        horizontalLayout_2->addWidget(radioButton_z);


        verticalLayout_2->addWidget(groupBox_Axis);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        groupBox_ColorMode = new QGroupBox(centralwidget);
        groupBox_ColorMode->setObjectName(QStringLiteral("groupBox_ColorMode"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox_ColorMode->sizePolicy().hasHeightForWidth());
        groupBox_ColorMode->setSizePolicy(sizePolicy1);
        groupBox_ColorMode->setMinimumSize(QSize(230, 180));
        groupBox_ColorMode->setFont(font);
        verticalLayout = new QVBoxLayout(groupBox_ColorMode);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        radioButton_BlueRed = new QRadioButton(groupBox_ColorMode);
        radioButton_BlueRed->setObjectName(QStringLiteral("radioButton_BlueRed"));
        radioButton_BlueRed->setFont(font1);

        verticalLayout->addWidget(radioButton_BlueRed);

        radioButton_GreenMagenta = new QRadioButton(groupBox_ColorMode);
        radioButton_GreenMagenta->setObjectName(QStringLiteral("radioButton_GreenMagenta"));
        radioButton_GreenMagenta->setFont(font1);

        verticalLayout->addWidget(radioButton_GreenMagenta);

        radioButton_WhiteRed = new QRadioButton(groupBox_ColorMode);
        radioButton_WhiteRed->setObjectName(QStringLiteral("radioButton_WhiteRed"));
        radioButton_WhiteRed->setFont(font1);

        verticalLayout->addWidget(radioButton_WhiteRed);

        radioButton_GreyRed = new QRadioButton(groupBox_ColorMode);
        radioButton_GreyRed->setObjectName(QStringLiteral("radioButton_GreyRed"));
        radioButton_GreyRed->setFont(font1);

        verticalLayout->addWidget(radioButton_GreyRed);

        radioButton_Rainbow = new QRadioButton(groupBox_ColorMode);
        radioButton_Rainbow->setObjectName(QStringLiteral("radioButton_Rainbow"));
        radioButton_Rainbow->setFont(font1);
        radioButton_Rainbow->setChecked(true);

        verticalLayout->addWidget(radioButton_Rainbow);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        verticalLayout_2->addWidget(groupBox_ColorMode);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        pushButton_load = new QPushButton(centralwidget);
        pushButton_load->setObjectName(QStringLiteral("pushButton_load"));
        pushButton_load->setMinimumSize(QSize(50, 40));

        horizontalLayout->addWidget(pushButton_load);

        pushButton_save = new QPushButton(centralwidget);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));
        pushButton_save->setMinimumSize(QSize(50, 40));

        horizontalLayout->addWidget(pushButton_save);


        verticalLayout_2->addLayout(horizontalLayout);


        horizontalLayout_3->addLayout(verticalLayout_2);

        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(50);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy2);
        qvtkWidget->setMinimumSize(QSize(640, 480));

        horizontalLayout_3->addWidget(qvtkWidget);

        PCLViewer->setCentralWidget(centralwidget);
        QWidget::setTabOrder(pushButton_load, radioButton_z);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", Q_NULLPTR));
        groupBox_Axis->setTitle(QApplication::translate("PCLViewer", "Color on axis", Q_NULLPTR));
        radioButton_x->setText(QApplication::translate("PCLViewer", "X", Q_NULLPTR));
        radioButton_y->setText(QApplication::translate("PCLViewer", "Y", Q_NULLPTR));
        radioButton_z->setText(QApplication::translate("PCLViewer", "Z", Q_NULLPTR));
        groupBox_ColorMode->setTitle(QApplication::translate("PCLViewer", "Color mode", Q_NULLPTR));
        radioButton_BlueRed->setText(QApplication::translate("PCLViewer", "Blue to red", Q_NULLPTR));
        radioButton_GreenMagenta->setText(QApplication::translate("PCLViewer", "Green to magenta", Q_NULLPTR));
        radioButton_WhiteRed->setText(QApplication::translate("PCLViewer", "White to red", Q_NULLPTR));
        radioButton_GreyRed->setText(QApplication::translate("PCLViewer", "Grey / red", Q_NULLPTR));
        radioButton_Rainbow->setText(QApplication::translate("PCLViewer", "Rainbow", Q_NULLPTR));
        pushButton_load->setText(QApplication::translate("PCLViewer", "Load file", Q_NULLPTR));
        pushButton_save->setText(QApplication::translate("PCLViewer", "Save file", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
