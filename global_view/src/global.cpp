#include "global.h"
#include "../build/ui_global.h"

#include <QApplication>

Global::Global(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Global)
{
    ui->setupUi(this);
    connect(ui->pushButton_cond, SIGNAL(clicked(bool)), this, SLOT(condButtonPressed()));
    connect(ui->pushButton_sta, SIGNAL(clicked(bool)), this, SLOT(statisticButtonPressed()));
    connect(ui->pushButton_rgb, SIGNAL(clicked(bool)), this, SLOT(rgbButtonPressed()));
    connect(ui->pushButton_lccp, SIGNAL(clicked(bool)), this, SLOT(lccpButtonPressed()));
}


void Global::condButtonPressed()
{

    cond = new PCLViewer(this) ;
    cond->show();

}

void Global::statisticButtonPressed()
{
    statisFilter = new StatisticFilter(this);
    statisFilter->show();
}

void Global::rgbButtonPressed()
{

    rgbSeg = new RGBSegmentation(this) ;
    rgbSeg->show();

}


void Global::lccpButtonPressed()
{

    lccpSeg = new LCCPSeg(this) ;
    lccpSeg->show();

}

Global::~Global()
{
    delete ui;
}
