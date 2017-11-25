#include "global.h"
#include "../build/ui_global.h"

#include <QApplication>

Global::Global(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Global)
{
    ui->setupUi(this);
    connect(ui->pushButton_cond, SIGNAL(clicked(bool)), this, SLOT(condButtonPressed()));
    connect(ui->pushButton_seg, SIGNAL(clicked(bool)), this, SLOT(segButtonPressed()));
}


void Global::condButtonPressed()
{

    view = new PCLViewer(this) ;
    view->show();

}

void Global::segButtonPressed()
{

    seg = new RGBSegmentation(this) ;
    seg->show();

}

Global::~Global()
{
    delete ui;
}
