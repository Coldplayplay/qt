#include "supervoxelseg.h"
#include "ui_supervoxelseg.h"

SuperVoxelSeg::SuperVoxelSeg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SuperVoxelSeg)
{
    ui->setupUi(this);
}

SuperVoxelSeg::~SuperVoxelSeg()
{
    delete ui;
}
