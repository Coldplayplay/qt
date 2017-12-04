#ifndef GLOBAL_H
#define GLOBAL_H

#include <QMainWindow>
#include "pclviewer.h"//conditional filtering
#include "statisticfilter.h"
#include "rgbsegmentation.h"
#include "lccpseg.h"

namespace Ui {
class Global;
}

class Global : public QMainWindow
{
    Q_OBJECT

public:
    explicit Global(QWidget *parent = 0);
    ~Global();

public Q_SLOTS:
    void condButtonPressed();
    void statisticButtonPressed();
    void rgbButtonPressed();
    void lccpButtonPressed();

private:
    Ui::Global *ui;
    PCLViewer *cond;
    StatisticFilter *statisFilter;
    RGBSegmentation *rgbSeg;
    LCCPSeg *lccpSeg;
    
};

#endif // GLOBAL_H
