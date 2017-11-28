#ifndef GLOBAL_H
#define GLOBAL_H

#include <QMainWindow>
#include "pclviewer.h"
#include "rgbsegmentation.h"
#include "statisticfilter.h"
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
    void segButtonPressed();


private:
    Ui::Global *ui;
    PCLViewer *view;
    StatisticFilter *statisFilter;
    RGBSegmentation *seg;
};

#endif // GLOBAL_H
