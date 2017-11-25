#ifndef GLOBAL_H
#define GLOBAL_H

#include <QMainWindow>
#include "pclviewer.h"
#include "rgbsegmentation.h"
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
    void segButtonPressed();


private:
    Ui::Global *ui;
    PCLViewer *view;
    RGBSegmentation *seg;
};

#endif // GLOBAL_H
