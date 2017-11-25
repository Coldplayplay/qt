#ifndef SUPERVOXELSEG_H
#define SUPERVOXELSEG_H

#include <QMainWindow>

namespace Ui {
class SuperVoxelSeg;
}

class SuperVoxelSeg : public QMainWindow
{
    Q_OBJECT

public:
    explicit SuperVoxelSeg(QWidget *parent = 0);
    ~SuperVoxelSeg();

public Q_SLOTS:


private:
    Ui::SuperVoxelSeg *ui;
};

#endif // SUPERVOXELSEG_H
