#include "voxelfilter.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    VoxelFilter w;
    w.show();

    return a.exec();
}
