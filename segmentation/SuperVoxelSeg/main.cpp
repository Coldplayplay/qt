#include "supervoxelseg.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SuperVoxelSeg w;
    w.show();

    return a.exec();
}
