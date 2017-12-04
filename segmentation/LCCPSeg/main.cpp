#include "lccpseg.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    LCCPSeg w;
    w.show();

    return a.exec();
}
