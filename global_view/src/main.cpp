#include "global.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Global w;
    w.show();

    return a.exec();
}
