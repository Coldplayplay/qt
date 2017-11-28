#include "statisticfilter.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    StatisticFilter w;
    w.show();

    return a.exec();
}
