#include "filter_gui.h"
#include <QtWidgets/QApplication>
#include <QMainWindow>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    Filter_Gui w;
    w.show();
    return a.exec();
}
