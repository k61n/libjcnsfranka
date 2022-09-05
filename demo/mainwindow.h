#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <jcnsfranka.h>
#include "qspinbox.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_connectBtn_clicked();
    void on_stateChanged();
    void on_homeBtn_clicked();
    void on_clearBtn_clicked();
    void on_readBtn_clicked();
    void on_gripperBtn_clicked();
    void on_cmtestBtn_clicked();

private:
    Ui::MainWindow *ui;
    JcnsFranka *robot;
    QList<QSpinBox *> boxList;
};
#endif // MAINWINDOW_H
