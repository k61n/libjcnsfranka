#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->j1Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j2Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j3Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j4Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j5Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j6Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);
    connect(ui->j7Box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);

    this->on_connectBtn_clicked();

}

MainWindow::~MainWindow()
{
    delete this->robot;
    delete ui;
}


void MainWindow::on_stateChanged()
{
    double j1 = (double)ui->j1Box->value() / 180 * M_PI;
    double j2 = (double)ui->j2Box->value() / 180 * M_PI;
    double j3 = (double)ui->j3Box->value() / 180 * M_PI;
    double j4 = (double)ui->j4Box->value() / 180 * M_PI;
    double j5 = (double)ui->j5Box->value() / 180 * M_PI;
    double j6 = (double)ui->j6Box->value() / 180 * M_PI;
    double j7 = (double)ui->j7Box->value() / 180 * M_PI;
    this->robot->moveJoints(j1, j2 ,j3, j4, j5, j6, j7);
}


void MainWindow::on_connectBtn_clicked()
{
    std::string ip = ui->ipLine->text().toStdString();
    this->robot = new JcnsFranka(ip);
}



void MainWindow::on_homeBtn_clicked()
{
    robot->goHome();
}


void MainWindow::on_clearBtn_clicked()
{
    ui->connectionEdit->clear();
}


void MainWindow::on_readBtn_clicked()
{
    QString output = QString::fromStdString(robot->readState());
    ui->connectionEdit->appendPlainText(output);

}
