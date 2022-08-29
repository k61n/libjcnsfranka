#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QJsonDocument>
#include <QJsonObject>
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

    boxList.append(ui->j1Box);
    boxList.append(ui->j2Box);
    boxList.append(ui->j3Box);
    boxList.append(ui->j4Box);
    boxList.append(ui->j5Box);
    boxList.append(ui->j6Box);
    boxList.append(ui->j7Box);

    this->on_connectBtn_clicked();
}

MainWindow::~MainWindow()
{
    delete this->robot;
    delete ui;
}


void MainWindow::on_stateChanged()
{
    std::array<double, 7> joints;
    for (int i=0; i<boxList.count(); i++) {
        joints[i] = boxList.at(i)->value() / 180.0 * M_PI;
        qDebug() << i << boxList.at(i)->value() << joints[i];
    }
    this->robot->moveJoints(joints);

    QJsonDocument state = QJsonDocument::fromJson(QString::fromStdString(this->robot->readState()).toUtf8());
    QList joint_positions = state.object().toVariantMap()["q"].toStringList();
    qDebug() << joint_positions;

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
