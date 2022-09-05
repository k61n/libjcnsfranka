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

    boxList.append(ui->j1Box);
    boxList.append(ui->j2Box);
    boxList.append(ui->j3Box);
    boxList.append(ui->j4Box);
    boxList.append(ui->j5Box);
    boxList.append(ui->j6Box);
    boxList.append(ui->j7Box);

    foreach (const QSpinBox *box, boxList)
        connect(box, &QSpinBox::valueChanged, this, &MainWindow::on_stateChanged);

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
    ui->connectionEdit->clear();
    franka::RobotState state = this->robot->readState();


//    QJsonDocument state = QJsonDocument::fromJson(QString::fromStdString(this->robot->readState()).toUtf8());
//    QList joint_positions = state.object().toVariantMap()["q"].toStringList();
    for (int i=0; i<state.q.max_size(); i++) {
        double degrees = state.q[i] / M_PI * 180;
        ui->connectionEdit->appendPlainText("j" + QString::number(i) + " = " + QString::number(state.q[i]) + "\t" + QString::number((int)degrees));
    }
}


void MainWindow::on_gripperBtn_clicked()
{
    if (robot->isGripping())
        robot->release();
    else robot->grasp();
}
