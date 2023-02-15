#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
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


void MainWindow::on_connectBtn_clicked()
{
    std::string ip = ui->ipLine->text().toStdString();
    this->robot = new JcnsFranka::Robot(ip.data());
    check_error();
}


void MainWindow::on_setloadBtn_clicked()
{
    QJsonDocument doc;
    QString filename = QFileDialog::getOpenFileName(this, "Select JSON file", "", "JSON files (*.json)");
    if (!filename.isEmpty()) {
        ui->setloadLabel->setText(filename.split('/').last());
        QFile file(filename);
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        doc = QJsonDocument::fromJson(file.readAll());
    }
    QJsonObject obj = doc.object();

    double mass = obj["mass"].toDouble();
    QJsonArray arg2 = obj["F_x_Cload"].toArray();
    std::array<double, 3> F_x_Cload;
    for (int i = 0; i < arg2.size(); i++)
        F_x_Cload[i] = arg2[i].toDouble();
    QJsonArray arg3 = obj["load_inertia"].toArray();
    std::array<double, 9> load_inertia;
    for (int i = 0; i < arg3.size(); i++)
        load_inertia[i] = arg3[i].toDouble();
    robot->set_load(mass, F_x_Cload, load_inertia);
    check_error();
}


void MainWindow::on_stateChanged()
{
    std::array<double, 7> joints;
    for (int i=0; i<boxList.count(); i++) {
        joints[i] = boxList.at(i)->value() / 180.0 * M_PI;
    }
    this->robot->moveJoints(joints);
    check_error();
}


void MainWindow::on_homeBtn_clicked()
{
    robot->goHome();
    check_error();
}


void MainWindow::on_clearBtn_clicked()
{
    ui->connectionEdit->clear();
}


void MainWindow::on_readBtn_clicked()
{
    ui->connectionEdit->clear();
    JcnsFranka::Coordinates state = robot->readState();
    check_error();
    for (int i=0; i<state.joints.max_size(); i++) {
        double degrees = state.joints[i] / M_PI * 180;
        ui->connectionEdit->appendPlainText("j" + QString::number(i) + " = " + QString::number(state.joints[i], 'g', 6) + "\t" + QString::number((int)degrees));
    }
    ui->connectionEdit->appendPlainText("");
    ui->connectionEdit->appendPlainText("X = " + QString::number(state.xyz[0], 'g', 3));
    ui->connectionEdit->appendPlainText("Y = " + QString::number(state.xyz[1], 'g', 3));
    ui->connectionEdit->appendPlainText("Z = " + QString::number(state.xyz[2], 'g', 3));
}


void MainWindow::on_cmtestBtn_clicked()
{
    std::string ip = ui->ipLine->text().toStdString();
    JcnsFranka::communicationTest(ip.data());
}


void MainWindow::on_xPlusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(value, 0, 0);
    check_error();
}


void MainWindow::on_xMinusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(-value, 0, 0);
    check_error();
}


void MainWindow::on_yPlusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(0, value, 0);
    check_error();
}


void MainWindow::on_yMinusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(0, -value, 0);
    check_error();
}


void MainWindow::on_zPlusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(0, 0, value);
    check_error();
}


void MainWindow::on_zMinusBtn_clicked()
{
    double value = ui->displacementLine->text().toDouble();
    robot->moveRelative(0, 0, -value);
    check_error();
}


void MainWindow::on_moveBtn_clicked()
{
    double x = ui->xLn->text().toDouble();
    double y = ui->yLn->text().toDouble();
    double z = ui->zLn->text().toDouble();
    robot->moveAbsolute(x, y, z);
    check_error();
}


void MainWindow::on_closegripperBtn_clicked()
{
    double width = ui->widthcloseLn->text().toDouble();
    double force = ui->forceLn->text().toDouble();
    robot->close_gripper(width, force);
    check_error();
}


void MainWindow::on_opengripperBtn_clicked()
{
    double width = ui->widthopenLn->text().toDouble();
    robot->open_gripper(width);
    check_error();
}

void MainWindow::check_error()
{
    QString error = QString::fromUtf8(robot->read_error());
    if (error != "")
        qDebug() << error;
        robot->reset_error();
}
