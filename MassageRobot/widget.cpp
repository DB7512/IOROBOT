#include "widget.h"
#include "ui_widget.h"
#include "robotcontrol.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_clicked()
{
    while(!GetRobotControlInstance()->m_connectstate) {
        GetRobotControlInstance()->ConnectRobot();
        sleep_milliseconds(500);
    }
    while(!GetRobotControlInstance()->m_enablestate) {
        GetRobotControlInstance()->EnableRobot(true);
        sleep_milliseconds(10000);
    }
    while(!GetRobotControlInstance()->m_massagestate) {
        GetRobotControlInstance()->SetMassage(true);
        qDebug("2");
        sleep_milliseconds(500);
    }
}

