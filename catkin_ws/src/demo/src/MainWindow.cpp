#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setFocusPolicy(Qt::ClickFocus);

    QPixmap pix_steering(":/images/SteeringWheel.png");
    QPixmap pix_speed   (":/images/Speedometer.png"  );
    this->pix_steering_wheel = pix_steering;
    this->pix_speedometer    = pix_speed;
    ui->lblSteeringWheel->setPixmap(pix_steering_wheel);
    ui->lblSpeedometer  ->setPixmap(pix_speedometer);
}

MainWindow::~MainWindow()
{    
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    QPixmap pix_steering = this->pix_steering_wheel;
    QTransform tr_steering;
    tr_steering.rotate(-qtRosNode->current_steering*180/M_PI*10);
    pix_steering = pix_steering.transformed(tr_steering, Qt::SmoothTransformation);
    ui->lblSteeringWheel->setAlignment(Qt::AlignCenter);
    ui->lblSteeringWheel->setPixmap(pix_steering);

    QPixmap pix_speed = this->pix_speedometer;
    QPainter painter(&pix_speed);
    painter.setPen(QPen(Qt::black, 5));
    int speed_line_x = 128 + 75*cos(120.0/180*M_PI + fabs(qtRosNode->current_speed/100.0*270/180*M_PI));
    int speed_line_y = 128 + 75*sin(120.0/180*M_PI + fabs(qtRosNode->current_speed/100.0*270/180*M_PI));
    painter.drawLine(128,128,speed_line_x, speed_line_y);
    ui->lblSpeedometer->setPixmap(pix_speed);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_Up:
        qtRosNode->update_speed_and_publish(0.5);
        break;
    case Qt::Key_Down:
        qtRosNode->update_speed_and_publish(-0.5);
        break;
    case Qt::Key_Left:
        qtRosNode->update_steering_and_publish(0.01);
        break;
    case Qt::Key_Right:
        qtRosNode->update_steering_and_publish(-0.01);
        break;
    case Qt::Key_Space:
        qtRosNode->publish_speed(0);
        qtRosNode->publish_steering(0);
        break;
    }
}

