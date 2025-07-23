/*********************************************************************
 * Software License Agreement (Apache License, Version 2.0 )
 *
 * Copyright 2024 TPC Mechatronics Crop.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

 /* Author: DeokLae Kim (kdl79@tanhay.com)*/

#include "tm5_gui/mainwindow.hpp"



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setUiObject();
    initBtnJointPlus();
    initBtnJointMinus();
    initJointValueText();
    initBtnPoseJOg();
    viewJobList();
    connect(ui->list_job_name, &QListWidget::itemClicked, this, &MainWindow::jobListSelected);

    updateTimer = new QTimer(this);
    connect(updateTimer,&QTimer::timeout, this, &MainWindow::displayCureJointValue);
    updateTimer->start(100);

    spinBoxJointJogVelChanged(10);
    spinBoxPoseJogVelChanged(10);
    spinBoxJointTargetVelChanged(10);
    spinBoxJointTargetAccellChanged(10);

    Manipulation.setFrameidforTaskMsg(TpcManipulationMoveit::BASE_FRAME_ID);

    spinBoxJobMoveVelChanged(10);
    spinBoxJobMoveAccellChanged(10);

    /*std::thread(std::bind(&Moveit_Servo::spin, &moveitServo)).detach();
    moveitServo.connect_moveit_servo();
    moveitServo.start_moveit_servo();*/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setUiObject()
{
    textBrJointValues.append(ui->joint_act_value_1);
    textBrJointValues.append(ui->joint_act_value_2);
    textBrJointValues.append(ui->joint_act_value_3);
    textBrJointValues.append(ui->joint_act_value_4);
    textBrJointValues.append(ui->joint_act_value_5);
    textBrJointValues.append(ui->joint_act_value_6);

    textPositionValues.append(ui->pose_x_value);
    textPositionValues.append(ui->pose_y_value);
    textPositionValues.append(ui->pose_z_value);
    textPositionValues.append(ui->orient_x_value);
    textPositionValues.append(ui->orient_y_value);
    textPositionValues.append(ui->orient_z_value);
    textPositionValues.append(ui->orient_w_value);

    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_1);
    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_2);
    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_3);
    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_4);
    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_5);
    robot_joint_names.push_back(TpcManipulationMoveit::JOINT_NAME_6);

    btnPosePlus.append(ui->btn_position_move_x_plus);
    btnPosePlus.append(ui->btn_position_move_y_plus);
    btnPosePlus.append(ui->btn_position_move_z_plus);
    btnPosePlus.append(ui->btn_rotation_move_x_plus);
    btnPosePlus.append(ui->btn_rotation_move_y_plus);
    btnPosePlus.append(ui->btn_rotation_move_z_plus);

    btnPoseMinsu.append(ui->btn_position_move_x_minus);
    btnPoseMinsu.append(ui->btn_position_move_y_minus);
    btnPoseMinsu.append(ui->btn_position_move_z_minus);
    btnPoseMinsu.append(ui->btn_rotation_move_x_minus);
    btnPoseMinsu.append(ui->btn_rotation_move_y_minus);
    btnPoseMinsu.append(ui->btn_rotation_move_z_minus);

    textTargetJoint.append(ui->lineEdit_joint_target_value_1);
    textTargetJoint.append(ui->lineEdit_joint_target_value_2);
    textTargetJoint.append(ui->lineEdit_joint_target_value_3);
    textTargetJoint.append(ui->lineEdit_joint_target_value_4);
    textTargetJoint.append(ui->lineEdit_joint_target_value_5);
    textTargetJoint.append(ui->lineEdit_joint_target_value_6);

    textTargetPose.append(ui->lineEdit_pose_target_x_value);
    textTargetPose.append(ui->lineEdit_pose_target_y_value);
    textTargetPose.append(ui->lineEdit_pose_target_z_value);
    textTargetPose.append(ui->lineEdit_orient_target_x_value);
    textTargetPose.append(ui->lineEdit_orient_target_y_value);
    textTargetPose.append(ui->lineEdit_orient_target_z_value);
    textTargetPose.append(ui->lineEdit_orient_target_w_value);

    // joint jog frame //
    connect(ui->spinBox_joint_jog_vel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxJointJogVelChanged);
    ui->spinBox_joint_jog_vel->setValue(10);
    connect(ui->spinBox_joint_target_vel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxJointTargetVelChanged);
    ui->spinBox_joint_target_vel->setValue(10);
    connect(ui->spinBox_joint_target_accell, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxJointTargetAccellChanged);
    ui->spinBox_joint_target_accell->setValue(10);
    connect(ui->btn_move_target_joint, &QPushButton::pressed, this, &MainWindow::btnMoveTargetJointPressed);
    connect(ui->btn_stop_target_joint, &QPushButton::pressed, this, [this](){
       Manipulation.moveTargetJointStop();
    });
    // joint jog frame //

    // pose jog frame //
    connect(ui->cmb_coord_select, &QComboBox::currentTextChanged, this, &MainWindow::cmbCoordSelectChanged);
    ui->cmb_coord_select->setCurrentIndex(0);
    connect(ui->spinBox_pose_jog_vel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxPoseJogVelChanged);
    ui->spinBox_pose_jog_vel->setValue(10);
    connect(ui->btn_move_target_pose, &QPushButton::clicked, this, &MainWindow::btnMoveTargetPosePressed);
    connect(ui->spinBox_pose_target_vel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxPoseTargetVelChanged);
    ui->spinBox_pose_target_vel->setValue(10);
    connect(ui->spinBox_pose_target_accell, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxPoseTargetAccellChanged);
    ui->spinBox_pose_target_accell->setValue(10);
    connect(ui->btn_stop_target_pose, &QPushButton::pressed, this, [this](){
        Manipulation.moveTargetPoseStop();
    });
    // pose jog frame //

    //job move//
    connect(ui->btn_save_joints, &QPushButton::clicked, this, &MainWindow::btnSaveJointsPressed);
    connect(ui->btn_save_pose, &QPushButton::clicked, this, &MainWindow::btnSavePosePressed);
    connect(ui->btn_save_waypoint, &QPushButton::clicked, this, &MainWindow::btnSaveWaypointPressed);
    connect(ui->btn_move_job_start, &QPushButton::clicked, this, &MainWindow::btnMoveJobStartPressed);
    connect(ui->btn_move_job_pause, &QPushButton::clicked, this,[this](){
        Manipulation.moveJobPause();
    });
    connect(ui->btn_move_job_stop, &QPushButton::clicked, this,[this](){
        Manipulation.moveJobStop();
    });
    connect(ui->btn_move_job_forward, &QPushButton::pressed, this, &MainWindow::btnMoveJobStartPressed);
    connect(ui->btn_move_job_forward, &QPushButton::released, this,[this](){
        Manipulation.moveJobPause();
    });
    connect(ui->btn_move_job_back, &QPushButton::pressed, this, &MainWindow::btnMoveJobBackPressed);
    connect(ui->btn_move_job_back, &QPushButton::released, this, [this](){
        Manipulation.moveJobPause();
    });
    connect(ui->btn_move_job_next_step, &QPushButton::pressed, this, &MainWindow::btnMoveJobNextStepPressed);
    connect(ui->btn_move_job_next_step, &QPushButton::released, this, [this](){
        Manipulation.moveJobStepStop();
    });
    connect(ui->btn_move_job_previous_step, &QPushButton::pressed, this, &MainWindow::btnMoveJobPreviousStepPressed);
    connect(ui->btn_move_job_previous_step, &QPushButton::released, this, [this]()
    {
        Manipulation.moveJobStepStop();
    });
    connect(ui->spinBox_job_move_vel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::spinBoxJobMoveVelChanged);
    ui->spinBox_job_move_vel->setValue(10);
    connect(ui->spinBox_job_move_accell, QOverload<int>::of(&QSpinBox::valueChanged), this ,&MainWindow::spinBoxJobMoveAccellChanged);
    ui->spinBox_job_move_accell->setValue(10);
    //job move//

    //arc move//
    cmb_arc_direct.append(ui->cmb_arc_dir_x);
    cmb_arc_direct.append(ui->cmb_arc_dir_y);
    cmb_arc_direct.append(ui->cmb_arc_dir_z);
    for(int i = 0; i < cmb_arc_direct.size(); ++i)
    {
        cmb_arc_direct[i]->addItem("+", 1);
        cmb_arc_direct[i]->addItem("0", 0);
        cmb_arc_direct[i]->addItem("-", -1);
    }
    ui->cmb_coord_select_arc->setCurrentIndex(0);

    connect(ui->btn_move_arc, &QPushButton::pressed, this, &MainWindow::btnMoveArcPressed);
    connect(ui->btn_move_arc, &QPushButton::released, this, [this]()
    {
       Manipulation.moveArcWaypointStop();
    });

    connect(ui->btn_save_arc, &QPushButton::clicked, this, &MainWindow::btnSaveArcPressed);
    //arc move//
}

void MainWindow::initBtnJointPlus()
{
    for(size_t i = 0; i < 6; ++i)
    {
        QString pushBtnName = QString("btn_joint_plus_%1").arg(i+1);
        QPushButton* pushBtn = findChild<QPushButton*>(pushBtnName);
        if(pushBtn)
        {
            connect(pushBtn, &QPushButton::pressed, this, &MainWindow::btnJointPlusPressed);
            connect(pushBtn, &QPushButton::released, this, &MainWindow::btnJointPlusReleased);
        } else{
            qWarning() << "pushBtnName" << pushBtnName << "not found";
        }
    }
}

void MainWindow::initBtnJointMinus()
{
    for(size_t i = 0; i < 6; ++i)
    {
        QString pushBtnName = QString("btn_joint_minus_%1").arg(i+1);
        QPushButton* pushBtn = findChild<QPushButton*>(pushBtnName);
        if(pushBtn)
        {
            connect(pushBtn, &QPushButton::pressed, this, &MainWindow::btnJointMinusPressed);
            connect(pushBtn, &QPushButton::released, this, &MainWindow::btnJointMinusReleased);
        } else{
            qWarning() << "pushBtnName" << pushBtnName << "not found";
        }
    }
}

void MainWindow::initBtnPoseJOg()
{
    for(int i = 0; i < btnPosePlus.size(); ++i)
    {
        connect(btnPosePlus[i], &QPushButton::pressed, this,[this,i](){
            Manipulation.moveTaskJog(i,arm_twist_vel_);
        });

        connect(btnPoseMinsu[i], &QPushButton::pressed, this,[this,i](){
            Manipulation.moveTaskJog(i,-arm_twist_vel_);
        });

        connect(btnPosePlus[i], &QPushButton::released, this,[this](){
            Manipulation.stopTaskJog();
        });
        connect(btnPoseMinsu[i], &QPushButton::released, this, [this](){
            Manipulation.stopTaskJog();
        });
    }
}

void MainWindow::initJointValueText()
{
    for(size_t i = 0; i < 6; ++i)
    {
        QString textBrName = QString("joint_act_value_%1").arg(i+1);
        QTextBrowser* browser = findChild<QTextBrowser*>(textBrName);
        if(browser)
        {
            browser->setText("No Data!!");
        } else{
            qWarning() << "textBrName" << textBrName << "not found";
        }
    }
}

void MainWindow::btnJointPlusPressed()
{
    QPushButton *pushBtn = qobject_cast<QPushButton *>(sender());
    if(pushBtn)
    {
        QString BtnName = pushBtn->objectName();
        qDebug() << BtnName << "pressed";
        QChar btnNumChar = BtnName.back();
        int btnNum = btnNumChar.digitValue();
        Manipulation.moveJointJog(robot_joint_names[btnNum-1], arm_joint_vel_);
    }
}

void MainWindow::btnJointPlusReleased()
{
    QPushButton *pushBtn = qobject_cast<QPushButton *>(sender());
    if(pushBtn)
    {
        QString BtnName = pushBtn->objectName();
        Manipulation.stopJointJog();
    }
}

void MainWindow::btnJointMinusPressed()
{
  QPushButton *pushBtn = qobject_cast<QPushButton *>(sender());
  if(pushBtn)
  {
      QString BtnName = pushBtn->objectName();
      qDebug() << BtnName << "pressed";
      QChar btnNumChar = BtnName.back();
      int btnNum = btnNumChar.digitValue();
      Manipulation.moveJointJog(robot_joint_names[btnNum-1], -arm_joint_vel_);
  }
}

void MainWindow::btnJointMinusReleased()
{
    QPushButton *pushBtn = qobject_cast<QPushButton *>(sender());
    if(pushBtn)
    {
        QString BtnName = pushBtn->objectName();
        qDebug() << BtnName << "released";
        Manipulation.stopJointJog();
    }
}

void MainWindow::btnMoveTargetJointPressed()
{
    std::vector<double> tmpTarget;
    double tempNum;
    for(int i = 0; i < textTargetJoint.size(); ++i)
    {
        tempNum = textTargetJoint[i]->text().toDouble();
        tmpTarget.emplace_back(tempNum);
    }
    spinBoxJointTargetVelChanged(ui->spinBox_joint_target_vel->value());
    spinBoxJointTargetAccellChanged(ui->spinBox_joint_target_accell->value());
    std::thread moveTargetJointThread([=](){
        Manipulation.movetargetjointStart(tmpTarget);
    });
    moveTargetJointThread.detach();
}

void MainWindow::displayCureJointValue()
{
    for(size_t i = 0; i < 6; ++i)
    {
        textBrJointValues[i]->setText(QString::number(Manipulation.cure_joint_values_[i]));
    }

    textPositionValues[0]->setText(QString::number(Manipulation.cure_pose_values_.pose.position.x));
    textPositionValues[1]->setText(QString::number(Manipulation.cure_pose_values_.pose.position.y));
    textPositionValues[2]->setText(QString::number(Manipulation.cure_pose_values_.pose.position.z));
    textPositionValues[3]->setText(QString::number(Manipulation.cure_pose_values_.pose.orientation.x));
    textPositionValues[4]->setText(QString::number(Manipulation.cure_pose_values_.pose.orientation.y));
    textPositionValues[5]->setText(QString::number(Manipulation.cure_pose_values_.pose.orientation.z));
    textPositionValues[6]->setText(QString::number(Manipulation.cure_pose_values_.pose.orientation.w));
}

void MainWindow::spinBoxJointJogVelChanged(int value)
{
    arm_joint_vel_ = TpcManipulationMoveit::ARM_JOINT_VEL * (value/100.0);
}

void MainWindow::spinBoxJointTargetVelChanged(int value)
{
    Manipulation.joint_velocity_scale_ = TpcManipulationMoveit::JOINT_VELOCITY_SCALE_MAX * (value/100.0);
}

void MainWindow::spinBoxJointTargetAccellChanged(int value)
{
    Manipulation.joint_acceleration_scale_ = TpcManipulationMoveit::jOINT_ACCELERATION_SCALE_MAX * (value/100.0);
}

void MainWindow::cmbCoordSelectChanged(const QString &base_name)
{
    if(base_name == "BASE")
    {
        Manipulation.setFrameidforTaskMsg(TpcManipulationMoveit::BASE_FRAME_ID);
    }else
    {
        Manipulation.setFrameidforTaskMsg(TpcManipulationMoveit::TOOL_FRAME_ID);
    }
}

void MainWindow::spinBoxPoseJogVelChanged(int value)
{
     arm_twist_vel_ = TpcManipulationMoveit::ARM_TWIST_VEL * (value / 100.0);
}

void MainWindow::spinBoxPoseTargetVelChanged(int value)
{
    Manipulation.cartesian_velocity_scale_ = TpcManipulationMoveit::CARTE_VELOCITY_SCALE_MAX * (value / 100.0);
}

void MainWindow::spinBoxPoseTargetAccellChanged(int value)
{
    Manipulation.cartesian_acceleration_scale_ = TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_MAX * (value / 100.0);
}

void MainWindow::btnMoveTargetPosePressed()
{
    spinBoxPoseTargetVelChanged(ui->spinBox_pose_target_vel->value());
    spinBoxPoseTargetAccellChanged(ui->spinBox_pose_target_accell->value());

    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.pose.position.x = textTargetPose[0]->text().toDouble();
    targetPose.pose.position.y = textTargetPose[1]->text().toDouble();
    targetPose.pose.position.z = textTargetPose[2]->text().toDouble();
    targetPose.pose.orientation.x = textTargetPose[3]->text().toDouble();
    targetPose.pose.orientation.y = textTargetPose[4]->text().toDouble();
    targetPose.pose.orientation.z = textTargetPose[5]->text().toDouble();
    targetPose.pose.orientation.w = textTargetPose[6]->text().toDouble();

    Manipulation.moveTargetPoseStart(targetPose);
}

void MainWindow::btnSaveJointsPressed()
{
    if(ui->txtBr_job_name->toPlainText() != "")
    {
        setCurrentJobName();
        Manipulation.saveCureJointTojob(cure_job_name_);
    }else
    {
        qDebug() << "Warning!! empty job name!!!";
        QMessageBox::warning(this, "Warning", "Enter the Job Name!!");
    }
    viewJobList();
}

void MainWindow::btnSavePosePressed()
{
    if(ui->txtBr_job_name->toPlainText() != "")
    {
        setCurrentJobName();
        Manipulation.saveCurePoseToJob(cure_job_name_);
    }else
    {
        qDebug() << "Warning!! empty job name!!!";
        QMessageBox::warning(this, "Warning", "Enter the Job Name!!");
    }
    viewJobList();
}

void MainWindow::btnSaveWaypointPressed()
{
    if(ui->txtBr_job_name->toPlainText() != "")
    {
        setCurrentJobName();
        Manipulation.saveCureWapointToJob(cure_job_name_);
    }else
    {
        qDebug() << "Warning!! empty job name!!!";
        QMessageBox::warning(this, "Warning", "Enter the Job Name!!");
    }
    viewJobList();
}

void MainWindow::viewJobList()
{
    ui->list_job_name->clear();
    QDir directory("jobfiles");

    QStringList files = directory.entryList(QDir::Files);
    for (const QString &file : files)
    {
        ui->list_job_name->addItem(file);
    }
}

void MainWindow::jobListSelected(QListWidgetItem *item)
{
    QString selectedFile = item->text();
    ui->txtBr_job_name->setText(selectedFile);
    setCurrentJobName();
}

void MainWindow::setCurrentJobName()
{
    size_t i = ui->txtBr_job_name->toPlainText().toStdString().find_last_of(".");
    std::string jobName = ui->txtBr_job_name->toPlainText().toStdString().substr(0, i);
    cure_job_name_ = jobName;
}

void MainWindow::btnMoveJobStartPressed()
{
    spinBoxJobMoveVelChanged(ui->spinBox_job_move_vel->value());
    spinBoxJobMoveAccellChanged(ui->spinBox_job_move_accell->value());
    Manipulation.readJobValuesFromFile(cure_job_name_);
    Manipulation.moveJobStart();
}

void MainWindow::btnMoveJobBackPressed()
{
    spinBoxJobMoveVelChanged(ui->spinBox_job_move_vel->value());
    spinBoxJobMoveAccellChanged(ui->spinBox_job_move_accell->value());
    Manipulation.readJobValuesFromFile(cure_job_name_);
    Manipulation.moveJobStartBack();
}

void MainWindow::spinBoxJobMoveVelChanged(int value)
{
    Manipulation.cartesian_velocity_scale_ = TpcManipulationMoveit::CARTE_VELOCITY_SCALE_MAX * (value/100.0);
    Manipulation.joint_velocity_scale_ = TpcManipulationMoveit::JOINT_VELOCITY_SCALE_MAX * (value/100.0);
}

void MainWindow::spinBoxJobMoveAccellChanged(int value)
{
    Manipulation.cartesian_acceleration_scale_ = TpcManipulationMoveit::CARTE_ACCELERATION_SCALE * (value/100.0);
    Manipulation.joint_acceleration_scale_ = TpcManipulationMoveit::jOINT_ACCELERATION_SCALE_MAX * (value/100.0);
}

void MainWindow::btnMoveJobNextStepPressed()
{
    spinBoxJobMoveAccellChanged(ui->spinBox_job_move_accell->value());
    spinBoxJobMoveVelChanged(ui->spinBox_job_move_vel->value());
    Manipulation.readJobValuesFromFile(cure_job_name_);
    Manipulation.moveJobStep(true);
}

void MainWindow::btnMoveJobPreviousStepPressed()
{
    spinBoxJobMoveAccellChanged(ui->spinBox_job_move_accell->value());
    spinBoxJobMoveVelChanged(ui->spinBox_job_move_vel->value());
    Manipulation.readJobValuesFromFile(cure_job_name_);
    Manipulation.moveJobStep(false);
}

void MainWindow::btnMoveArcPressed()
{
    double radius = ui->lineEdit_arc_radius->text().toDouble();
    int direc_x = ui->cmb_arc_dir_x->currentData().toInt();
    int direc_y = ui->cmb_arc_dir_y->currentData().toInt();
    int direc_z = ui->cmb_arc_dir_z->currentData().toInt();
    int point_num = ui->lineEdit_arc_pointNum->text().toInt();
    double angle = ui->lineEdit_arc_angle->text().toDouble();
    if(ui->cmb_coord_select_arc->currentText() == "BASE")
    {
        //std::string frame_name = TpcManipulationMoveit::BASE_FRAME_ID;
        Manipulation.moveArcWaypoints(radius, point_num, angle, direc_x, direc_y, direc_z, TpcManipulationMoveit::BASE_FRAME_ID);
    } else if(ui->cmb_coord_select_arc->currentText() == "TOOL")
    {
        Manipulation.moveArcWaypoints(radius, point_num, angle, direc_x, direc_y, direc_z, TpcManipulationMoveit::TOOL_FRAME_ID);
    } else
    {
        QMessageBox::warning(this, "Warning", "Check FRAME ID Value!!!");
    }


}

void MainWindow::btnSaveArcPressed()
{
    if(ui->txtBr_job_name->toPlainText() != "")
    {
        setCurrentJobName();
        Manipulation.saveArcWaypoits(cure_job_name_);
    }else
    {
        qDebug() << "Warning!! empty job name!!!";
        QMessageBox::warning(this, "Warning", "Enter the Job Name!!");
    }
    viewJobList();
}
