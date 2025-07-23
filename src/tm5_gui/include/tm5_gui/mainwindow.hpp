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

// Include guards
#ifndef TM5_GUI_MAINWINDOW_HPP
#define TM5_GUI_MAINWINDOW_HPP

#include <QMainWindow>
#include "./ui_mainwindow.h"
#include "tpc_manipulation_moveit.hpp"
#include "moveit_servo.hpp"
#include <QDebug>
#include <QTimer>
#include <QMessageBox>
#include <list>
#include <QDir>
#include <QObject>
#include <QThread>

class MainWindow : public QMainWindow {
    Q_OBJECT   //QT의 메타 오브젝트 시스템을 위한 매크로

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
signals:
    void requestUpdateCureJointValue();

private slots:
    void btnJointPlusPressed();
    void btnJointPlusReleased();
    void btnJointMinusPressed();
    void btnJointMinusReleased();
    void btnMoveTargetJointPressed();
    void spinBoxJointJogVelChanged(int value);
    void spinBoxJointTargetVelChanged(int value);
    void spinBoxJointTargetAccellChanged(int value);
    void cmbCoordSelectChanged(const QString &base_name);
    void spinBoxPoseJogVelChanged(int value);
    void spinBoxPoseTargetVelChanged(int value);
    void spinBoxPoseTargetAccellChanged(int value);
    void btnSaveJointsPressed();
    void btnSavePosePressed();
    void btnSaveWaypointPressed();
    void jobListSelected(QListWidgetItem *item);
    void btnMoveJobStartPressed();
    void btnMoveJobBackPressed();
    void spinBoxJobMoveVelChanged(int value);
    void spinBoxJobMoveAccellChanged(int value);
    void btnMoveJobNextStepPressed();
    void btnMoveJobPreviousStepPressed();
    void btnMoveTargetPosePressed();
    void btnMoveArcPressed();
    void btnSaveArcPressed();

private:
    // UI components
    Ui::MainWindow *ui;
    double arm_joint_vel_;
    double arm_twist_vel_;
    std::string cure_job_name_;
    void setUiObject();
    void initBtnJointPlus();
    void initBtnJointMinus();
    void initBtnPoseJOg();
    void initJointValueText();
    void displayCureJointValue();
    void viewJobList();
    void setCurrentJobName();
    Moveit_Servo moveitServo;
    TpcManipulationMoveit::Manipulation Manipulation;
    QTimer *updateTimer;
    QList<QTextBrowser*> textBrJointValues;
    QList<QTextBrowser*> textPositionValues;
    QList<QLineEdit*> textTargetJoint;
    QList<QPushButton*> btnPosePlus;
    QList<QPushButton*> btnPoseMinsu;
    QList<QLineEdit*> textTargetPose;
    std::vector<const char*> robot_joint_names;
    QList<QComboBox*> cmb_arc_direct;
};

#endif // MAINWINDOW_HPP
