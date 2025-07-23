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
//#include "tm5_gui/moveit_servo.hpp"

#include <QApplication>
#include <signal.h>

void quit(int sig);

void quit(int sig)
{
  (void)sig;
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  //ros 초기화

    signal(SIGINT, quit);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    int result = a.exec();
    rclcpp::shutdown();  //ros 종료
    return result;
}
