// Include guards
#ifndef MY_QT_GUI_MAINWINDOW_HPP
#define MY_QT_GUI_MAINWINDOW_HPP

#include <QMainWindow>
#include "./ui_mainwindow.h"


class MainWindow : public QMainWindow {
    Q_OBJECT   //QT의 메타 오브젝트 시스템을 위한 매크로

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    // UI components
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_HPP
