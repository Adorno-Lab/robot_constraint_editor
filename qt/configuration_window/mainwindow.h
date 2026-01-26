/*
#    Copyright (c) 2025 Adorno-Lab
#
#    robot_constraint_editor is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    robot_constraint_editor is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with robot_constraint_editor.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#   Minimal Example
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
#   Contributors:
#   Author:
#
# ################################################################
*/

#pragma once
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void _helloWorld_pushButton_pressed();

private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *event);
    int timerId_;
    int counter_;
    void _connect_signal_to_slots();
};

