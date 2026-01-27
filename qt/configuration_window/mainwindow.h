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
#include <QFile>
#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>

QT_BEGIN_NAMESPACE
using namespace DQ_robotics_extensions;
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

public slots:
    void on_FileOpenValueReturnFromDialog(QString file_path); // i'm so sorry this is named like this but qt moc starts to get really annoyed if you don't follow their naming conventions
    // See: https://stackoverflow.com/questions/24355023/qmetaobjectconnectslotsbyname-no-matching-signal
private slots:
    void on_open_file_action_triggered();

private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *event);
    int timerId_;
    int counter_;
    void _connect_signal_to_slots();
};

