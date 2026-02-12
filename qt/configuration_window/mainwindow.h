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
#   Robot Constraint Editor
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
#include "open_constraint_file_dialog.h"
#include <QPointer>


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
    void file_open_value_returned_from_dialog(QString file_path);
private slots:
    void open_file_action_triggered();

private:
    Ui::MainWindow *ui;
    void _connect_signal_to_slots();
    QString constraint_file_filepath_;
    std::shared_ptr<DQ_robotics_extensions::VFIConfigurationFileYaml> vfi_yaml_;
    DQ_robotics_extensions::RobotConstraintEditor robot_constraint_editor_;
};

