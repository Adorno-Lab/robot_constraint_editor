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

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "openconstraintfiledialog.h"

/**
 * @brief MainWindow::MainWindow ctor of the class
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow{parent}
    , ui{new Ui::MainWindow},
    counter_{0}
{
    ui->setupUi(this);
    timerId_ = startTimer(1000); // Start timer for 1000ms


}

/**
 * @brief MainWindow::_connect_signal_to_slots connects the signals to their
 *                  corresponding slots. This method must be called in the ctor
 *                  of the class.
 *                  https://doc.qt.io/qt-6/signalsandslots.html
 */


/**
 * @brief MainWindow::~MainWindow destructor of the class
 */
MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
}

/**
 * @brief MainWindow::timerEvent This method is called periodically at the interval defined in the class constructor.
 *              In this example, this method updates the progressBar object.
 * @param event
 */
void MainWindow::timerEvent(QTimerEvent *event)
{
    counter_++;
    if (counter_>100)
        counter_=0;
}


void MainWindow::on_open_file_action_triggered()
{
    auto *open_file_dialogue = new OpenConstraintFileDialog(this);
    open_file_dialogue->show();
}
