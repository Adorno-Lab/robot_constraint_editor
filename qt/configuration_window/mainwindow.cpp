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

void MainWindow::on_FileOpenValueReturnFromDialog(QString file_path)
{
    if (file_path.length()>60){
        MainWindow::ui->constraint_file_label->setText("File: ..."+file_path.last(60)); // prevents file path wrap arround at default size
    }
    else{
        MainWindow::ui->constraint_file_label->setText("File: "+file_path);
    }

}

void MainWindow::on_open_file_action_triggered()
{

    this->setEnabled(0);
    auto *open_file_dialogue = new OpenConstraintFileDialog(this);
    QObject::connect(open_file_dialogue,&OpenConstraintFileDialog::return_open_file_to_window,this,&MainWindow::on_FileOpenValueReturnFromDialog);
    open_file_dialogue->show();
    if (!open_file_dialogue->exec())
    {
        this->setEnabled(1);
    }
}


