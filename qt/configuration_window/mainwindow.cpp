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
    this->_connect_signal_to_slots();
}

/**
 * @brief MainWindow::~MainWindow destructor of the class
 */
MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
}

void MainWindow::_connect_signal_to_slots()
{

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

void MainWindow::file_open_value_returned_from_dialog(QString file_path)
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
    setEnabled(0); // disabled to prevent additional dialogs being opened
    open_file_dialogue_ = new OpenConstraintFileDialog(this);
    //following connection not opened in constructor as does not need to be open for lifetime of program
    QObject::connect(open_file_dialogue_,&OpenConstraintFileDialog::return_open_file_to_window,this,&MainWindow::file_open_value_returned_from_dialog);
    open_file_dialogue_->show();
    if (!open_file_dialogue_->exec())
    {
        setEnabled(1);
    }
}


