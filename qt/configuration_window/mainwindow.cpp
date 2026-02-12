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
 * @brief MainWindow::MainWindow  ctor of the class
 * @param parent
 */

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow{parent}
    , ui{new Ui::MainWindow}
    , robot_constraint_editor_(vfi_yaml_)
{
    ui->setupUi(this);
    _connect_signal_to_slots();
}

/**
 * @brief MainWindow::~MainWindow destructor of the class
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * @brief MainWindow::_connect_signal_to_slots connects the signals to their
  *                                             corresponding slots. This method must be called in the ctor
  *                                             of the class.https://doc.qt.io/qt-6/signalsandslots.html
 */
void MainWindow::_connect_signal_to_slots()
{
    QObject::connect(ui->open_file_action, &QAction::triggered, this, &MainWindow::open_file_action_triggered);

}


/**
 * @brief MainWindow::file_open_value_returned_from_dialog Is a QT slot which accepts the file path to a valid VFI config file.
 *                                                          It stores this value in the constraint_file_filepath_ member variable.
 *                                                          This is value is also saved to the text field of text label in the the main window.
 *                                                          The value is shortened if over a certain length for visual simplicity.
 *                                                          It is connected not in open_file_action_triggered
 * @param file_path
 */
void MainWindow::file_open_value_returned_from_dialog(QString file_path)
{   this->constraint_file_filepath_ = file_path;
    if (file_path.length()>60){
        MainWindow::ui->constraint_file_label->setText("File: ..."+file_path.last(60)); // prevents file path wrap arround at default size
    }
    else{
        MainWindow::ui->constraint_file_label->setText("File: "+file_path);
    }
    ui->constraint_select_tableWidget->clear();
    QStringList headers = {"Tag","Constraint Type", "VFI Gain"};
    ui->constraint_select_tableWidget->setHorizontalHeaderLabels(headers);
}

/**
 * @brief MainWindow::open_file_action_triggered is a QT slot which responds to the action of the user opening a new file
  *                                               (either by pressing the hotbar button or by using the shortcut Ctrl+o .
  *                                               It disables the main window, connects the signals and slots needed to return a value
  *                                               and simply waits for the value to be returned.
 */
void MainWindow::open_file_action_triggered()
{
    setEnabled(0); // disabled to prevent additional dialogs being opened
     OpenConstraintFileDialog open_file_dialog;
    //following connection not opened in constructor as does not need to be open for lifetime of program
    QObject::connect(&open_file_dialog,&OpenConstraintFileDialog::return_open_file_to_window,this,&MainWindow::file_open_value_returned_from_dialog);
    open_file_dialog.show();
    if (!open_file_dialog.exec())
    {
        setEnabled(1);
    }
}


