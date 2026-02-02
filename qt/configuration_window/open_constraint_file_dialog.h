#pragma once
#include <QDialog>

#include <QFileDialog>
#include <QDebug>
#include <QFile>
#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

namespace Ui {
class OpenConstraintFileDialog;
}

class OpenConstraintFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OpenConstraintFileDialog(QWidget *parent = nullptr);
    ~OpenConstraintFileDialog();

private slots:
    void on_open_file_explore_pushButton_clicked();

    void on_cancel_pushButton_clicked();

    void on_open_file_pushButton_clicked();

signals:
    void return_open_file_to_window(QString file_path);
private:
    Ui::OpenConstraintFileDialog *ui;
    void _connect_signal_to_slots();

};

