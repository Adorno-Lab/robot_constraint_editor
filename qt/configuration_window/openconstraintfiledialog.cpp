#include "openconstraintfiledialog.h"
#include "ui_openconstraintfiledialog.h"
#include <QFileDialog>
#include <QDebug>
#include <QFile>


OpenConstraintFileDialog::OpenConstraintFileDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::OpenConstraintFileDialog)
{
    ui->setupUi(this);
}

OpenConstraintFileDialog::~OpenConstraintFileDialog()
{
    delete ui;
}

void OpenConstraintFileDialog::on_open_file_explore_pushButton_clicked()
{
    ui->open_file_explore_pushButton->setEnabled(0);
    auto *file_select_dialog = new QFileDialog();
    file_select_dialog->setFileMode(QFileDialog::ExistingFile); // the user can only select a single existing file instead of directories etc.
    const QStringList filters({"*.yaml"}); // when more files are supported this will need to be updated
    file_select_dialog->setNameFilters(filters);
    file_select_dialog->show();
    QStringList file_name;
    if (file_select_dialog->exec()){
        ui->error_label->setText("");
        file_name = file_select_dialog->selectedFiles();
        ui->file_path_lineEdit->setText(file_name.at(0).toLatin1()); // Since we set the file mode to single existing file we only need to index the firs item in the returned list
        ui->open_file_explore_pushButton->setEnabled(1);
    }
    else{
        ui->open_file_explore_pushButton->setEnabled(1);
    }

}


void OpenConstraintFileDialog::on_cancle_pushButton_clicked()
{
    this->reject();
}


void OpenConstraintFileDialog::on_open_file_pushButton_clicked()
{
    if(QFile::exists(ui->file_path_lineEdit->text()) == true){
        qDebug("Yay the file exists");
    }
    else{
        ui->error_label->setText("ERROR: YAML file not found. Check file path.");
    }
}

