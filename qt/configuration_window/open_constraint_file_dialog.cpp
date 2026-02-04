
#include "open_constraint_file_dialog.h"
#include "ui_open_constraint_file_dialog.h"

using namespace DQ_robotics_extensions;

OpenConstraintFileDialog::OpenConstraintFileDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::OpenConstraintFileDialog)
{
    ui->setupUi(this);
    _connect_signal_to_slots();
}

OpenConstraintFileDialog::~OpenConstraintFileDialog()
{
    delete ui;
}

void OpenConstraintFileDialog::_connect_signal_to_slots(){
    QObject::connect(ui->cancel_pushButton,&QPushButton::clicked,this,&OpenConstraintFileDialog::on_cancel_pushButton_clicked);
    QObject::connect(ui->open_file_explore_pushButton,&QPushButton::clicked,this,&OpenConstraintFileDialog::on_open_file_explore_pushButton_clicked);
    QObject::connect(ui->open_file_pushButton, &QPushButton::clicked,this,&::OpenConstraintFileDialog::on_open_file_pushButton_clicked);
}

void OpenConstraintFileDialog::on_open_file_explore_pushButton_clicked()
{
    ui->open_file_explore_pushButton->setEnabled(0);
    QFileDialog file_select_dialog(this);
    file_select_dialog.setFileMode(QFileDialog::ExistingFile); // the user can only select a single existing file instead of directories etc.
    const QStringList filters({"*.yaml"}); // when more files are supported this will need to be updated
    file_select_dialog.setNameFilters(filters);
    file_select_dialog.show();
    QStringList file_name;
    if (file_select_dialog.exec()){
        ui->error_label->setText("");
        file_name = file_select_dialog.selectedFiles();
        ui->file_path_lineEdit->setText(file_name.at(0).toLatin1()); // Since we set the file mode to single existing file we only need to index the firs item in the returned list
        ui->open_file_explore_pushButton->setEnabled(1);
    }
    else{
        ui->open_file_explore_pushButton->setEnabled(1);
    }
}


void OpenConstraintFileDialog::on_cancel_pushButton_clicked()
{
    this->reject();
}


void OpenConstraintFileDialog::on_open_file_pushButton_clicked()
{
    if(QFile::exists(ui->file_path_lineEdit->text()) == true){
        //qDebug("Yay the file exists");
        auto ri= std::make_shared<VFIConfigurationFileYaml>();
        try{
            ri->load_data(ui->file_path_lineEdit->text().toStdString());
            OpenConstraintFileDialog::return_open_file_to_window(ui->file_path_lineEdit->text());
            this->reject(); // Not sure if using reject here is bad but eh it works
        }
        catch(const std::runtime_error& error){ // Need to talk to juan about error catching here. His code simply prints to terminal without passing an error that can be caught
            ui->error_label->setText(QString::fromStdString(error.what()));
        }
    }
    else{
        ui->error_label->setText("ERROR: YAML file not found. Check file path.");
    }
}

