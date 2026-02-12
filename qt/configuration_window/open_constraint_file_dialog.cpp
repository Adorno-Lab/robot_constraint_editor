
#include "open_constraint_file_dialog.h"
#include "ui_open_constraint_file_dialog.h"

using namespace DQ_robotics_extensions;

/**
 * @brief OpenConstraintFileDialog::OpenConstraintFileDialog constructor of the class
 * @param parent should always be the main window
 */
OpenConstraintFileDialog::OpenConstraintFileDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::OpenConstraintFileDialog)
{
    ui->setupUi(this);
    _connect_signal_to_slots();
}

/**
 * @brief OpenConstraintFileDialog::~OpenConstraintFileDialog destructor of class
 */
OpenConstraintFileDialog::~OpenConstraintFileDialog()
{
    delete ui;
}

/**
 * @brief OpenConstraintFileDialog::_connect_signal_to_slots connects the signals to their
 *                                                            corresponding slots. This method must be called in the ctor                                                              of the class.https://doc.qt.io/qt-6/signalsandslots.html
 */
void OpenConstraintFileDialog::_connect_signal_to_slots(){
    QObject::connect(ui->cancel_pushButton,&QPushButton::clicked,this,&OpenConstraintFileDialog::cancel_pushButton_clicked);
    QObject::connect(ui->open_file_explore_pushButton,&QPushButton::clicked,this,&OpenConstraintFileDialog::open_file_explore_pushButton_clicked);
    QObject::connect(ui->open_file_pushButton, &QPushButton::clicked,this,&::OpenConstraintFileDialog::open_file_pushButton_clicked);
}


/**
 * @brief OpenConstraintFileDialog::open_file_explore_pushButton_clicked QT slot connecting to file_explore_pushButton being clicked.
 *                                                                        Disables button. Opens QFileDialog instance. Sets file mode so user can only
 *                                                                        select a single file. Restricts file display to YAML files. Stores result in file
 *                                                                        path line edit so user can edit filepath if needed. Closes file explore. Renables
 *                                                                        Button.
 */
void OpenConstraintFileDialog::open_file_explore_pushButton_clicked()
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

/**
 * @brief OpenConstraintFileDialog::cancel_pushButton_clicked QT slot which connects cancel button to closure of OpenConstraintFileDialog instance.
 */
void OpenConstraintFileDialog::cancel_pushButton_clicked()
{
    this->reject();
}


/**
 * @brief OpenConstraintFileDialog::open_file_pushButton_clicked QT slot which connects open file button to return of the file path to the main window.
 *                                                                First checks if file exists. Then tries to open the file as VFIConfigurationFileYaml. If
 *                                                                it fails we can assume the file is not in the correct format and can deliver the required
 *                                                                error messages to the user via label. If it works we can assume the file is correct and
 *                                                                return it to the main window for further use.
 */
void OpenConstraintFileDialog::open_file_pushButton_clicked()
{
    if(QFile::exists(ui->file_path_lineEdit->text()) == true){
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

