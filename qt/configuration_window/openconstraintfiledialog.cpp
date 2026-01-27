#include "openconstraintfiledialog.h"
#include "ui_openconstraintfiledialog.h"

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
