#ifndef OPENCONSTRAINTFILEDIALOG_H
#define OPENCONSTRAINTFILEDIALOG_H

#include <QDialog>

namespace Ui {
class OpenConstraintFileDialog;
}

class OpenConstraintFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OpenConstraintFileDialog(QWidget *parent = nullptr);
    ~OpenConstraintFileDialog();

private:
    Ui::OpenConstraintFileDialog *ui;
};

#endif // OPENCONSTRAINTFILEDIALOG_H
