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

private slots:
    void on_open_file_explore_pushButton_clicked();

    void on_cancle_pushButton_clicked();

    void on_open_file_pushButton_clicked();

signals:
    void return_open_file_to_window(QString Filepath);
    void enable_main_window();
private:
    Ui::OpenConstraintFileDialog *ui;
};

#endif // OPENCONSTRAINTFILEDIALOG_H
