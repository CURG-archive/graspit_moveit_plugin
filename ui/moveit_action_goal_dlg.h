#ifndef MOVEIT_ACTION_GOAL_DLG_H
#define MOVEIT_ACTION_GOAL_DLG_H

#include <QDialog>

namespace Ui {
class MoveitActionGoalDlg;
}

class MoveitActionGoalDlg : public QDialog
{
    Q_OBJECT
    
public:
    explicit MoveitActionGoalDlg(QWidget *parent = 0);
    ~MoveitActionGoalDlg();
    
private:
    Ui::MoveitActionGoalDlg *ui;
};

#endif // MOVEIT_ACTION_GOAL_DLG_H
