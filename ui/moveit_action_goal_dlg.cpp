#include "moveit_action_goal_dlg.h"
#include "ui_moveit_action_goal_dlg.h"

MoveitActionGoalDlg::MoveitActionGoalDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MoveitActionGoalDlg)
{
    ui->setupUi(this);
}

MoveitActionGoalDlg::~MoveitActionGoalDlg()
{
    delete ui;
}
