#pragma once

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_srvs/Trigger.h>
#include <pluginlib/class_list_macros.h>

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif


class QPushButton;

namespace rviz_plugins
{

class StateTriggerPanel: public rviz::Panel
{
Q_OBJECT
public:
    StateTriggerPanel(QWidget* parent = 0);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:
    void pushStartNavigation();
    void pushResumeNavigation();
    
protected:
    ros::ServiceClient start_client_, resume_client_;
    QPushButton *start_nav_button_;
    QPushButton *resume_nav_button_;

};

} // namespace rviz_plugins
