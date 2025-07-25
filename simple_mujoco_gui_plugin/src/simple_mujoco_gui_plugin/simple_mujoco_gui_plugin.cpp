#include "simple_mujoco_gui_plugin/simple_mujoco_gui_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

/* Qt libraries*/
#include <QStringList>

/* C++ STL */
#include <bitset>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

using std::placeholders::_1;

namespace simple_mujoco_gui_plugin
{
/**
 * Constructor is called first before initPlugin function, needless to say.
 */
SimpleMujocoGuiPlugin::SimpleMujocoGuiPlugin() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("SimpleMujocoGuiPlugin");  // give QObjects reasonable names

  this->initGuiStates();  // initialize member variables

  node_ = std::make_shared<rclcpp::Node>("simple_mujoco_gui_plugin");

  /* Define QoS, Publishers, and Subscribers */
  const auto qos_gui = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_gui_cmd_ = node_->create_publisher<std_msgs::msg::Bool>("mj_gui_command", qos_gui);

  const auto qos_ctrl = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  sub_joint_cmd_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "mj_joint_command", qos_ctrl,
    std::bind(&SimpleMujocoGuiPlugin::subControllerState, this, std::placeholders::_1));

  const auto qos_sim = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "mj_joint_state", qos_sim,
    std::bind(&SimpleMujocoGuiPlugin::subMujocoState, this, std::placeholders::_1));

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
    this->pubGuiCommand();
  });
  spin_timer_->start(SPIN_PERIOD_MS);
}

void SimpleMujocoGuiPlugin::initGuiStates()
{
  q_des_.resize(kDoF);
  dq_des_.resize(kDoF);
  tau_des_.resize(kDoF);

  q_mes_.resize(kDoF);
  dq_mes_.resize(kDoF);
  tau_mes_.resize(kDoF);
}

//* ----- PLUGIN INTERFACE -----------------------------------------------------------------------
void SimpleMujocoGuiPlugin::initPlugin(qt_gui_cpp::PluginContext & context)
{
  QStringList argv = context.argv();  // access standalone command line arguments
  widget_ = new QWidget();            // create QWidget
  ui_.setupUi(widget_);

  // ! these should be located here
  this->initCheckBoxes();
  this->initLineEdits();
  this->initPlotSettings();
  this->updateGuiWidgets();

  // * Define SIGNAL & SLOT connections here

  context.addWidget(widget_);  // add widget to user interface
}

/**
 * @note Add objects that are required to be reset
 */
void SimpleMujocoGuiPlugin::shutdownPlugin()
{
  if (gui_update_timer_)
  {
    gui_update_timer_->stop();
    delete gui_update_timer_;
    gui_update_timer_ = nullptr;
  }

  if (spin_timer_)
  {
    spin_timer_->stop();
    delete spin_timer_;
    spin_timer_ = nullptr;
  }
}

void SimpleMujocoGuiPlugin::saveSettings(qt_gui_cpp::Settings & plugin_settings,
                                         qt_gui_cpp::Settings & instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v);
}

void SimpleMujocoGuiPlugin::restoreSettings(const qt_gui_cpp::Settings & plugin_settings,
                                            const qt_gui_cpp::Settings & instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k);
}

//* ----- QCUSTOMPLOT ------------------------------------------------------------------------------

/**
 * @note Define plots here
 */
void SimpleMujocoGuiPlugin::initPlotSettings()
{
  QVector<QCheckBox *> autoScaleBoxes = { ui_.scale_plt_q1,   ui_.scale_plt_q2,      //
                                          ui_.scale_plt_dq1,  ui_.scale_plt_dq2,     //
                                          ui_.scale_plt_tau1, ui_.scale_plt_tau2 };  //

  for (auto & checkBox : autoScaleBoxes)
  {
    checkBox->setChecked(true);
    connect(checkBox, &QCheckBox::toggled, this, &SimpleMujocoGuiPlugin::slotUpdatePlot);
  }

  /* joint pos */
  QVector<QString> plt_q_legend = { "Desired", "Measured" };
  QVector<QColor> plt_q_color = { Qt::red, Qt::blue };
  QVector<Qt::PenStyle> plt_q_linestyle = { Qt::DotLine, Qt::SolidLine };
  QVector<double> plt_q_linewidth = { 2, 1.5 };
  this->createPlot(ui_.plt_q1, "q1 [rad]", plt_q_legend, plt_q_color, plt_q_linestyle,
                   plt_q_linewidth);
  this->createPlot(ui_.plt_q2, "q2 [rad]", plt_q_legend, plt_q_color, plt_q_linestyle,
                   plt_q_linewidth);

  /* joint vel */
  QVector<QString> plt_dq_legend = { "Desired", "Measured" };
  QVector<QColor> plt_dq_color = { Qt::red, Qt::blue };
  QVector<Qt::PenStyle> plt_dq_linestyle = { Qt::DotLine, Qt::SolidLine };
  QVector<double> plt_dq_linewidth = { 2, 1.5 };
  this->createPlot(ui_.plt_dq1, "dq1 [rad/s]", plt_dq_legend, plt_dq_color, plt_dq_linestyle,
                   plt_dq_linewidth);
  this->createPlot(ui_.plt_dq2, "dq2 [rad/s]", plt_dq_legend, plt_dq_color, plt_dq_linestyle,
                   plt_dq_linewidth);

  /* joint torque */
  QVector<QString> plt_tau_legend = { "Desired", "Measured" };
  QVector<QColor> plt_tau_color = { Qt::red, Qt::blue };
  QVector<Qt::PenStyle> plt_tau_linestyle = { Qt::DotLine, Qt::SolidLine };
  QVector<double> plt_tau_linewidth = { 2, 1.5 };
  this->createPlot(ui_.plt_tau1, "tau1 [Nm]", plt_tau_legend, plt_tau_color, plt_tau_linestyle,
                   plt_tau_linewidth);
  this->createPlot(ui_.plt_tau2, "tau2 [Nm]", plt_tau_legend, plt_tau_color, plt_tau_linestyle,
                   plt_tau_linewidth);
}

void SimpleMujocoGuiPlugin::createPlot(QCustomPlot * plt,                        //
                                       const QString & ylabel,                   //
                                       const QVector<QString> & legend_list,     //
                                       const QVector<QColor> & color,            //
                                       const QVector<Qt::PenStyle> & linestyle,  //
                                       const QVector<double> & linewidth)
{
  for (int i = 0; i < legend_list.size(); ++i)
  {
    plt->addGraph();
    plt->graph(i)->setPen(QPen(color[i], linewidth[i], linestyle[i]));
    plt->graph(i)->setName(legend_list[i]);
  }

  /* add label */
  plt->xAxis->setLabel("Time [sec]");
  plt->yAxis->setLabel(ylabel);

  plt->yAxis->setNumberFormat("f");
  plt->yAxis->setNumberPrecision(3);

  /* add legend */
  plt->legend->setVisible(true);
  plt->legend->setFont(QFont("sans", 9));
  plt->legend->setRowSpacing(-3);
  plt->legend->setBrush(QBrush(QColor(255, 255, 255, 230)));

  plt->axisRect()->insetLayout()->setInsetPlacement(0, QCPLayoutInset::ipBorderAligned);
  plt->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);
  plt->axisRect()->insetLayout()->setMargins(QMargins(2, 2, 2, 2));

  /* make a compact layout */
  QCPMarginGroup * margin_group = new QCPMarginGroup(plt);
  plt->axisRect()->setMarginGroup(QCP::msLeft | QCP::msRight, margin_group);
  plt->axisRect()->setMargins(QMargins(50, 5, 5, 30));  // left, top, right, bottom

  connect(plt, &QCustomPlot::legendClick, this, &SimpleMujocoGuiPlugin::deactivateLegend);
}

void SimpleMujocoGuiPlugin::drawPlot(QCustomPlot * plt, QCheckBox * autoscale,
                                     const QVector<double> & data,
                                     const QPair<double, double> & xlim,
                                     const QPair<double, double> & ylim)
{
  for (int i = 0; i < data.size(); ++i)
  {
    plt->graph(i)->addData(current_time_, data[i]);
    plt->graph(i)->data()->removeBefore(current_time_ - PLOT_TIME_RANGE_SEC_);
  }
  plt->xAxis->setRange(xlim.first, xlim.second);

  if (autoscale->isChecked())
  {
    plt->yAxis->rescale(true);
  }
  else
  {
    plt->yAxis->setRange(ylim.first, ylim.second);
  }
  plt->replot();
}

void SimpleMujocoGuiPlugin::deactivateLegend(QCPLegend * legend, QCPAbstractLegendItem * item)
{
  if (!item)
  {
    return;
  }

  QCPPlottableLegendItem * legend_item = qobject_cast<QCPPlottableLegendItem *>(item);
  if (!legend_item)
  {
    return;
  }

  QCPGraph * graph = qobject_cast<QCPGraph *>(legend_item->plottable());
  if (!graph)
  {
    return;
  }

  graph->setVisible(!graph->visible());
  graph->parentPlot()->replot();
}

//* ----- ROS INTERFACE ----------------------------------------------------------------------------
void SimpleMujocoGuiPlugin::pubGuiCommand()
{
  auto msg = std_msgs::msg::Bool();
  msg.data = ui_.ckBox_sim_run->isChecked();
  pub_gui_cmd_->publish(msg);
}

void SimpleMujocoGuiPlugin::subMujocoState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (int i = 0; i < kDoF; ++i)
  {
    q_mes_[i] = msg->position[i];
    dq_mes_[i] = msg->velocity[i];
    tau_mes_[i] = msg->effort[i];
  }
}

void SimpleMujocoGuiPlugin::subControllerState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (int i = 0; i < kDoF; ++i)
  {
    q_des_[i] = msg->position[i];
    dq_des_[i] = msg->velocity[i];
    tau_des_[i] = msg->effort[i];
  }
}

//* ----- QT METHODS -------------------------------------------------------------------------------
void SimpleMujocoGuiPlugin::initCheckBoxes()
{
  /* append checkboxes to include all widgets */
  QVector<QCheckBox *> checkBoxes = { ui_.ckBox_sim_run };

  QString styleSheet =
    "QCheckBox::indicator { width:50px; height:50px; }"
    "QCheckBox::indicator:checked { image: url(:/icons/on-button.png); }"
    "QCheckBox::indicator:unchecked { image: url(:/icons/off-button.png); }";

  for (auto & checkBox : checkBoxes)
  {
    checkBox->setChecked(false);
    checkBox->setStyleSheet(styleSheet);
  }
}

void SimpleMujocoGuiPlugin::initLineEdits()
{
  le_plt_q_min_ = { ui_.le_plt_ymin_q1, ui_.le_plt_ymin_q2 };
  plt_q_min_.resize(kDoF);
  le_plt_q_max_ = { ui_.le_plt_ymax_q1, ui_.le_plt_ymax_q2 };
  plt_q_max_.resize(kDoF);
  le_plt_dq_min_ = { ui_.le_plt_ymin_dq1, ui_.le_plt_ymin_dq2 };
  plt_dq_min_.resize(kDoF);
  le_plt_dq_max_ = { ui_.le_plt_ymax_dq1, ui_.le_plt_ymax_dq2 };
  plt_dq_max_.resize(kDoF);
  le_plt_tau_min_ = { ui_.le_plt_ymin_tau1, ui_.le_plt_ymin_tau2 };
  plt_tau_min_.resize(kDoF);
  le_plt_tau_max_ = { ui_.le_plt_ymax_tau1, ui_.le_plt_ymax_tau2 };
  plt_tau_max_.resize(kDoF);

  for (int i = 0; i < kDoF; ++i)
  {
    plt_q_max_[i] = le_plt_q_max_[i]->text().toDouble();
    connect(le_plt_q_max_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);

    plt_q_min_[i] = le_plt_q_min_[i]->text().toDouble();
    connect(le_plt_q_min_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);

    plt_dq_max_[i] = le_plt_dq_max_[i]->text().toDouble();
    connect(le_plt_dq_max_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);

    plt_dq_min_[i] = le_plt_dq_min_[i]->text().toDouble();
    connect(le_plt_dq_min_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);

    plt_tau_max_[i] = le_plt_tau_max_[i]->text().toDouble();
    connect(le_plt_tau_max_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);

    plt_tau_min_[i] = le_plt_tau_min_[i]->text().toDouble();
    connect(le_plt_tau_min_[i], &QLineEdit::editingFinished, this,
            &SimpleMujocoGuiPlugin::slotUpdateYlim);
  }
}

void SimpleMujocoGuiPlugin::updateGuiWidgets()
{
  if (!gui_update_timer_)
  {
    gui_update_timer_ = new QTimer(this);
    connect(gui_update_timer_, SIGNAL(timeout()), this, SLOT(slotUpdatePlot()));
    gui_update_timer_->start(GUI_UPDATE_PERIOD_MS);
  }
  if (!gui_update_timer_->isActive())  // start update timer always
    gui_update_timer_->start(GUI_UPDATE_PERIOD_MS);
}

//* ----- SLOT METHODS -----------------------------------------------------------------------------

/**
 * @note Define plot's DATA and RANGE what you want to check here
 */
void SimpleMujocoGuiPlugin::slotUpdatePlot()
{
  current_time_ += dt_;
  QPair<double, double> xlim = qMakePair(current_time_ - PLOT_TIME_RANGE_SEC_, current_time_);

  QVector<QCustomPlot *> q_plots = { ui_.plt_q1, ui_.plt_q2 };
  QVector<QCheckBox *> q_scales = { ui_.scale_plt_q1, ui_.scale_plt_q2 };
  for (int i = 0; i < kDoF; ++i)
  {
    QVector<double> q_data = { q_des_[i], q_mes_[i] };
    QPair<double, double> q_ylim = { plt_q_min_[i], plt_q_max_[i] };
    this->drawPlot(q_plots[i], q_scales[i], q_data, xlim, q_ylim);
  }

  QVector<QCustomPlot *> dq_plots = { ui_.plt_dq1, ui_.plt_dq2 };
  QVector<QCheckBox *> dq_scales = { ui_.scale_plt_dq1, ui_.scale_plt_dq2 };
  for (int i = 0; i < kDoF; ++i)
  {
    QVector<double> dq_data = { dq_des_[i], dq_mes_[i] };
    QPair<double, double> dq_ylim = { plt_dq_min_[i], plt_dq_max_[i] };
    this->drawPlot(dq_plots[i], dq_scales[i], dq_data, xlim, dq_ylim);
  }

  QVector<QCustomPlot *> tau_plots = { ui_.plt_tau1, ui_.plt_tau2 };
  QVector<QCheckBox *> tau_scales = { ui_.scale_plt_tau1, ui_.scale_plt_tau2 };
  for (int i = 0; i < kDoF; ++i)
  {
    QVector<double> tau_data = { tau_des_[i], tau_mes_[i] };
    QPair<double, double> tau_ylim = { plt_tau_min_[i], plt_tau_max_[i] };
    this->drawPlot(tau_plots[i], tau_scales[i], tau_data, xlim, tau_ylim);
  }
}

void SimpleMujocoGuiPlugin::slotUpdateYlim()
{
  for (int i = 0; i < kDoF; ++i)
  {
    plt_tau_max_[i] = le_plt_tau_max_[i]->text().toDouble();
    plt_tau_min_[i] = le_plt_tau_min_[i]->text().toDouble();

    plt_q_max_[i] = le_plt_q_max_[i]->text().toDouble();
    plt_q_min_[i] = le_plt_q_min_[i]->text().toDouble();

    plt_dq_max_[i] = le_plt_dq_max_[i]->text().toDouble();
    plt_dq_min_[i] = le_plt_dq_min_[i]->text().toDouble();
  }
}

}  // namespace simple_mujoco_gui_plugin

PLUGINLIB_EXPORT_CLASS(simple_mujoco_gui_plugin::SimpleMujocoGuiPlugin, rqt_gui_cpp::Plugin)