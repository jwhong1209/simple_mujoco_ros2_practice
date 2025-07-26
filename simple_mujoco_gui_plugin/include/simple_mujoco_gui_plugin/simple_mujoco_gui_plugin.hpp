/** @file simple_mujoco_gui_plugin.hpp
 * ! If you declare function in a header, while not defining it in source file, linking error occurs
 * * It is better to use `virtual` keyword for protected slot function
 */

#ifndef SIMPLE_MUJOCO_GUI_PLUGIN_HPP_
#define SIMPLE_MUJOCO_GUI_PLUGIN_HPP_

/* C++ STL */
#include <chrono>
#include <cstdint>
#include <memory>

/* ROS2 packages */
#include <rqt_gui_cpp/rqt_gui_cpp/plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

/* UI widget header */
#include <ui_simple_mujoco_gui_plugin.h>

/* Qt libraries */
#include <qcustomplot.h>
#include <QPair>
#include <QTimer>
#include <QVector>
#include <QWidget>

namespace simple_mujoco_gui_plugin
{
class SimpleMujocoGuiPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

private:
  Ui::SimpleMujocoGuiPlugin ui_;  // ! don't forget to change as class name
  QWidget * widget_;

  QTimer * spin_timer_ = nullptr;
  const int SPIN_PERIOD_MS = 10;  // subscribe & publish at 10ms

  QTimer * gui_update_timer_ = nullptr;
  const int GUI_UPDATE_PERIOD_MS = 15;  // approximate human's reaction time (60Hz = 16.7ms)

  double current_time_ = 0.0;
  double dt_ = GUI_UPDATE_PERIOD_MS / 1000.0;

  const int kDoF = 2;

  // * Match member variables matched with ROS messages
  std::vector<double> q_des_, q_mes_;
  std::vector<double> dq_des_, dq_mes_;
  std::vector<double> tau_des_, tau_mes_;
  void initGuiStates();

  //* ----- QT WIDGETS & METHODS -------------------------------------------------------------------
  void initCheckBoxes();
  void initLineEdits();
  void updateGuiWidgets();

  //* ----- QCUSTOMPLOT ----------------------------------------------------------------------------
  const double PLOT_TIME_RANGE_SEC_ = 10.0;

  QVector<double> plt_q_min_, plt_q_max_;
  QVector<QLineEdit *> le_plt_q_min_, le_plt_q_max_;
  QVector<double> plt_dq_min_, plt_dq_max_;
  QVector<QLineEdit *> le_plt_dq_min_, le_plt_dq_max_;
  QVector<double> plt_tau_min_, plt_tau_max_;
  QVector<QLineEdit *> le_plt_tau_min_, le_plt_tau_max_;

  void initPlotSettings();
  void createPlot(QCustomPlot * plt, const QString & ylabel, const QVector<QString> & legend_list,
                  const QVector<QColor> & color, const QVector<Qt::PenStyle> & linestyle,
                  const QVector<double> & linewidth);
  void drawPlot(QCustomPlot * plt, QCheckBox * autoscale, const QVector<double> & data,
                const QPair<double, double> & xlim, const QPair<double, double> & ylim);
  void deactivateLegend(QCPLegend * legend, QCPAbstractLegendItem * item);

  //* ----- ROS INTERFACE --------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;

  /* Publisher */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_gui_cmd_;
  void pubGuiCommand();

  /* Subscriber */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_cmd_;
  void subJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  void subJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

public:
  SimpleMujocoGuiPlugin();

  //* ----- PLUGIN INTERFACE -----------------------------------------------------------------------
  // ! Don't change these functions
  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings & plugin_settings,
                            qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings & plugin_settings,
                               const qt_gui_cpp::Settings & instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

protected slots:
  virtual void slotUpdatePlot();
  virtual void slotUpdateYlim();
};
};  // namespace simple_mujoco_gui_plugin

#endif  // SIMPLE_MUJOCO_GUI_PLUGIN_HPP_