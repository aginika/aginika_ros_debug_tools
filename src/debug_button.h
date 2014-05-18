#ifndef DEBUG_BUTTON_H
#define DEBUG_BUTTON_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include <QtGui>
#include "rviz/properties/float_property.h"

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;
class PropertyTreeWidget;


namespace aginika_ros_debug_tool
{
  class DebugButton: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      DebugButton( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      protected Q_SLOTS:

      void publishDebugSignal();
      void publishDebugSignal1();
      void publishDebugSignal2();
      void publishDebugSignal3();
      void publishDebugSignal4();
      void publishFloat(float float_val);
    protected:
      QPushButton* publish_debug_signal_button_;
      QPushButton* simple_debug_signal1_button_;
      QPushButton* simple_debug_signal2_button_;
      QPushButton* simple_debug_signal3_button_;
      QPushButton* simple_debug_signal4_button_;

      QVBoxLayout* layout;

      // The ROS publisher for the command velocity.
      ros::Publisher debug_signal_float32_publisher_;

      // The ROS node handle.
      ros::NodeHandle nh_;

      QLineEdit* output_value_editor_;

    };

}

#endif // TELEOP_PANEL_H
