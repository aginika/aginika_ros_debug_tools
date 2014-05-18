#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "rviz/config.h"
#include "rviz/properties/float_property.h"

#include <std_msgs/Float32.h>

#include "debug_button.h"

using namespace rviz;

namespace aginika_ros_debug_tool
{

  DebugButton::DebugButton( QWidget* parent )
    : rviz::Panel( parent )
  {
    debug_signal_float32_publisher_ = nh_.advertise<std_msgs::Float32>("debug_signal_float", 1);
    layout = new QVBoxLayout;

    QHBoxLayout* buttons_layout = new QHBoxLayout;
    {
      simple_debug_signal1_button_ = new QPushButton("1");
      simple_debug_signal2_button_ = new QPushButton("2");
      simple_debug_signal3_button_ = new QPushButton("3");
      simple_debug_signal4_button_ = new QPushButton("4");

      buttons_layout->addWidget( simple_debug_signal1_button_ );
      buttons_layout->addWidget( simple_debug_signal2_button_ );
      buttons_layout->addWidget( simple_debug_signal3_button_ );
      buttons_layout->addWidget( simple_debug_signal4_button_ );
    }

    layout->addLayout(buttons_layout);

    QHBoxLayout* value_layout = new QHBoxLayout;
    {
      value_layout->addWidget( new QLabel( "Value:" ));
      output_value_editor_ = new QLineEdit;
      value_layout->addWidget( output_value_editor_ );
    }

    layout->addLayout( value_layout );

    publish_debug_signal_button_ = new QPushButton("Debug Signal Publish");
    layout->addWidget( publish_debug_signal_button_ );

    setLayout( layout );

    connect( publish_debug_signal_button_, SIGNAL( clicked() ), this, SLOT( publishDebugSignal ()));
    connect( simple_debug_signal1_button_, SIGNAL( clicked() ), this, SLOT( publishDebugSignal1 ()));
    connect( simple_debug_signal2_button_, SIGNAL( clicked() ), this, SLOT( publishDebugSignal2 ()));
    connect( simple_debug_signal3_button_, SIGNAL( clicked() ), this, SLOT( publishDebugSignal3 ()));
    connect( simple_debug_signal4_button_, SIGNAL( clicked() ), this, SLOT( publishDebugSignal4 ()));

  }

  void DebugButton::publishFloat(float float_property_val){
    std_msgs::Float32 float32_msgs;
    float32_msgs.data = float_property_val;
    debug_signal_float32_publisher_.publish(float32_msgs);
  }

  void DebugButton::publishDebugSignal(){
    publishFloat((output_value_editor_->text()).toFloat());
  }

  void DebugButton::publishDebugSignal1(){
    publishFloat(1.0);
  }

  void DebugButton::publishDebugSignal2(){
    publishFloat(2.0);
  }

  void DebugButton::publishDebugSignal3(){
    publishFloat(3.0);
  }

  void DebugButton::publishDebugSignal4(){
    publishFloat(4.0);
  }

  void DebugButton::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void DebugButton::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aginika_ros_debug_tool::DebugButton, rviz::Panel )
