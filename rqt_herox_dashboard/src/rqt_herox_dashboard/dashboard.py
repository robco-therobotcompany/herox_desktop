import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from herox_coordinator.msg import SubsystemStatus, SubsystemStatusSingle
from herox_coordinator.srv import SubsystemControl, SubsystemControlResponse

hwState = False
ctrlState = False
navState = False
teleopState = False

class HeroxDashboard(Plugin):

    def __init__(self, context):
        super(HeroxDashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HeroxDashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_herox_dashboard'), 'resource', 'HeroxDashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('HeroxDashboardUi')

        # Set up subscribers
        self._subsystemSub = rospy.Subscriber('subsystem_status', SubsystemStatus, self.subsystemStatusCallback)

        # Set up button callbacks
        self._widget.btnHw.clicked.connect(self.btnHw_callback)
        self._widget.btnCtrl.clicked.connect(self.btnCtrl_callback)
        self._widget.btnNav.clicked.connect(self.btnNav_callback)
        self._widget.btnTeleop.clicked.connect(self.btnTeleop_callback)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
    
    def updateButtonProperty(self, button, prop, value):
        button.setProperty(prop, value)
        button.style().unpolish(button)
        button.style().polish(button)

    def subsystemStatusCallback(self, msg):
        global hwState
        global ctrlState
        global navState
        global teleopState

        for sub in msg.subsystems:
            if sub.name == "HW":
                hwState = sub.status
                self.updateButtonProperty(self._widget.btnHw, "active", hwState)
            elif sub.name == "CTRL":
                ctrlState = sub.status
                self.updateButtonProperty(self._widget.btnCtrl, "active", ctrlState)
            elif sub.name == "NAV":
                navState = sub.status
                self.updateButtonProperty(self._widget.btnNav, "active", navState)
            elif sub.name == "TELEOP":
                teleopState = sub.status
                self.updateButtonProperty(self._widget.btnTeleop, "active", teleopState)

    def subsystemControlRequest(self, subsystem, state):
        rospy.wait_for_service('subsystem_control')
        try:
            subsystem_control = rospy.ServiceProxy('subsystem_control', SubsystemControl)
            subsystem_control(subsystem, state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def btnHw_callback(self):
        global hwState

        rospy.loginfo('requesting HW %s'%(not hwState))
        self.subsystemControlRequest('HW', not hwState)

    def btnCtrl_callback(self):
        global ctrlState
        rospy.loginfo('requesting CTRL %s'%(not ctrlState))
        self.subsystemControlRequest('CTRL', not ctrlState)

    def btnNav_callback(self):
        global navState
        rospy.loginfo('requesting NAV %s'%(not navState))
        self.subsystemControlRequest('NAV', not navState)

    def btnTeleop_callback(self):
        global teleopState
        rospy.loginfo('requesting TELEOP %s'%(not teleopState))
        self.subsystemControlRequest('TELEOP', not teleopState)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

