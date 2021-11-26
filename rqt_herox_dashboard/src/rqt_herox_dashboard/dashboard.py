import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import QWidget, QPushButton
from herox_coordinator.msg import SubsystemStatus, SubsystemStatusSingle, Missions
from herox_coordinator.srv import SubsystemControl, SubsystemControlResponse, LoadMission, NewMission, SaveMission
from std_srvs.srv import Trigger
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Pose

class Subsystem(QtCore.QObject):
    status_updated = QtCore.pyqtSignal(object)

    def __init__(self, name):
        QtCore.QObject.__init__(self)
        self.name = name
        self.status = False
        self.button = None

    def update_status(self, status):
        self.status = status
        self.status_updated.emit(self)

# Values for colored terminal output
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class HeroxDashboard(Plugin):
    subsys_received = False
    subsystems = []

    missionsUpdated = QtCore.pyqtSignal(object)
    missions = []

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

        # Set up publishers
        self._cancelGoalPub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Set up services
        print("Waiting for services...")
        rospy.wait_for_service('/subsystem_control')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /subsystem_control')

        # Set up subscribers (except for missions, that's set up at the end to populate combo box)
        self._subsystemSub = rospy.Subscriber('subsystem_status', SubsystemStatus, self.subsystemStatusCallback)

        # Set up subsystem buttons
        print("Waiting for subsystem message...")
        while not self.subsys_received:
            rospy.sleep(1)
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' Subsystems received')

        columns = 3
        row = 0
        col = 0

        for sub in self.subsystems:
            sub.button = QPushButton(sub.name)
            self._widget.gbxSubsystems.layout().addWidget(sub.button, row, col)
            sub.button.clicked.connect(self.subsysButton_callback)
            sub.status_updated.connect(self.subsystemUpdated_callback)
            col = col + 1
            if col >= columns:
                col = 0
                row = row + 1

        self._haltService = rospy.ServiceProxy('/driver/halt', Trigger)
        self._subsysService = rospy.ServiceProxy('/subsystem_control', SubsystemControl)

        # Set up button callbacks
        self._widget.btnCancelGoal.clicked.connect(self.btnCancelGoal_callback)
        self._widget.btnEstop.clicked.connect(self.btnEstop_callback)

        self._widget.setWindowTitle('HEROX Dashboard')

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
        global motorsState
        global lasersState
        global navState
        global teleopState
        global coordState
        global controlState

        for sub in msg.subsystems:
            # Initialize subsystems on first message
            if not self.subsys_received:
                newSubsys = Subsystem(sub.name)
                self.subsystems.append(newSubsys)
                continue

            for s in self.subsystems:
                if s.name == sub.name:
                    s.update_status(sub.status)

        if not self.subsys_received:
            self.subsys_received = True

    def subsystemControlRequest(self, subsystem, state):
        try:
            self._subsysService(subsystem, state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def subsystemUpdated_callback(self, sub):
        self.updateButtonProperty(sub.button, "active", sub.status)

    def subsysButton_callback(self):
        btn = self.sender()
        sub = None

        for s in self.subsystems:
            if s.name == btn.text():
                sub = s

        if sub is None:
            return

        rospy.loginfo('requesting %s %s'%(sub.name, not sub.status))
        self.subsystemControlRequest(sub.name, not sub.status)

    def btnCancelGoal_callback(self):
        cancelMessage = GoalID()
        self._cancelGoalPub.publish(cancelMessage)

    def btnEstop_callback(self):
        try:
            self._haltService()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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

