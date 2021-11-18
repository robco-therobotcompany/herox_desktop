import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from herox_coordinator.msg import SubsystemStatus, SubsystemStatusSingle
from herox_coordinator.srv import SubsystemControl, SubsystemControlResponse, LoadMission, NewMission, SaveMission
from std_srvs.srv import Trigger
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Pose

baseState = False
navState = False
teleopState = False

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

        # Set up publishers
        self._cancelGoalPub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Set up services
        print("Waiting for services...")
        rospy.wait_for_service('/subsystem_control')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /subsystem_control')
        rospy.wait_for_service('/driver/halt')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /driver/halt')
        rospy.wait_for_service('/load_mission')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /load_mission')
        rospy.wait_for_service('/new_mission')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /new_mission')
        rospy.wait_for_service('/save_mission')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /save_mission')
        rospy.wait_for_service('/start_mission')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /start_mission')
        rospy.wait_for_service('/stop_mission')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /stop_mission')
        rospy.wait_for_service('/add_waypoint')
        print(bcolors.OKGREEN + u'\u2713' + bcolors.ENDC + ' /add_waypoint')

        print(bcolors.OKGREEN + 'All services online.' + bcolors.ENDC)

        self._haltService = rospy.ServiceProxy('/driver/halt', Trigger)
        self._loadService = rospy.ServiceProxy('/load_mission', LoadMission)
        self._newService = rospy.ServiceProxy('/new_mission', NewMission)
        self._saveService = rospy.ServiceProxy('/save_mission', SaveMission)
        self._startService = rospy.ServiceProxy('/start_mission', Trigger)
        self._stopService = rospy.ServiceProxy('/stop_mission', Trigger)
        self._addWPService = rospy.ServiceProxy('/add_waypoint', Trigger)

        self._subsysService = rospy.ServiceProxy('/subsystem_control', SubsystemControl)

        # Set up button callbacks
        self._widget.btnBase.clicked.connect(self.btnBase_callback)
        self._widget.btnNav.clicked.connect(self.btnNav_callback)
        self._widget.btnTeleop.clicked.connect(self.btnTeleop_callback)
        self._widget.btnCancelGoal.clicked.connect(self.btnCancelGoal_callback)
        self._widget.btnEstop.clicked.connect(self.btnEstop_callback)
        self._widget.btnLoadMission.clicked.connect(self.btnLoad_callback)
        self._widget.btnSaveMission.clicked.connect(self.btnSave_callback)
        self._widget.btnAddWaypoint.clicked.connect(self.btnAddWP_callback)
        self._widget.btnStartMission.clicked.connect(self.btnStart_callback)
        self._widget.btnStopMission.clicked.connect(self.btnStop_callback)
        
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
        global baseState
        global navState
        global teleopState

        for sub in msg.subsystems:
            if sub.name == "BASE":
                baseState = sub.status
                self.updateButtonProperty(self._widget.btnBase, "active", baseState)
            elif sub.name == "NAV":
                navState = sub.status
                self.updateButtonProperty(self._widget.btnNav, "active", navState)
            elif sub.name == "TELEOP":
                teleopState = sub.status
                self.updateButtonProperty(self._widget.btnTeleop, "active", teleopState)

    def subsystemControlRequest(self, subsystem, state):
        try:
            self._subsysService(subsystem, state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def btnBase_callback(self):
        global baseState

        rospy.loginfo('requesting BASE %s'%(not baseState))
        self.subsystemControlRequest('BASE', not baseState)

    def btnNav_callback(self):
        global navState
        rospy.loginfo('requesting NAV %s'%(not navState))
        self.subsystemControlRequest('NAV', not navState)

    def btnTeleop_callback(self):
        global teleopState
        rospy.loginfo('requesting TELEOP %s'%(not teleopState))
        self.subsystemControlRequest('TELEOP', not teleopState)

    def btnCancelGoal_callback(self):
        cancelMessage = GoalID()
        self._cancelGoalPub.publish(cancelMessage)

    def btnEstop_callback(self):
        try:
            self._haltService()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def btnLoad_callback(self):
        mission = self._widget.cbxMission.currentText()
        print('Requesting to load mission ' + mission)
        self._loadService(mission)

    def btnSave_callback(self):
        mission = self._widget.cbxMission.currentText()
        print('Requesting to save mission ' + mission)
        self._saveService(mission)
        
    def btnAddWP_callback(self):
        print('Requesting to add WP at current pose')
        self._addWPService()

    def btnStart_callback(self):
        print('Requesting to start mission')
        self._startService()

    def btnStop_callback(self):
        print('Requesting to stop mission')
        self._stopService()

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

