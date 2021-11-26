import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem, QPushButton
from std_srvs.srv import Trigger
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Pose
from herox_coordinator.msg import Mission as MissionMessage, Missions as MissionsMessage
from herox_coordinator.srv import SubsystemControl, SubsystemControlResponse, LoadMission, NewMission, SaveMission, WaypointAction

# Data model for the table view
#class WaypointModel(QtCore.QAbstractTableModel):
#    header_labels = ['ID','Tag','Visits','Avg. SPL']
#
#    def __init__(self,waypoints):
#        super(WaypointModel, self).__init__()
#        self.waypoints = waypoints
#
#    def setWaypoints(self, wps):
#        self.waypoints = wps
#        self.dataChanged.emit(self.index(0,0), self.index(len(self.header_labels),len(self.waypoints)), [])
#
#    def data(self, index, role):
#        if role == Qt.DisplayRole:
#            if index.column() == 0:
#                return self.waypoints[index.row()].id
#            if index.column() == 1:
#                return self.waypoints[index.row()].tagname
#            if index.column() == 2:
#                return len(self.waypoints[index.row()].visits)
#            if index.column() == 3:
#                return -1.0
#
#    def headerData(self, section, orientation, role=Qt.DisplayRole):
#        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
#            return self.header_labels[section]
#        return QtCore.QAbstractTableModel.headerData(self, section, orientation, role)
#
#    def rowCount(self, index):
#        return len(self.waypoints)
#
#    def columnCount(self, index):
#        return 4

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

class HeroxMissionEditor(Plugin):
    missionsUpdated = QtCore.pyqtSignal(object)
    currentMissionUpdated = QtCore.pyqtSignal(object)
    missions = []
    currentMission = None
    lockTags = False

    def __init__(self, context):
        super(HeroxMissionEditor, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HeroxMissionEditor')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_herox_missioneditor'), 'resource', 'HeroxMissionEditor.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('HeroxMissionEditorUI')

        self._loadService = rospy.ServiceProxy('/load_mission', LoadMission)
        self._newService = rospy.ServiceProxy('/new_mission', NewMission)
        self._saveService = rospy.ServiceProxy('/save_mission', SaveMission)
        self._startService = rospy.ServiceProxy('/start_mission', Trigger)
        self._stopService = rospy.ServiceProxy('/stop_mission', Trigger)
        self._addWPService = rospy.ServiceProxy('/add_waypoint', Trigger)
        self._wpUpService = rospy.ServiceProxy('/waypoint_up', WaypointAction)
        self._wpDownService = rospy.ServiceProxy('/waypoint_down', WaypointAction)
        self._wpSetTagService = rospy.ServiceProxy('/waypoint_set_tag', WaypointAction)
        self._wpSetIDService = rospy.ServiceProxy('/waypoint_set_id', WaypointAction)
        self._wpRemoveService = rospy.ServiceProxy('/waypoint_remove', WaypointAction)

        # Set up button callbacks
        self._widget.btnLoadMission.clicked.connect(self.btnLoad_callback)
        self._widget.btnSaveMission.clicked.connect(self.btnSave_callback)
        self._widget.btnAddWaypoint.clicked.connect(self.btnAddWP_callback)
        self._widget.btnStartMission.clicked.connect(self.btnStart_callback)
        self._widget.btnStopMission.clicked.connect(self.btnStop_callback)
        self._widget.btnRemoveWaypoint.clicked.connect(self.btnRemoveWP_callback)
        self._widget.btnMoveUp.clicked.connect(self.btnMoveUp_callback)
        self._widget.btnMoveDown.clicked.connect(self.btnMoveDown_callback)

        self.missionsUpdated.connect(self.missionsUpdatedCallback)
        self.currentMissionUpdated.connect(self.currentMissionUpdatedCallback)

        self._widget.twWaypoints.setColumnCount(4)
        self._widget.twWaypoints.setHorizontalHeaderLabels(['ID','Tag','Visits','Avg. SPL'])
        self._widget.twWaypoints.cellChanged.connect(self.cellChangedCallback)
        
        self._widget.setWindowTitle('HEROX Mission Editor')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._missionsSub = rospy.Subscriber('missions', MissionsMessage, self.missionsCallback)
        rospy.Subscriber('current_mission', MissionMessage, self.currentMissionCallback)

    def missionsCallback(self, msg):
        self.missions = msg.missions
        self.missionsUpdated.emit(msg.missions)

    def missionsUpdatedCallback(self, missions):
        self._widget.cbxMission.clear()
        self._widget.cbxMission.addItems(missions)

    def currentMissionCallback(self, msg):
        print("Received current mission: " + msg.name)
        self.currentMission = msg
        self.currentMissionUpdated.emit(msg)

    def lockTags(self):
        self._widget.twWaypoints.cellChanged.disconnect(self.cellChangedCallback)
    def unlockTags(self):
        self._widget.twWaypoints.cellChanged.connect(self.cellChangedCallback)
        
    def currentMissionUpdatedCallback(self, mission):
        print("mission updated callback, waypoints: " + str(len(mission.waypoints)))
        self.lockTags()
        self._widget.twWaypoints.setRowCount(0)
        for wp in mission.waypoints:
            row = self._widget.twWaypoints.rowCount()
            self._widget.twWaypoints.insertRow(row)
            self._widget.twWaypoints.setItem(row, 0, QTableWidgetItem(str(wp.id)))
            self._widget.twWaypoints.setItem(row, 1, QTableWidgetItem(wp.tagname))
            self._widget.twWaypoints.setItem(row, 2, QTableWidgetItem(str(len(wp.visits))))
            self._widget.twWaypoints.setItem(row, 3, QTableWidgetItem("N/A"))
        self.unlockTags()

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
        
    def btnRemoveWP_callback(self):
        selection = self._widget.twWaypoints.selectedRanges()
        if (len(selection) == 0):
            return

        row = selection[0].topRow()

        print('Requesting to remove selected WP ' + str(row))
        self._wpRemoveService(row, "")

    def btnMoveUp_callback(self):
        selection = self._widget.twWaypoints.selectedRanges()
        if (len(selection) == 0):
            return

        row = selection[0].topRow()
        if row == 0:
            return

        self._wpUpService(row, "")
        #for c in range(self._widget.twWaypoints.columnCount()):
        #    self.tblSwapItems(row,c,row-1,c)

        #self._widget.twWaypoints.selectRow(row-1)

    def btnMoveDown_callback(self):
        selection = self._widget.twWaypoints.selectedRanges()
        if (len(selection) == 0):
            return

        row = selection[0].topRow()
        if row == self._widget.twWaypoints.rowCount() - 1:
            return

        self._wpDownService(row, "")
        #for c in range(self._widget.twWaypoints.columnCount()):
        #    self.tblSwapItems(row,c,row+1,c)

        #self._widget.twWaypoints.selectRow(row+1)

    def cellChangedCallback(self, row, column):
        if column == 1:
            print("Tag changed! " + str(row) + " " + str(column))
            self._wpSetTagService(row, self._widget.twWaypoints.item(row,1).text())
        if column == 0:
            print("ID changed! " + str(row) + " " + str(column))
            self._wpSetIDService(row, self._widget.twWaypoints.item(row,0).text())

    def tblSwapItems(self,r1,c1,r2,c2):
        i1 = self._widget.twWaypoints.takeItem(r1,c1)
        i2 = self._widget.twWaypoints.takeItem(r2,c2)
        self._widget.twWaypoints.setItem(r1,c1,i2)
        self._widget.twWaypoints.setItem(r2,c2,i1)

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

