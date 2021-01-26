import rospy
import rospkg
import os
import pyqtgraph as pg
import rosnode
import numpy as np
import yaml
from datetime import datetime

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding.QtWidgets as QtWidgets
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtCore import Signal, QThread, QObject
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from hippocampus_msgs.msg import GantryMotorPosition, GantryMotorVelocity, GantryMotorLimitSwitches


def _call_trigger(service_name):
    service = rospy.ServiceProxy(service_name, Trigger)
    result = None
    try:
        result = service()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False, result
    else:
        rospy.loginfo("%s", result)
        return True, result


def call_start(node_name):
    return _call_trigger("{}/start".format(node_name))


def call_stop(node_name):
    return _call_trigger("{}/stop".format(node_name))


def call_move_to_start(node_name):
    return _call_trigger("{}/move_to_start_position".format(node_name))


def call_update_path(node_name):
    return _call_trigger("{}/update_path".format(node_name))


class PlotDataContainer(object):
    def __init__(self, pos_line):
        self.pos_x = np.array([])
        self.pos_y = np.array([])
        self.pos_stamp = np.array([])
        self.buffer_time = 30.0
        self.pos_pen = pg.mkPen(color=(255, 0, 0), width=2)
        self.pos_line = pos_line

    def update_pos_with_time_limit(self, now):
        threshold = now - self.buffer_time
        for i, stamp in enumerate(self.pos_stamp):
            if stamp > threshold:
                break
        if i == 0:
            return
        self.pos_x = self.pos_x[i:]
        self.pos_y = self.pos_y[i:]
        self.pos_stamp = self.pos_stamp[i:]

    def add_data(self, x, y, stamps):
        if not (len(x) == len(y) == len(stamps)):
            return False

        self.pos_x = np.append(self.pos_x, x)
        self.pos_y = np.append(self.pos_y, y)
        self.pos_stamp = np.append(self.pos_stamp, stamps)
        return True


class PathFollowerPlugin(Plugin):
    def __init__(self, context):
        super(PathFollowerPlugin, self).__init__(context)
        self.plot_data = []
        self.position_subs = []
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        ui_file = os.path.join(rospkg.RosPack().get_path("rqt_gantry"),
                               "resource", "PathFollower.ui")
        self._widget = QtWidgets.QWidget()
        loadUi(ui_file, self._widget)
        self._init_plot()
        self._init_buttons()
        self._init_node_selection()
        context.add_widget(self._widget)
        self.update_position_subs(self.get_selected_node())

    def _init_node_selection(self):
        namespaces = self.get_namespaces()
        ns_combo = self._widget.namespace_combobox
        ns_combo.clear()
        ns_combo.addItems(namespaces)
        ns_combo.currentTextChanged.connect(self.on_namespace_changed)
        node_combo = self._widget.node_combobox
        node_combo.currentTextChanged.connect(self.on_node_changed)

        current_ns = ns_combo.currentText()
        self.on_namespace_changed(current_ns)

    def _log(self, level, text):
        console = self._widget.console
        time = datetime.fromtimestamp(rospy.get_time())
        output = "[{}] [{}] {}".format(level, time.strftime("%H:%M:%S"), text)
        current_scrollbar = console.verticalScrollBar().value()
        max_scrollbar = console.verticalScrollBar().maximum()
        autoscroll =  current_scrollbar == max_scrollbar

        console.appendPlainText(output)
        if autoscroll:
            console.verticalScrollBar().setValue(console.verticalScrollBar().maximum())

    def loginfo(self, text):
        self._log("INFO", text)
    
    def logwarn(self, text):
        self._log("WARN", text)

    def logerr(self, text):
        self._log("ERROR", text)


    def on_namespace_changed(self, namespace):
        node_combo = self._widget.node_combobox
        node_combo.clear()
        nodes = rosnode.get_node_names(namespace)
        node_combo.addItems(nodes)

    def on_node_changed(self, node):
        self.update_position_subs(node)

    def get_namespaces(self):
        nodes = rosnode.get_node_names()
        node_namespaces = []
        for node in nodes:
            ns = os.path.dirname(node)
            if ns not in node_namespaces:
                node_namespaces.append(ns)
        return node_namespaces

    def get_selected_node(self):
        node_combo = self._widget.node_combobox
        return node_combo.currentText()

    def _init_plot(self):
        tank_size = (2, 4)
        self._widget.plot.setAspectLocked()
        self._widget.plot.plot([0, tank_size[0], tank_size[0], 0, 0],
                               [0, 0, tank_size[1], tank_size[1], 0])
        pos_line = self._widget.plot.plot()
        self.plot_data = PlotDataContainer(pos_line)

    def _init_buttons(self):
        button = self._widget.open_path_button
        button.clicked.connect(self.on_open_path)

        button = self._widget.upload_path_button
        button.clicked.connect(self.on_upload_path)

        button = self._widget.start_button
        button.clicked.connect(self.on_start)

        button = self._widget.stop_button
        button.clicked.connect(self.on_stop)

        button = self._widget.move_to_start_button
        button.clicked.connect(self.on_move_to_start)

    def on_open_path(self):
        dialog = QtWidgets.QFileDialog()
        dialog.setFileMode(QtWidgets.QFileDialog.ExistingFile)
        dialog.setMimeTypeFilters(["application/x-yaml"])
        filenames = []
        if dialog.exec_():
            filenames = dialog.selectedFiles()
            self._widget.path_file_edit.setText(filenames[0])

    def on_upload_path(self):
        filename = self._widget.path_file_edit.text()
        if not os.path.isfile(filename):
            mbox = QtWidgets.QMessageBox()
            mbox.setIcon(QtWidgets.QMessageBox.Critical)
            mbox.setText("The file '{}' does not exist.".format(filename))
            mbox.setInformativeText("Please select a valid YAML file.")
            mbox.setWindowTitle("File does not exist.")
            mbox.exec_()
        with open(filename) as f:
            data = yaml.load(f)
        rospy.set_param(self.get_selected_node(), data)
        self.handle_trigger_response(
            *call_update_path(self.get_selected_node()), show_success=True)

    def on_start(self):
        self.handle_trigger_response(*call_start(self.get_selected_node()))

    def on_stop(self):
        self.handle_trigger_response(call_stop(self.get_selected_node()))

    def on_move_to_start(self):
        self.handle_trigger_response(
            *call_move_to_start(self.get_selected_node()))

    def update_position_subs(self, node_name, unsubscribe_only=False):
        rospy.loginfo("Subscribing to %s", node_name)
        self.loginfo("Subscribing to {}.".format(node_name))
        for sub in self.position_subs:
            sub.unregister()
        self.position_subs = []
        if not unsubscribe_only:
            self.pos_sub = rospy.Subscriber(
                "{}/current_position".format(node_name),
                Point,
                self.on_current_position,
                queue_size=30)
            self.position_subs.append(self.pos_sub)

    def on_current_position(self, msg):
        self.plot_data.add_data([msg.x], [msg.y], [rospy.get_time()])
        self.plot_data.update_pos_with_time_limit(rospy.get_time())
        self.plot_data.pos_line.setData(self.plot_data.pos_x,
                                        self.plot_data.pos_y,
                                        clear=True,
                                        pen=self.plot_data.pos_pen)

    def create_message_service_unavailable(self):
        mbox = QtWidgets.QMessageBox()
        mbox.setWindowTitle("Service unavailable")
        mbox.setText("The called service is unavailable.")
        mbox.setInformativeText("Are you sure '{}' is the correct node?".format(
            self.get_selected_node()))
        mbox.setIcon(QtWidgets.QMessageBox.Critical)
        return mbox

    def create_message_trigger_service_fail(self, message):
        mbox = QtWidgets.QMessageBox()
        mbox.setWindowTitle("Service failed.")
        mbox.setText("The service call was not successfull.")
        mbox.setDetailedText("{}".format(message))
        mbox.setIcon(QtWidgets.QMessageBox.Warning)
        return mbox

    def create_message_trigger_service_success(self, message):
        mbox = QtWidgets.QMessageBox()
        mbox.setWindowTitle("Service executed.")
        mbox.setText("Service was exectued successfully.")
        mbox.setDetailedText("{}".format(message))
        mbox.setIcon(QtWidgets.QMessageBox.Information)
        return mbox

    def handle_trigger_response(self, success, response, show_success=False):
        if not success:
            mbox = self.create_message_service_unavailable()
            mbox.exec_()
        else:
            if not response.success:
                mbox = self.create_message_trigger_service_fail(
                    response.message)
                mbox.exec_()
            else:
                rospy.loginfo("Service reponse: %s", response.message)
                self.loginfo("Service response: {}".format(response.message))
                if show_success:
                    mbox = self.create_message_trigger_service_success(
                        response.message)
                    mbox.exec_()

    def shutdown_plugin(self):
        for sub in self.position_subs:
            sub.unregister()
