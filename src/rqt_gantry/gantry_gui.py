import rospy
import rospkg
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Signal, QThread, QObject
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from hippocampus_msgs.msg import GantryMotorPosition, GantryMotorVelocity, GantryMotorLimitSwitches


def get_axis_letter(axis):
    if axis == 0:
        return "x"
    elif axis == 1:
        return "y"
    elif axis == 2:
        return "z"
    else:
        raise ValueError("Given axis not defined.")


def call_go_home(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/start_homing".format(axis_letter)
    start_homing = rospy.ServiceProxy(service_name, Trigger)
    try:
        start_homing()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False
    return True


def call_set_home(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/set_home_position".format(axis_letter)
    set_home = rospy.ServiceProxy(service_name, Trigger)
    try:
        set_home()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False
    return True


def call_enable(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/enable".format(axis_letter)
    enable = rospy.ServiceProxy(service_name, Trigger)
    try:
        enable()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False
    return True


def call_disable(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/disable".format(axis_letter)
    disable = rospy.ServiceProxy(service_name, Trigger)
    try:
        disable()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False
    return True


def call_emergency_stop(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/emergency_stop".format(axis_letter)
    emergency_stop = rospy.ServiceProxy(service_name, Trigger)
    try:
        emergency_stop()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False


def call_release_emergency_stop(axis):
    axis_letter = get_axis_letter(axis)
    service_name = "gantry_motor_{}/release_emergency_stop".format(axis_letter)
    release = rospy.ServiceProxy(service_name, Trigger)
    try:
        release()
    except rospy.ServiceException as e:
        rospy.logerr("%s", e)
        return False


class HomingWorkerXY(QObject):
    finished = Signal()

    def __init__(self, axis):
        super(HomingWorkerXY, self).__init__()
        self.axis_letter = ""
        if axis == 0:
            self.axis_letter = "x"
        elif axis == 1:
            self.axis_letter = "y"
        elif axis == 2:
            self.axis_letter = "z"

    def home(self):
        rospy.loginfo("Start homing in %s-direction", self.axis_letter)
        service_name = "gantry_motor_{}/start_homing".format(self.axis_letter)
        start_homing = rospy.ServiceProxy(service_name, Trigger)
        rospy.loginfo("Calling start_homing service for %s", self.axis_letter)
        try:
            start_homing()
        except rospy.ServiceException as e:
            rospy.logwarn("%s", e)
            self.finished.emit()
            return
        service_name = "gantry_motor_{}/is_homing".format(self.axis_letter)
        is_homing = rospy.ServiceProxy(service_name, Trigger)
        while True:
            rospy.loginfo("Calling is_homing service for %s", self.axis_letter)
            try:
                still_homing = is_homing().success
            except rospy.ServiceException as e:
                rospy.logwarn("%s", e)
                self.finished.emit()
                return
            if not still_homing:
                break
            rospy.sleep(1.0)
        service_name = "gantry_motor_{}/set_home_position".format(
            self.axis_letter)
        set_home = rospy.ServiceProxy(service_name, Trigger)
        try:
            set_home()
        except rospy.ServiceException as e:
            rospy.logwarn("%s", e)
        self.finished.emit()


class HomingWorkerZ(QObject):
    finished = Signal()

    def __init__(self):
        super(HomingWorkerZ, self).__init__()

    def home(self):
        set_home = rospy.ServiceProxy("gantry_motor_z/set_home_position",
                                      Trigger)
        try:
            set_home()
        except rospy.ServiceException as e:
            rospy.logwarn("%s", e)
        self.finished.emit()


class ManualControlPlugin(Plugin):
    MODES = [
        "Rel Pos.",
        "Velocity",
    ]
    position_changed = Signal(float, int)
    velocity_changed = Signal(float, int)

    def __init__(self, context):
        super(ManualControlPlugin, self).__init__(context)
        self.setObjectName("GantryGuiPlugin")

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path("rqt_gantry"),
                               "resource", "ManualControl.ui")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName("ManualControl")
        self.init_publishers()
        self.init_modes()
        self.position_changed.connect(self.on_position_changed)
        self.velocity_changed.connect(self.on_velocity_changed)
        self.init_go_home_buttons()
        self.init_set_home_buttons()
        self.init_move_relative_buttons()
        self.init_disable_buttons()
        self.init_enable_buttons()
        self.init_limit_switch_indicators()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (" (%d)" % context.serial_number()))

        context.add_widget(self._widget)
        self.init_subscribers()

    def init_move_relative_buttons(self):
        widget = self._widget.findChild(QWidget, "rel_backward_x")
        widget.clicked.connect(lambda: self.move_relative(0, False))
        widget = self._widget.findChild(QWidget, "rel_forward_x")
        widget.clicked.connect(lambda: self.move_relative(0, True))
        widget = self._widget.findChild(QWidget, "rel_backward_y")
        widget.clicked.connect(lambda: self.move_relative(1, False))
        widget = self._widget.findChild(QWidget, "rel_forward_y")
        widget.clicked.connect(lambda: self.move_relative(1, True))
        widget = self._widget.findChild(QWidget, "rel_backward_z")
        widget.clicked.connect(lambda: self.move_relative(2, False))
        widget = self._widget.findChild(QWidget, "rel_forward_z")
        widget.clicked.connect(lambda: self.move_relative(2, True))

    def move_relative(self, axis, forward):
        step_size_widget = self._widget.findChild(QWidget, "rel_step_size")
        size = step_size_widget.value()
        if not forward:
            size = -size
        msg = Float64(size)
        if axis == 0:
            self.rel_pos_x_pub.publish(msg)
        elif axis == 1:
            self.rel_pos_y_pub.publish(msg)
        elif axis == 2:
            self.rel_pos_z_pub.publish(msg)
        else:
            rospy.logwarn("Given axis for relative position is undefined!")

    def init_go_home_buttons(self):
        widgets = self._get_go_home_xy_widgets()
        for widget in widgets:
            widget.clicked.connect(self.on_go_home_xy)

    def init_set_home_buttons(self):
        widgets = self._get_set_home_xy_widgets()
        for widget in widgets:
            widget.clicked.connect(self.on_set_home_xy)

        widgets = self._get_set_home_z_widgets()
        for widget in widgets:
            widget.clicked.connect(self.on_set_home_z)

    def init_limit_switch_indicators(self):
        for axis in range(3):
            widgets = self._get_limit_switch_widgets(axis)
            for widget in widgets:
                widget.setChecked(False)
                widget.clicked.connect(widget.toggle)

    def init_enable_buttons(self):
        widgets = self._get_enable_widgets()
        for widget in widgets:
            widget.clicked.connect(self.on_enable_clicked)

    def init_disable_buttons(self):
        widgets = self._get_disable_widgets()
        for widget in widgets:
            widget.clicked.connect(self.on_disable_clicked)

    def init_publishers(self):
        self.rel_pos_x_pub = rospy.Publisher(
            "gantry_motor_x/setpoint_position/relative", Float64, queue_size=1)
        self.rel_pos_y_pub = rospy.Publisher(
            "gantry_motor_y/setpoint_position/relative", Float64, queue_size=1)
        self.rel_pos_z_pub = rospy.Publisher(
            "gantry_motor_z/setpoint_position/relative", Float64, queue_size=1)
        self.pubs = [self.rel_pos_x_pub, self.rel_pos_y_pub, self.rel_pos_z_pub]

    def init_subscribers(self):
        self.pos_x_sub = rospy.Subscriber("gantry_motor_x/position",
                                          GantryMotorPosition,
                                          self.on_position_msg,
                                          0,
                                          queue_size=1)
        self.pos_y_sub = rospy.Subscriber("gantry_motor_y/position",
                                          GantryMotorPosition,
                                          self.on_position_msg,
                                          1,
                                          queue_size=1)
        self.pos_z_sub = rospy.Subscriber("gantry_motor_z/position",
                                          GantryMotorPosition,
                                          self.on_position_msg,
                                          2,
                                          queue_size=1)
        self.vel_x_sub = rospy.Subscriber("gantry_motor_x/velocity",
                                          GantryMotorVelocity,
                                          self.on_velocity_msg,
                                          0,
                                          queue_size=1)
        self.vel_y_sub = rospy.Subscriber("gantry_motor_y/velocity",
                                          GantryMotorVelocity,
                                          self.on_velocity_msg,
                                          1,
                                          queue_size=1)
        self.vel_z_sub = rospy.Subscriber("gantry_motor_z/velocity",
                                          GantryMotorVelocity,
                                          self.on_velocity_msg,
                                          2,
                                          queue_size=1)
        self.limit_switch_x_sub = rospy.Subscriber(
            "gantry_motor_x/limit_switches",
            GantryMotorLimitSwitches,
            lambda msg: self.on_limit_switches_msg(msg, 0),
            queue_size=1)

        self.limit_switch_y_sub = rospy.Subscriber(
            "gantry_motor_y/limit_switches",
            GantryMotorLimitSwitches,
            lambda msg: self.on_limit_switches_msg(msg, 1),
            queue_size=1)

        self.limit_switch_z_sub = rospy.Subscriber(
            "gantry_motor_z/limit_switches",
            GantryMotorLimitSwitches,
            lambda msg: self.on_limit_switches_msg(msg, 2),
            queue_size=1)

        self.subs = [
            self.pos_x_sub, self.pos_y_sub, self.pos_z_sub, self.vel_x_sub,
            self.vel_y_sub, self.vel_z_sub
        ]

    def _get_go_home_xy_widgets(self):
        return [
            self._widget.findChild(QWidget, "rel_go_home_xy"),
            self._widget.findChild(QWidget, "vel_go_home_xy")
        ]

    def _get_set_home_xy_widgets(self):
        return [
            self._widget.findChild(QWidget, "rel_set_home_xy"),
            self._widget.findChild(QWidget, "vel_go_home_xy")
        ]

    def _get_set_home_z_widgets(self):
        return [
            self._widget.findChild(QWidget, "rel_set_home_z"),
            self._widget.findChild(QWidget, "vel_set_home_z")
        ]

    def _get_limit_switch_widgets(self, axis):
        axis_letter = get_axis_letter(axis)
        return [
            self._widget.findChild(QWidget,
                                   "ll_{}_indicator".format(axis_letter)),
            self._widget.findChild(QWidget,
                                   "ul_{}_indicator".format(axis_letter)),
        ]

    def _get_enable_widgets(self):
        return [
            self._widget.findChild(QWidget, "rel_enable"),
            self._widget.findChild(QWidget, "vel_enable"),
        ]

    def _get_disable_widgets(self):
        return [
            self._widget.findChild(QWidget, "rel_disable"),
            self._widget.findChild(QWidget, "vel_disable"),
        ]

    def on_go_home_xy(self):
        call_go_home(0)
        call_go_home(1)

    def on_set_home_xy(self):
        call_set_home(0)
        call_set_home(1)

    def on_set_home_z(self):
        call_set_home(2)

    def on_enable_clicked(self):
        for axis in range(3):
            call_enable(axis)

    def on_disable_clicked(self):
        for axis in range(3):
            call_disable(axis)

    def on_position_msg(self, msg, axis):
        self.position_changed.emit(msg.position, axis)

    def on_velocity_msg(self, msg, axis):
        self.velocity_changed.emit(msg.velocity, axis)

    def on_position_changed(self, position, axis):
        text = "{}".format("%6.3F" % position)
        if axis == 0:
            widget = self._widget.findChild(QWidget, "actual_position_x")
        elif axis == 1:
            widget = self._widget.findChild(QWidget, "actual_position_y")
        elif axis == 2:
            widget = self._widget.findChild(QWidget, "actual_position_z")
        widget.setText(text)

    def on_velocity_changed(self, velocity, axis):
        # velocity is displayed in mm/s
        text = "{:6.1f}".format(float(velocity) * 1000.0)
        if axis == 0:
            name = "actual_velocity_x"
        elif axis == 1:
            name = "actual_velocity_y"
        elif axis == 2:
            name = "actual_velocity_z"
        widget = self._widget.findChild(QWidget, name)
        widget.setText(text)

    def on_limit_switches_msg(self, msg, axis):
        widgets = self._get_limit_switch_widgets(axis)
        widgets[0].setChecked(msg.lower_limit)
        widgets[1].setChecked(msg.upper_limit)

    def init_modes(self):
        mode_widget = self._widget.findChild(QWidget, "mode_combobox")
        for i in range(mode_widget.count()):
            mode_widget.removeItem(0)
        mode_widget.addItems(self.MODES)
        mode_widget.currentIndexChanged.connect(self.on_mode_index_changed)

    def on_mode_index_changed(self, index):
        stack = self._widget.findChild(QWidget, "stack_widget")
        stack.setCurrentIndex(index)

    def shutdown_plugin(self):
        for pub in self.pubs:
            pub.unregister()
        for sub in self.subs:
            sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
