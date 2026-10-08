"""Qt dashboard for the measured SO100 forward-kinematics learning report.

The window is intentionally independent of ROS so its rendering can be checked
offscreen. ROS callbacks are serviced by a nonblocking Qt timer on the GUI
thread; no background executor or competing QApplication is created.
"""

import json
import math
import signal
import sys

from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from .kinematics import ARM_JOINTS, GRIPPER_JOINT, JOINT_ORIGINS


def _number(value):
    """Allow unavailable fields while rejecting nonfinite display values."""
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _format(value, precision=5):
    result = _number(value)
    return '\N{EM DASH}' if result is None else f'{result:.{precision}f}'


def _vector(value, length=3):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return [None] * length
    return [_number(item) for item in value]


def _matrix(value, rows, columns):
    if not isinstance(value, (list, tuple)) or len(value) != rows:
        return [[None] * columns for _ in range(rows)]
    return [_vector(row, columns) for row in value]


def _rpy_from_matrix(value):
    """Display fallback only; FK itself always comes from the report producer."""
    matrix = _matrix(value, 4, 4)
    if any(element is None for row in matrix[:3] for element in row[:3]):
        return [None] * 3
    pitch = math.atan2(-matrix[2][0], math.hypot(matrix[0][0], matrix[1][0]))
    if math.hypot(matrix[0][0], matrix[1][0]) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return [roll, pitch, yaw]


class MatrixPanel(QtWidgets.QGroupBox):
    """Compact, read-only homogeneous matrix with a translation-column tint."""

    def __init__(self, title, parent=None):
        super().__init__(title, parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.table = QtWidgets.QTableWidget(4, 4)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.horizontalHeader().hide()
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setFixedHeight(132)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        font.setPointSize(10)
        self.table.setFont(font)
        for row in range(4):
            for column in range(4):
                item = QtWidgets.QTableWidgetItem('\N{EM DASH}')
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if column == 3 and row < 3:
                    item.setBackground(QtGui.QColor('#edf7f6'))
                self.table.setItem(row, column, item)
        layout.addWidget(self.table)

    def update_matrix(self, value):
        for row, values in enumerate(_matrix(value, 4, 4)):
            for column, element in enumerate(values):
                self.table.item(row, column).setText(_format(element))


class DashboardWindow(QtWidgets.QMainWindow):
    """Render the /fk/report JSON contract without requiring a ROS node."""

    def __init__(self, on_selected=None):
        super().__init__()
        self.on_selected = on_selected
        self.report = {}
        self.elapsed = QtCore.QElapsedTimer()
        self.setWindowTitle('SO100 · Forward kinematics lab')
        body_font = self.font()
        body_font.setPointSize(10)
        self.setFont(body_font)
        self.resize(1120, 860)
        self.setMinimumSize(850, 650)
        self.setStyleSheet('''
            QWidget { font-size: 13px; }
            QMainWindow, QWidget#dashboard { background: #f4f6f8; color: #1d2939; }
            QGroupBox { background: white; border: 1px solid #d6dce3;
                border-radius: 6px; margin-top: 14px; padding-top: 10px;
                font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QTableWidget { background: white; gridline-color: #e4e9ef; border: 0; }
            QHeaderView::section { background: #edf1f5; padding: 8px; border: 0;
                color: #344054; font-weight: 600; }
            QTabWidget::pane { background: #f4f6f8; border: 1px solid #d6dce3;
                border-radius: 5px; }
            QTabBar::tab { padding: 10px 18px; background: #e8edf3; }
            QTabBar::tab:selected { background: white; color: #087f8c; }
            QComboBox { background: white; padding: 7px 10px; min-width: 170px; }
        ''')
        root = QtWidgets.QWidget()
        root.setObjectName('dashboard')
        root.setFont(body_font)
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(20, 15, 20, 12)
        layout.setSpacing(10)
        title = QtWidgets.QLabel('Forward kinematics lab')
        font = title.font()
        font.setPointSize(21)
        font.setBold(True)
        title.setFont(font)
        title.setStyleSheet('font-size: 28px; font-weight: 700;')
        layout.addWidget(title)
        description = QtWidgets.QLabel(
            'Move a joint slider, then follow how its measured angle changes the TCP. '
            'Matrices and the Jacobian use measured joint states.')
        description.setWordWrap(True)
        layout.addWidget(description)

        selection = QtWidgets.QHBoxLayout()
        selection.addWidget(QtWidgets.QLabel('Inspect joint'))
        self.selected = QtWidgets.QComboBox()
        self.selected.addItems(ARM_JOINTS)
        self.selected.setCurrentIndex(1)
        self.selected.currentTextChanged.connect(self._selected_changed)
        selection.addWidget(self.selected)
        selection.addWidget(QtWidgets.QLabel('The selected axis is highlighted in RViz.'))
        selection.addStretch(1)
        layout.addLayout(selection)
        self.status = QtWidgets.QLabel('Waiting for /fk/report …')
        self.status.setWordWrap(True)
        self.status.setStyleSheet('color: #596579; padding: 5px 0;')
        layout.addWidget(self.status)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        self._build_pose_tab()
        self._build_chain_tab()
        self._build_jacobian_tab()
        source = QtWidgets.QLabel(
            'Algorithm: so_arm_gazebo/kinematics.py · forward_kinematics_chain(), '
            'position_jacobian()\n'
            'Robot data: urdf/so100.urdf.xacro · joint origins, axes, limits and fixed tool0 offset')
        source.setWordWrap(True)
        source.setStyleSheet('font-size: 11px; color: #667085;')
        layout.addWidget(source)
        self.freshness_timer = QtCore.QTimer(self)
        self.freshness_timer.timeout.connect(self._refresh_status)
        self.freshness_timer.start(500)

    @property
    def selected_joint(self):
        return self.selected.currentText()

    def _build_pose_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setSpacing(10)
        self.joints = QtWidgets.QTableWidget(6, 6)
        self.joints.setHorizontalHeaderLabels([
            'Joint', 'Desired · rad', 'Desired · deg', 'Measured · rad',
            'Measured · deg', 'Error · deg',
        ])
        self.joints.horizontalHeaderItem(5).setToolTip('Measured angle minus desired angle, in degrees')
        self.joints.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.joints.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.joints.verticalHeader().hide()
        self.joints.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.joints.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.joints.verticalHeader().setDefaultSectionSize(28)
        self.joints.setFixedHeight(210)
        for row, name in enumerate((*ARM_JOINTS, GRIPPER_JOINT)):
            for column in range(6):
                item = QtWidgets.QTableWidgetItem(name if column == 0 else '\N{EM DASH}')
                item.setTextAlignment(QtCore.Qt.AlignVCenter |
                                      (QtCore.Qt.AlignLeft if column == 0 else QtCore.Qt.AlignRight))
                self.joints.setItem(row, column, item)
        layout.addWidget(self.joints)
        note = QtWidgets.QLabel(
            'All six joints are revolute: values are radians. The gripper joint moves the '
            'jaw only; it does not change tool0 or the five-joint TCP chain.')
        note.setWordWrap(True)
        layout.addWidget(note)
        pose_group = QtWidgets.QGroupBox('TCP pose in the world frame')
        pose_layout = QtWidgets.QGridLayout(pose_group)
        self.xyz = QtWidgets.QLabel('x —   y —   z — m')
        self.rpy = QtWidgets.QLabel('roll —   pitch —   yaw — deg')
        self.tf_xyz = QtWidgets.QLabel('Awaiting world → tool0 TF')
        self.tf_error = QtWidgets.QLabel('Position error: —')
        for row, (label, field) in enumerate((
                ('FK position', self.xyz), ('FK orientation', self.rpy),
                ('TF position', self.tf_xyz), ('FK ↔ TF check', self.tf_error))):
            pose_layout.addWidget(QtWidgets.QLabel(label), row, 0)
            pose_layout.addWidget(field, row, 1)
        pose_layout.setColumnStretch(1, 1)
        layout.addWidget(pose_group)
        matrices = QtWidgets.QHBoxLayout()
        self.base_tool = MatrixPanel('T_base_tool0')
        self.world_tool = MatrixPanel('T_world_tool0 = T_world_base × T_base_tool0')
        matrices.addWidget(self.base_tool)
        matrices.addWidget(self.world_tool)
        layout.addLayout(matrices)
        layout.addStretch(1)
        self._add_tab(tab, 'Angles & TCP pose')

    def _build_chain_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        explanation = QtWidgets.QLabel(
            '<b>For each joint:</b> '
            'T_local(q) = Trans(xyz) · Rz(yaw) · Ry(pitch) · Rx(roll) · Rot(axis, q).<br>'
            'The URDF <b>origin</b> is the fixed parent-to-joint transform at zero angle; '
            '<b>motion</b> rotates about the joint-local axis. Multiply in this order.')
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.origin_values = QtWidgets.QLabel()
        self.origin_values.setWordWrap(True)
        self.origin_values.setStyleSheet('color: #087f8c; padding: 7px 0;')
        layout.addWidget(self.origin_values)
        grid = QtWidgets.QGridLayout()
        self.origin = MatrixPanel('1 · Fixed URDF origin')
        self.motion = MatrixPanel('2 · Joint motion Rot(axis, q)')
        self.local = MatrixPanel('3 · Local = origin × motion')
        self.cumulative = MatrixPanel('4 · Accumulated: base → selected child')
        grid.addWidget(self.origin, 0, 0)
        grid.addWidget(self.motion, 0, 1)
        grid.addWidget(self.local, 1, 0)
        grid.addWidget(self.cumulative, 1, 1)
        layout.addLayout(grid)
        chain = QtWidgets.QLabel(
            '<b>Whole chain:</b> T_base_tool0 = T₁(q₁) · T₂(q₂) · T₃(q₃) · '
            'T₄(q₄) · T₅(q₅) · T_gripper_tool0.<br>'
            'The fixed TCP offset is [0, −0.100, 0] m in gripper_link. '
            'The world transform places the base on the tabletop at z = 0.60 m.<br>'
            'Highlighted last columns are translations in metres; the top-left 3 × 3 blocks are rotations.')
        chain.setWordWrap(True)
        layout.addWidget(chain)
        layout.addStretch(1)
        self._add_tab(tab, 'Transform chain')
        self._render_chain()

    def _build_jacobian_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        intro = QtWidgets.QLabel(
            '<b>Position Jacobian:</b> Δp ≈ J(q) Δq. '
            'Rows are world x, y, z; columns follow the five arm joints.<br>'
            'Each column tells you how TCP position changes per radian of one joint. '
            'These are local, first-order estimates at the current measured pose.')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.jacobian = QtWidgets.QTableWidget(3, 5)
        self.jacobian.setHorizontalHeaderLabels(list(ARM_JOINTS))
        self.jacobian.setVerticalHeaderLabels(['x · m/rad', 'y · m/rad', 'z · m/rad'])
        self.jacobian.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.jacobian.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.jacobian.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.jacobian.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.jacobian.setMinimumHeight(170)
        self.jacobian.setMaximumHeight(210)
        for row in range(3):
            for column in range(5):
                item = QtWidgets.QTableWidgetItem('\N{EM DASH}')
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.jacobian.setItem(row, column, item)
        layout.addWidget(self.jacobian)
        group = QtWidgets.QGroupBox('Try a small change')
        group_layout = QtWidgets.QVBoxLayout(group)
        self.displacement = QtWidgets.QLabel()
        self.displacement.setWordWrap(True)
        self.displacement.setStyleSheet('font-size: 16px; padding: 12px; color: #087f8c;')
        group_layout.addWidget(self.displacement)
        layout.addWidget(group)
        note = QtWidgets.QLabel(
            'For +1°, multiply the selected column by π / 180. '
            'The displayed result is in millimetres. Large slider changes need a fresh FK '
            'calculation because the Jacobian changes with the pose.<br><br>'
            'Wrist roll can rotate tool0 while its position column stays near zero: '
            'the chosen TCP lies on that rotation axis. A position Jacobian does not '
            'describe orientation changes.')
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch(1)
        self._add_tab(tab, 'Jacobian · small changes')
        self._render_jacobian()

    def _add_tab(self, content, title):
        # Keep full rows and matrix labels readable on smaller laptop displays.
        # A scroll area avoids Qt compressing wrapped labels into adjacent data.
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(content)
        self.tabs.addTab(scroll, title)

    def _selected_changed(self, name):
        if self.on_selected:
            self.on_selected(name)
        self._render_chain()
        self._render_jacobian()
        self._highlight_joint()

    def update_report(self, report):
        """Render valid JSON reports; missing fields remain visibly unavailable."""
        if not isinstance(report, dict):
            raise ValueError('FK report must be a JSON object')
        self.report = report
        self.elapsed.start()
        names = report.get('joint_names', [])
        desired = report.get('desired_positions', [])
        measured = report.get('measured_positions', [])
        names = names if isinstance(names, list) else []
        desired = desired if isinstance(desired, list) else []
        measured = measured if isinstance(measured, list) else []
        desired_by_name = dict(zip(names, desired))
        measured_by_name = dict(zip(names, measured))
        for row, name in enumerate((*ARM_JOINTS, GRIPPER_JOINT)):
            command = _number(desired_by_name.get(name))
            actual = _number(measured_by_name.get(name))
            command_deg = None if command is None else math.degrees(command)
            actual_deg = None if actual is None else math.degrees(actual)
            delta = None if command is None or actual is None else math.degrees(actual - command)
            for column, value in enumerate((command, command_deg, actual, actual_deg, delta), 1):
                self.joints.item(row, column).setText(_format(value, 4 if column in (1, 3) else 2))
        self._highlight_joint()
        position = _vector(report.get('tcp_world'))
        self.xyz.setText('    '.join(f'{axis} {_format(value, 4)}' for axis, value in zip('xyz', position)) + ' m')
        angles = _vector(report.get('rpy_world'))
        if all(value is None for value in angles):
            angles = _rpy_from_matrix(report.get('world_tool'))
        self.rpy.setText('    '.join(
            f'{axis} {_format(None if value is None else math.degrees(value), 2)}°'
            for axis, value in zip(('roll', 'pitch', 'yaw'), angles)))
        tf_position = _vector(report.get('tcp_tf_world'))
        if all(value is None for value in tf_position):
            self.tf_xyz.setText('Awaiting world → tool0 TF')
        else:
            self.tf_xyz.setText('    '.join(
                f'{axis} {_format(value, 4)}' for axis, value in zip('xyz', tf_position)) + ' m')
        error = _number(report.get('tf_position_error'))
        self.tf_error.setText('Position error: ' + ('—' if error is None else f'{error * 1000.0:.3f} mm'))
        self.base_tool.update_matrix(report.get('base_tool'))
        self.world_tool.update_matrix(report.get('world_tool'))
        self._render_chain()
        self._render_jacobian()
        self._refresh_status()

    def _highlight_joint(self):
        for row, name in enumerate((*ARM_JOINTS, GRIPPER_JOINT)):
            color = QtGui.QColor('#e4f5f3' if name == self.selected_joint else '#ffffff')
            for column in range(6):
                self.joints.item(row, column).setBackground(color)

    def _render_chain(self):
        index = ARM_JOINTS.index(self.selected_joint)
        xyz, rpy, axis = JOINT_ORIGINS[index]
        chain = self.report.get('chain', [])
        if isinstance(chain, dict):
            chain = chain.get('steps', [])
        if not isinstance(chain, list):
            chain = []
        step = next((item for item in chain if isinstance(item, dict)
                     and item.get('name') == self.selected_joint), {})
        self.origin_values.setText(
            f'{self.selected_joint}:  origin xyz {list(xyz)} m  |  '
            f'origin rpy {list(rpy)} rad  |  axis {list(axis)}  |  '
            f'measured q = {_format(step.get("angle"), 4)} rad')
        for panel, key in ((self.origin, 'origin'), (self.motion, 'motion'),
                           (self.local, 'local'), (self.cumulative, 'cumulative')):
            panel.update_matrix(step.get(key))

    def _render_jacobian(self):
        index = ARM_JOINTS.index(self.selected_joint)
        jacobian = _matrix(self.report.get('jacobian'), 3, 5)
        for row, values in enumerate(jacobian):
            for column, value in enumerate(values):
                item = self.jacobian.item(row, column)
                item.setText(_format(value, 5))
                item.setBackground(QtGui.QColor('#e4f5f3' if column == index else '#ffffff'))
        predicted = [None if row[index] is None else row[index] * math.pi / 180.0 * 1000.0
                     for row in jacobian]
        self.displacement.setText(
            f'If {self.selected_joint} increases by +1°:\n'
            + '    '.join(f'Δ{axis} ≈ {_format(value, 3)} mm'
                         for axis, value in zip('xyz', predicted)))

    def _refresh_status(self):
        if not self.elapsed.isValid():
            return
        age = self.elapsed.elapsed() / 1000.0
        state = str(self.report.get('status', 'Receiving measured joint states'))
        if age > 3.0:
            self.status.setText(f'Report stale · last update {age:.1f} s ago · {state}')
            self.status.setStyleSheet('color: #a55b00; padding: 5px 0;')
        elif self.report.get('ready', False):
            self.status.setText(f'Live · {state}')
            self.status.setStyleSheet('color: #087f8c; padding: 5px 0;')
        else:
            self.status.setText(f'Waiting · {state}')
            self.status.setStyleSheet('color: #596579; padding: 5px 0;')


class DashboardNode(Node):
    def __init__(self):
        super().__init__('fk_dashboard')
        self.window = None
        selected_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.selected_publisher = self.create_publisher(String, '/fk/selected_joint', selected_qos)
        self.report_subscription = self.create_subscription(String, '/fk/report', self._report, 10)

    def publish_selected(self, name):
        self.selected_publisher.publish(String(data=name))

    def _report(self, message):
        if self.window is None:
            return
        try:
            self.window.update_report(json.loads(message.data))
        except (ValueError, TypeError, KeyError) as error:
            self.window.status.setText(f'Invalid FK report: {error}')
            self.get_logger().warning(f'Invalid /fk/report: {error}')


def main(args=None):
    arguments = sys.argv if args is None else [sys.argv[0], *args]
    rclpy.init(args=arguments)
    app = QtWidgets.QApplication(remove_ros_args(arguments))
    app.setApplicationName('SO100 FK dashboard')
    # Quit through Qt instead of raising KeyboardInterrupt at an arbitrary
    # point in a Qt callback while ROS launch is stopping the application.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    node = None
    timer = None
    try:
        node = DashboardNode()
        window = DashboardWindow(on_selected=node.publish_selected)
        node.window = window
        node.publish_selected(window.selected_joint)
        timer = QtCore.QTimer(window)

        def spin():
            if not rclpy.ok():
                window.close()
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except KeyboardInterrupt:
                window.close()

        timer.timeout.connect(spin)
        timer.start(20)
        window.show()
        return app.exec_()
    finally:
        if timer is not None:
            timer.stop()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
