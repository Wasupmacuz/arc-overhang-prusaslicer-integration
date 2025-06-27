import os
import sys
import json
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtGui import QDoubleValidator, QVector3D
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QMainWindow, QScrollArea, 
    QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout, QPushButton, 
    QFileDialog, QGroupBox, QSlider, QSplitter, QLineEdit,
    QHBoxLayout
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from pyqtgraph.opengl.shaders import ShaderProgram, VertexShader, FragmentShader
import pyqtgraph.opengl as gl
import numpy as np
from numba import njit
from PyQt5.QtGui import QSurfaceFormat
import threading

fmt = QSurfaceFormat()
fmt.setSamples(4)  # Use 4x multisampling; increase for higher quality if needed.
QSurfaceFormat.setDefaultFormat(fmt)

feature_colors = (
    (("D9FFC6"), ("558DFF"), ("E3004C"), ("267300")),
    (("8EFFD9"), ("B31DFF"), ("AB7200"), ("003A27")),
    (("AAFFB7"), ("5A39FF"), ("C82100"), ("00570E")),
    (("72E7FF"), ("FF00D4"), ("778F00"), ("00191E")),
)

HERE = os.path.dirname(os.path.abspath(__file__))
custom = ShaderProgram('diffuseHilight', 
    [   
        VertexShader("""
            #version 140
            uniform mat4 u_mvp;
            uniform mat3 u_normal;
            in vec4 a_position;
            in vec3 a_normal;
            in vec4 a_color;
            out vec4 v_color;
            out vec3 v_normal;

            void main() {
                v_normal = normalize(u_normal * a_normal);
                v_color = a_color;
                gl_Position = u_mvp * a_position;
            }
        """),
        FragmentShader("""
            #version 140
            #ifdef GL_ES
            precision mediump float;
            #endif
            in vec4 v_color;
            in vec3 v_normal;
            out vec4 FragColor;

            void main() {
                // Diffuse lighting
                float contrast = 2.0;
                float p = pow(max(v_normal.z, 0.0), contrast);
                vec4 color = vec4(v_color.rgb * p, v_color.a);
                
                // Highlight effect
                float s = pow(v_normal.x * v_normal.x + v_normal.y * v_normal.y, 2.0);
                color.r += s * (1.0 - color.r);
                color.g += s * (1.0 - color.g);
                color.b += s * (1.0 - color.b);
                
                // Use the per-vertex alpha computed in the mesh.
                FragColor = color;
            }
        """)
    ])

class PyQtGraphCanvas(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setCameraPosition(distance=200, elevation=30, azimuth=-45)
        self.last_mouse_pos = None

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.last_mouse_pos = event.pos()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.RightButton and self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            self.pan_camera(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()
        else:
            super().mouseMoveEvent(event)

    def pan_camera(self, dx, dy):
        """Automatically adjusts pan speed based on zoom level and viewport size."""
        base_pan_speed = 0.00842  # Base sensitivity (adjust empirically)
        fov = self.opts['fov']

        # Scale pan speed by camera distance (zoom level)
        zoom_scale = base_pan_speed * fov

        # Optional: Scale by viewport size (uncomment if needed)
        width, height = self.width(), self.height()
        viewport_scale = np.sqrt(width**2 + height**2) / 1000  # Normalize
        zoom_scale *= viewport_scale

        elevation_rad = np.radians(self.opts['elevation'])
        azimuth_rad = np.radians(self.opts['azimuth'])

        # Compute view direction (same as before)
        view_dir = np.array([
            np.cos(azimuth_rad) * np.cos(elevation_rad),
            np.sin(azimuth_rad) * np.cos(elevation_rad),
            np.sin(elevation_rad)
        ])

        # Compute right/up vectors (same as before)
        world_up = np.array([0, 0, 1])
        right_vector = np.cross(world_up, view_dir)
        right_vector /= np.linalg.norm(right_vector) or 1  # Avoid division by zero
        up_vector = np.cross(view_dir, right_vector)
        up_vector /= np.linalg.norm(up_vector) or 1

        # Apply adaptive pan speed
        shift = zoom_scale * (-dx * right_vector + dy * up_vector)
        self.opts['center'] += QVector3D(*shift)
        self.update()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interactive GCode Preview")
        self.setGeometry(100, 100, 800, 600)
        self.selected_file = None
        self.current_plot = None
        self.polygon_border = None

        # Central widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # File selection at the top
        self.file_selection = QPushButton("Select File")
        self.file_selection.clicked.connect(self.select_file)
        self.layout.addWidget(self.file_selection)

        # Add QCheckBox to control settings visibility
        self.show_settings_checkbox = QCheckBox("Show Settings")
        self.show_settings_checkbox.setChecked(False)
        self.show_settings_checkbox.stateChanged.connect(self.on_checkbox_toggled)
        self.layout.addWidget(self.show_settings_checkbox)

        # Splitter for settings and preview
        self.splitter = QSplitter(Qt.Horizontal)
        
        # Settings menu
        self.settings_menu = SettingsMenu(os.path.join(HERE, 'settings.json'))
        self.settings_menu.setVisible(False)
        self.settings_menu.setMinimumWidth(200)  # Minimum width for settings
        self.settings_menu.setSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, 
                                       QtWidgets.QSizePolicy.Expanding)
        self.splitter.addWidget(self.settings_menu)

        # Preview widget
        self.preview_widget = QWidget()
        self.preview_widget.setMinimumWidth(400)  # Minimum width for preview
        self.preview_widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, 
                                        QtWidgets.QSizePolicy.Expanding)
        self.preview_layout = QHBoxLayout(self.preview_widget)
        self.preview = PyQtGraphCanvas()
        # Add layer slider
        self.layer_slider = SliderWithEdit(value=0, min_value=0, max_value=100, step_size=1,
                                           orientation=Qt.Vertical, line_edit_position='above')
        self.layer_slider.setVisible(False)  # Initially hidden
        self.layer_slider.setFixedWidth(60)
        self.layer_slider.valueChanged.connect(self.on_layer_slider_changed)
        # Warm up numba functions
        threading.Thread(target=self.create_gcode_mesh,
                         args=(
                            ((
                                (0, 0, 0),
                                (0, 1, 0),  # 0°
                                (1, 2, 0),  # 45°
                                (1, 0, 0),  # 135°
                                (2, -1, 0), # -45°
                                (2, 2, 1),  # z change
                                (3, 3, 1),  # 45°
                                (4, 3, 1)   # 0°
                            ), ),
                        )
                ).start()
        self.preview_layout.addWidget(self.preview)
        self.preview_layout.addWidget(self.layer_slider)
        self.splitter.addWidget(self.preview_widget)

        # Set initial splitter sizes
        self.splitter.setSizes([200, 600])  # Initial ratio: 1/3 settings, 2/3 preview
        
        # Set stretch factors to maintain proportions
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 3)

        self.layout.addWidget(self.splitter)

    def on_checkbox_toggled(self, state):
        if state == Qt.Checked:
            self.settings_menu.setVisible(True)
            # Restore reasonable splitter sizes when showing settings
            current_sizes = self.splitter.sizes()
            if sum(current_sizes) == 0:
                current_sizes = [200, 600]
            else:
                total = sum(current_sizes)
                current_sizes = [max(200, int(total * 0.25)), int(total * 0.75)]
            self.splitter.setSizes(current_sizes)
        else:
            # Store current sizes before hiding
            self.last_sizes = self.splitter.sizes()
            self.settings_menu.setVisible(False)
            # Expand preview to full width when hiding settings
            self.splitter.setSizes([0, sum(self.splitter.sizes())])

    def on_layer_slider_changed(self, value):
        if self.current_plot is not None:
            self.update_mesh_alpha(value)


    def select_file(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Select File")
        if file_name:
            self.selected_file = file_name
            self.update_preview()
            self.show_settings_checkbox.setChecked(True)

    def update_preview(self):
        if self.selected_file is None:
            return

        self.line_strings = parse_gcode(self.selected_file)
        if not self.line_strings:
            return

        if self.current_plot:
            self.preview.removeItem(self.current_plot)
            self.current_plot = None
        if self.polygon_border:
            self.preview.removeItem(self.polygon_border)
            self.polygon_border = None

        all_points = []
        for ls in self.line_strings:
            all_points.extend(ls)
        valid_points = np.array([p for p in all_points if not np.isnan(p[0])], dtype=np.float32)
        if len(valid_points) == 0:
            return
        
        # self.current_plot = gl.GLLinePlotItem(pos=valid_points, color = (0.5, 1, 0.5, 0.5))
        self.draw_allowed_space_border(self.settings_menu.data)
        import time
        start = time.perf_counter()
        self.mesh_data, normal_array = self.create_gcode_mesh(self.line_strings)
        print(time.perf_counter() - start)
        
        # Configure layer slider
        self.layer_slider.setRange(0, self.num_layers, step_size=1)
        self.layer_slider.setValue(self.num_layers)  # Start with all layers visible
        self.layer_slider.setVisible(True)

        self.current_plot = gl.GLMeshItem(
            meshdata=self.mesh_data,
            shader='diffuseHilight',
            smooth=True,
            computeNormals=False
        )
        self.current_plot.normals = normal_array
        self.current_plot.setGLOptions('translucent')
        self.on_layer_slider_changed(self.num_layers)
        self.preview.addItem(self.current_plot)

        center = np.mean(valid_points[:, :3], axis=0)
        centroid = QVector3D(center[0], center[1], center[2])
        self.preview.setCameraPosition(
            distance=self.preview.opts['distance'], 
            elevation=30, 
            azimuth=-45,
            pos=centroid
        )

    # Main function that processes segments and uses round_corner if needed.
    def create_gcode_mesh(self, segments, radius=0.44, num_segments=8):
        # Flatten z values to compute color mapping.
        all_z = sorted(set(p[2] for seg in segments for p in seg))
        layer_map = {k: v for v, k in enumerate(all_z)}
        if not all_z:
            return gl.MeshData()
        self.num_layers = len(all_z)
        EPSILON = 1e-6

        cos_vals, sin_vals = precompute_ring_angles(num_segments)
        
        # Lists to accumulate results.
        vertices_list = []
        normals_list  = []
        faces_list    = []
        colors_list   = []
        layer_id_list = []
        base_vert_count = 0

        # Process each segment.
        for seg in segments:
            if len(seg) < 2:
                continue
            current_layer = layer_map[seg[0][2]]
            # Convert segment to float32 numpy array.
            seg_np = np.array(seg, dtype=np.float32)
            # Build a “cleaned” version with rounded corners.
            # For each interior point, insert additional points.
            # (We use the numba round_corner; note that if round_corner returns empty,
            #  we simply skip.)
            clean_points = [seg_np[0]]
            for i in range(1, seg_np.shape[0]-1):
                A = seg_np[i-1]
                B = seg_np[i]
                C = seg_np[i+1]
                corner_pts = round_corner_numba(A, B, C, radius/8, 5, EPSILON)
                if corner_pts.shape[0] > 0:
                    # Append the rounded points.
                    for pt in corner_pts:
                        clean_points.append(pt)
                else:
                    clean_points.append(B)
            clean_points.append(seg_np[-1])
            clean_seg = np.vstack(clean_points)

            # Process the cleaned segment (compiled).
            verts, norms, faces, cols, layer_ids = process_segment_numba(clean_seg, radius, num_segments,
                                                            cos_vals, sin_vals, EPSILON, current_layer, self.num_layers)
            # Adjust face indices to account for previously accumulated vertices.
            faces += base_vert_count
            base_vert_count += verts.shape[0]
            vertices_list.append(verts)
            normals_list.append(norms)
            colors_list.append(cols)
            faces_list.append(faces)
            layer_id_list.append(layer_ids)
        
        if len(vertices_list) == 0:
            return gl.MeshData()
        
        vertices_array = np.vstack(vertices_list)
        normals_array = np.vstack(normals_list)
        colors_array = np.vstack(colors_list)
        faces_array = np.vstack(faces_list).astype(np.uint32)
        self.layer_ids = np.hstack(layer_id_list)

        # Normalize normals vectorized.
        norms_val = np.linalg.norm(normals_array, axis=1, keepdims=True)
        norms_val[norms_val < EPSILON] = 1.0
        normals_array /= norms_val

        mesh_data = gl.MeshData()
        mesh_data.setVertexes(vertices_array)
        # mesh_data.setNormals(normals_array)
        mesh_data.setFaces(faces_array)
        mesh_data.setVertexColors(colors_array)
        return mesh_data, normals_array
    
    def update_mesh_alpha(self, threshold_layer):
        if self.current_plot.colors is None:
            return
            
        # Set alpha to 0.0 where layer > threshold_layer, otherwise 1.0.
        self.current_plot.colors[:, 3] = np.where(self.layer_ids > threshold_layer, 0.0, 1.0)
        self.current_plot.upload_vertex_buffers()
        self.preview.update()

    def draw_allowed_space_border(self, settings_data):
        if not settings_data.get("General Arc Settings", {}).get("CheckForAllowedSpace"):
            return  # Don't need to draw
        
        # Retrieve the list of points from the JSON setting.
        allowed_space = settings_data.get("General Arc Settings", {}).get("AllowedSpaceForArcs", {}).get("value", [])
        if len(allowed_space) < 3:
            return  # Nothing to draw

        # Ensure the polygon is closed.
        if allowed_space[0] != allowed_space[-1]:
            allowed_space.append(allowed_space[0])
        
        # Convert 2D points into 3D coordinates with z=0.
        points_3d = np.array([[x, y, 0.0] for x, y in allowed_space], dtype=np.float32)
        
        # Create a line plot item to draw the polygon border.
        self.polygon_border = gl.GLLinePlotItem(
            pos=points_3d,
            color=(0.6, 0.6, 0.6, 0.6),  # gray border
            width=2,
            # mode='line_strip'
        )
        # Add the polygon border to the preview widget.
        self.preview.addItem(self.polygon_border)

    def rotate_vector(self, v, axis, theta):
        """Rodriguez rotation with safety checks"""
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            return v
        axis = axis / axis_norm
        return (v * np.cos(theta) + 
                np.cross(axis, v) * np.sin(theta) + 
                axis * np.dot(axis, v) * (1 - np.cos(theta)))

class SettingsMenu(QWidget):
    settingsChanged = pyqtSignal(dict)  # Signal to notify settings changes
    def __init__(self, json_file):
        super().__init__()
        self.json_file = json_file
        with open(self.json_file, 'r') as f:
            self.data = json.load(f)
        self.init_ui()

    def init_ui(self):
        # Create a scroll area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        # Create a widget for the scroll area's contents
        self.content_widget = QWidget()
        self.scroll_area.setWidget(self.content_widget)

        # Main layout for the content widget
        self.content_layout = QVBoxLayout()
        self.content_widget.setLayout(self.content_layout)

        # Create widgets for the settings
        self.create_widgets(self.data, self.content_layout)

        # Set the scroll area as part of the main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.scroll_area)

        # Add a button to save settings
        self.save_button = QPushButton("Save Settings")
        self.save_button.clicked.connect(self.save_settings)

        # Add the save button to the main layout
        main_layout.addWidget(self.save_button)

        # Set the main layout for the SettingsMenu
        self.setLayout(main_layout)

    def create_widgets(self, data, layout, parent_data=None, key=None):
        if isinstance(data, dict):
            for k, v in data.items():
                if isinstance(v, dict) and 'value' in v and 'description' in v:
                    setting_value = v['value']
                    setting_description = v['description']
                    
                    # Create a QGroupBox for the setting, using 'name' if available
                    group_box = QGroupBox(v.get('name', k))  # Use 'name' if present, else use k
                    group_box.setToolTip(setting_description)
                    
                    # Create a layout for the group box
                    group_layout = QVBoxLayout()
                    group_box.setLayout(group_layout)
                    
                    # Create widget based on the type of 'value'
                    if isinstance(setting_value, bool):
                        check_box = QCheckBox(k)
                        check_box.setChecked(setting_value)
                        check_box.setToolTip(setting_description)
                        group_layout.addWidget(check_box)
                        check_box.stateChanged.connect(
                            lambda state, d=data, k=k: self.update_bool(d, k, state)
                        )
                    elif isinstance(setting_value, (int, float)):
                        # Retrieve 'min' and 'max' from the data dictionary, with defaults
                        min_val = v.get('min', 0.0)
                        max_val = v.get('max', 10.0)
                        
                        # Ensure min_val is less than or equal to max_val
                        if min_val > max_val:
                            min_val, max_val = max_val, min_val
                        
                        # Clamp setting_value between min_val and max_val
                        setting_value = min(max(setting_value, min_val), max_val)
                        
                        # Determine step_size based on the type of setting_value and 'step_size' in 'v'
                        if isinstance(setting_value, int):
                            step_size = 1
                        else:
                            step_size = v.get('step_size', 0.1)
                        
                        # Create SliderWithEdit with min_value, max_value, and step_size
                        slider = SliderWithEdit(value=setting_value, min_value=min_val, max_value=max_val, step_size=step_size)
                        slider.setToolTip(setting_description)
                        group_layout.addWidget(slider)
                        slider.valueChanged.connect(
                            lambda value, d=data, k=k: self.update_number(d, k, value)
                        )
                    elif isinstance(setting_value, list) and all(isinstance(item, list) for item in setting_value):
                        table = QTableWidget()
                        # Add buttons for row management
                        button_layout = QHBoxLayout()
                        add_button = QPushButton("+")
                        remove_button = QPushButton("-")
                        button_layout.addWidget(add_button)
                        button_layout.addWidget(remove_button)

                        # Connect buttons
                        add_button.clicked.connect(lambda _, t=table: self.add_table_row(t))
                        add_button.clicked.connect(lambda _, d=data, k=k, t=table: self.update_list_of_lists(d, k, t))
                        remove_button.clicked.connect(lambda _, t=table: self.remove_table_row(t))
                        remove_button.clicked.connect(lambda _, d=data, k=k, t=table: self.update_list_of_lists(d, k, t))

                        rows = len(setting_value)
                        cols = len(setting_value[0]) if rows > 0 else 0
                        table.setRowCount(rows)
                        table.setColumnCount(cols)
                        for row in range(rows):
                            for col in range(cols):
                                item = QTableWidgetItem(str(setting_value[row][col]))
                                item.setFlags(item.flags() | Qt.ItemIsEditable)
                                table.setItem(row, col, item)
                        table.setHorizontalHeaderLabels(['X', 'Y'])
                        table.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
                        table.resizeColumnsToContents()
                        table.resizeRowsToContents()
                        table.setToolTip(setting_description)
                        table.setFixedSize(table.sizeHint())
                        group_layout.addWidget(table)
                        group_layout.addLayout(button_layout)
                        table.itemChanged.connect(
                            lambda _, d=data, k=k, t=table: self.update_list_of_lists(d, k, t)
                        )
                    else:
                        lineEdit = QLineEdit(str(setting_value))
                        lineEdit.setToolTip(setting_description)
                        group_layout.addWidget(lineEdit)
                        lineEdit.textChanged.connect(
                            lambda text, d=data, k=k: self.update_string(d, k, text)
                        )
                    
                    # Add the group box to the main layout
                    layout.addWidget(group_box)
                else:
                    subgroup_box = QGroupBox(k)
                    subgroup_layout = QVBoxLayout()
                    subgroup_box.setLayout(subgroup_layout)
                    self.create_widgets(v, subgroup_layout, data, k)
                    layout.addWidget(subgroup_box)
        elif isinstance(data, list):
            # Handle lists that are not nested (if any)
            combo_box = QComboBox()
            for item in data:
                combo_box.addItem(str(item))
            layout.addWidget(combo_box)
            combo_box.currentIndexChanged.connect(
                lambda index, d=data, k=key: self.update_list(d, k, index)
            )
        else:
            # Handle other data types
            lineEdit = QLineEdit(str(data))
            layout.addWidget(lineEdit)

    def add_table_row(self, table):
        current_rows = table.rowCount()
        table.insertRow(current_rows)
        # Initialize new row with default values (0.0)
        for col in range(table.columnCount()):
            item = QTableWidgetItem("0.0")
            table.setItem(current_rows, col, item)

    def remove_table_row(self, table: QTableWidget):
        current_row = table.currentRow()
        if current_row == -1:  # No selection, remove last row
            current_row = table.rowCount() - 1
        if current_row >= 0 and table.rowCount() > 1: # Must have at least one row at all times
            table.removeRow(current_row)

    @pyqtSlot(QTableWidgetItem)
    def update_list_of_lists(self, data, key, table):
        rows = table.rowCount()
        cols = table.columnCount()
        new_list = []
        for row in range(rows):
            row_data = []
            for col in range(cols):
                item = table.item(row, col)
                if item is not None:
                    value = float(item.text())  # Assuming numerical values
                    row_data.append(value)
            new_list.append(row_data)
        # Update the data
        data[key]['value'] = new_list

    @pyqtSlot(int)
    def update_list(self, data, key, index):
        data[key]['value'] = self.data[key][index]

    @pyqtSlot(int)
    def update_bool(self, data, key, state):
        data[key]['value'] = bool(state)

    @pyqtSlot(float)
    def update_number(self, data, key, value):
        data[key]['value'] = value

    def save_settings(self):
        with open(self.json_file, 'w') as f:
            json.dump(self.data, f, indent=4)
        print("Settings saved.")
        self.settingsChanged.emit(self.data.copy())  # Emit the updated settings

class SliderWithEdit(QWidget):
    valueChanged = pyqtSignal(float)

    def __init__(self, value, parent=None, min_value=0.0, max_value=10.0, step_size=0.1,
                 orientation=Qt.Horizontal, line_edit_position='right'):
        super(SliderWithEdit, self).__init__(parent)
        self.min_value = min_value
        self.max_value = max_value
        self.step_size = step_size
        self.orientation = orientation
        self.line_edit_position = line_edit_position

        # Calculate slider range.
        self.slider_min = int(round(self.min_value))
        self.slider_max = int(round((self.max_value - self.min_value) / self.step_size))

        # Create slider widget with the specified orientation.
        self.slider = QSlider(self.orientation)
        self.slider.setMinimum(self.slider_min)
        self.slider.setMaximum(self.slider_max)
        self.slider.setValue(self.float_to_slider(self.min_value + self.step_size))

        # Create the QLineEdit with a validator.
        self.lineEdit = QLineEdit()
        self.lineEdit.setValidator(QDoubleValidator(self.min_value, self.max_value, 10, self))

        # Set up the layout based on orientation and line_edit_position.
        if self.orientation == Qt.Horizontal:
            layout = QHBoxLayout()
            if self.line_edit_position in ('left', 'before'):
                layout.addWidget(self.lineEdit, stretch=0)
                layout.addWidget(self.slider, stretch=1)
            else:  # defaults to right or after
                layout.addWidget(self.slider, stretch=1)
                layout.addWidget(self.lineEdit, stretch=0)
        else:  # Vertical orientation
            layout = QVBoxLayout()
            if self.line_edit_position in ('above', 'top'):
                layout.addWidget(self.lineEdit, stretch=0)
                layout.addWidget(self.slider, stretch=1)
            elif self.line_edit_position in ('below', 'bottom'):
                layout.addWidget(self.slider, stretch=1)
                layout.addWidget(self.lineEdit, stretch=0)
            else:
                # Fallback: slider first then lineEdit.
                layout.addWidget(self.slider, stretch=1)
                layout.addWidget(self.lineEdit, stretch=0)
        self.setLayout(layout)

        # Set initial value.
        self.setValue(value)

        # Connect signals and slots.
        self.slider.valueChanged.connect(self.update_line_edit)
        self.lineEdit.editingFinished.connect(self.update_slider)

    def slider_to_float(self, slider_value):
        return self.min_value + slider_value * self.step_size

    def float_to_slider(self, value):
        value = max(self.min_value, min(self.max_value, value))
        slider_value = int(round((value - self.min_value) / self.step_size))
        slider_value = max(self.slider_min, min(self.slider_max, slider_value))
        return slider_value

    def update_line_edit(self, slider_value):
        value = self.slider_to_float(slider_value)
        # Format value to remove trailing zeros.
        formatted = f"{value:.10f}"
        if '.' in formatted:
            formatted = formatted.rstrip('0').rstrip('.')
        self.lineEdit.setText(formatted)
        self.valueChanged.emit(value)

    def update_slider(self):
        text = self.lineEdit.text()
        if text:
            value = float(text)
            slider_value = self.float_to_slider(value)
            if slider_value != self.value():
                self.slider.setValue(slider_value)
                self.valueChanged.emit(value)

    def setValue(self, value):
        slider_value = self.float_to_slider(value)
        self.slider.setValue(slider_value)
        self.update_line_edit(slider_value)

    def value(self):
        slider_value = self.slider.value()
        return self.slider_to_float(slider_value)

    def setRange(self, min_value, max_value, step_size):
        self.min_value = min_value
        self.max_value = max_value
        self.step_size = step_size
        self.slider_min = 0
        self.slider_max = int(round((self.max_value - self.min_value) / self.step_size))
        self.slider.setMinimum(self.slider_min)
        self.slider.setMaximum(self.slider_max)
        self.setValue(self.min_value + self.step_size)
        self.lineEdit.setValidator(QDoubleValidator(self.min_value, self.max_value, 10, self))


def parse_gcode(file_path):
    # settings = gsettings(graphics='mayavi')
    # gcode = gcody.read(file_path, settings=settings)
    features = set()
    segments = []
    z = 0.0
    new_z = 0.0
    points = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith(';TYPE:'):
                features.add(line)
            line = line.strip().upper().split(';', 1)[0]
            if line.startswith('G'):
                x, y, e = None, None, 0.0
                parts = line.split(" ")
                for part in parts:
                    if part.startswith('X'):
                        x = float(part[1:])
                    if part.startswith('Y'):
                        y = float(part[1:])
                    if part.startswith('Z'):
                        new_z = float(part[1:])
                    if part.startswith('E'):
                        e = float(part[1:])
                if x and y:
                    if e > 0 or len(points) == 0:
                        points.append((x, y, z))
                    else:
                        if len(points) > 1:
                            segments.append(points)
                        points = []
                        if x and y: points.append((x, y, z))
                elif new_z < z:
                    points[-1] = (points[-1][0], points[-1][1], new_z)
                z = new_z
    return segments

# A helper: rotate vector v about axis by angle (Rodrigues formula)
@njit(cache=True)
def rotate_vector_numba(v, axis, angle):
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    # compute cross product manually
    cross = np.empty(3, dtype=np.float32)
    cross[0] = axis[1]*v[2] - axis[2]*v[1]
    cross[1] = axis[2]*v[0] - axis[0]*v[2]
    cross[2] = axis[0]*v[1] - axis[1]*v[0]
    dot_val = v[0]*axis[0] + v[1]*axis[1] + v[2]*axis[2]
    return v * cos_a + cross * sin_a + axis * dot_val * (1 - cos_a)

# Rotate basis: given previous basis (dir, right, up) and a new direction,
# adjust the basis so that its “dir” becomes new_direction.
@njit(cache=True)
def rotate_basis_numba(old_dir, old_right, old_up, new_direction, EPSILON):
    # Compute cross product of old_dir and new_direction.
    rot_axis = np.empty(3, dtype=np.float32)
    rot_axis[0] = old_dir[1]*new_direction[2] - old_dir[2]*new_direction[1]
    rot_axis[1] = old_dir[2]*new_direction[0] - old_dir[0]*new_direction[2]
    rot_axis[2] = old_dir[0]*new_direction[1] - old_dir[1]*new_direction[0]
    rot_norm = np.sqrt(rot_axis[0]*rot_axis[0] +
                       rot_axis[1]*rot_axis[1] +
                       rot_axis[2]*rot_axis[2])
    if rot_norm > EPSILON:
        # Normalize rot_axis.
        rot_axis[0] /= rot_norm
        rot_axis[1] /= rot_norm
        rot_axis[2] /= rot_norm
        # Clamp dot product
        dot_val = old_dir[0]*new_direction[0] + old_dir[1]*new_direction[1] + old_dir[2]*new_direction[2]
        if dot_val > 1.0:
            dot_val = 1.0
        elif dot_val < -1.0:
            dot_val = -1.0
        angle = np.arccos(dot_val)
        new_right = rotate_vector_numba(old_right, rot_axis, angle)
        # Compute new_up as cross(new_direction, new_right)
        new_up = np.empty(3, dtype=np.float32)
        new_up[0] = new_direction[1]*new_right[2] - new_direction[2]*new_right[1]
        new_up[1] = new_direction[2]*new_right[0] - new_direction[0]*new_right[2]
        new_up[2] = new_direction[0]*new_right[1] - new_direction[1]*new_right[0]
        # Force all outputs to be float32.
        return (new_direction.astype(np.float32),
                new_right.astype(np.float32),
                new_up.astype(np.float32))
    else:
        # Return copies cast to float32.
        return (new_direction.astype(np.float32),
                old_right.astype(np.float32),
                old_up.astype(np.float32))

@njit(inline='always', cache=True)
def clip_numba(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x

# A simplified rounding-corner function in Numba.
# (For aggressive optimization we use a simplified version.)
@njit(cache=True)
def round_corner_numba(A, B, C, radius, num_points, EPSILON):
    # Compute BA and BC vectors.
    BA = A - B
    BC = C - B
    ba_norm = np.sqrt(BA[0]*BA[0] + BA[1]*BA[1] + BA[2]*BA[2])
    bc_norm = np.sqrt(BC[0]*BC[0] + BC[1]*BC[1] + BC[2]*BC[2])
    if ba_norm < EPSILON or bc_norm < EPSILON:
        return np.empty((0, 3), dtype=np.float32)
    # Compute cross product of BC and BA.
    cross_3d = np.empty(3, dtype=np.float32)
    cross_3d[0] = BC[1]*BA[2] - BC[2]*BA[1]
    cross_3d[1] = BC[2]*BA[0] - BC[0]*BA[2]
    cross_3d[2] = BC[0]*BA[1] - BC[1]*BA[0]
    cross_norm = np.sqrt(cross_3d[0]*cross_3d[0] +
                         cross_3d[1]*cross_3d[1] +
                         cross_3d[2]*cross_3d[2])
    if cross_norm < EPSILON:
        return np.empty((0, 3), dtype=np.float32)
    normal = cross_3d / cross_norm
    # Define local coordinate system: u along BA, v = cross(normal, u)
    u = BA / ba_norm
    v = np.empty(3, dtype=np.float32)
    v[0] = normal[1]*u[2] - normal[2]*u[1]
    v[1] = normal[2]*u[0] - normal[0]*u[2]
    v[2] = normal[0]*u[1] - normal[1]*u[0]
    v_norm = np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if v_norm < EPSILON:
        return np.empty((0, 3), dtype=np.float32)
    v /= v_norm
    # Project BC direction into local coords.
    w = BC / bc_norm
    w_local_0 = u[0]*w[0] + u[1]*w[1] + u[2]*w[2]
    w_local_1 = v[0]*w[0] + v[1]*w[1] + v[2]*w[2]
    norm_w_local = np.sqrt(w_local_0*w_local_0 + w_local_1*w_local_1)
    if norm_w_local < EPSILON:
        return np.empty((0, 3), dtype=np.float32)
    # Angle at B using our clip function.
    theta = np.arccos(clip_numba(w_local_0/norm_w_local, np.float32(-1.0), np.float32(1.0)))
    if np.abs(np.pi - theta) < EPSILON:
        return np.empty((0, 3), dtype=np.float32)
    L = radius / np.tan(theta/2)
    # Check if L exceeds the lengths of BA or BC
    if L > ba_norm or L > bc_norm:
        return np.empty((0, 3), dtype=np.float32)

    # Tangent points in local coords:
    T1_0 = L
    T1_1 = 0.0
    T2_0 = L * (w_local_0/norm_w_local)
    T2_1 = L * (w_local_1/norm_w_local)
    # Bisector in local coords:
    bis0 = 1.0 + w_local_0/norm_w_local
    bis1 = 0.0 + w_local_1/norm_w_local
    bis_norm = np.sqrt(bis0*bis0 + bis1*bis1)
    bis0 /= bis_norm
    bis1 /= bis_norm
    d = radius / np.sin(theta/2)
    O0 = d * bis0
    O1 = d * bis1
    # Compute start and end angles relative to circle center O.
    start_angle = np.arctan2(T1_1 - O1, T1_0 - O0)
    end_angle   = np.arctan2(T2_1 - O1, T2_0 - O0)
    diff = end_angle - start_angle
    diff = (diff + np.pi) % (2*np.pi) - np.pi
    if diff < 0:
        diff += 2*np.pi
    angles = np.linspace(start_angle, start_angle + diff, num_points)
    out = np.empty((num_points, 3), dtype=np.float32)
    for i in range(num_points):
        x = O0 + radius * np.cos(angles[i])
        y = O1 + radius * np.sin(angles[i])
        # Map back to 3D: B + x*u + y*v.
        out[i, 0] = B[0] + x*u[0] + y*v[0]
        out[i, 1] = B[1] + x*u[1] + y*v[1]
        out[i, 2] = B[2] + x*u[2] + y*v[2]
    return out


# Process a single (cleaned) segment.
# clean_seg is assumed to be an (N,3) float32 array.
@njit(cache=True)
def process_segment_numba(clean_seg, radius, num_segments, cos_vals, sin_vals, EPSILON, layer_num, num_layers):
    N = clean_seg.shape[0]
    # Estimate maximum counts.
    max_verts = N * num_segments
    max_faces = (N - 1) * 2 * num_segments
    vertices = np.empty((max_verts, 3), dtype=np.float32)
    normals  = np.empty((max_verts, 3), dtype=np.float32)
    colors   = np.empty((max_verts, 4), dtype=np.float32)
    faces    = np.empty((max_faces, 3), dtype=np.int32)
    layer_indices = np.empty(max_verts, dtype=np.int32)
    vert_idx = 0
    face_idx = 0

    # Initialize basis using first direction.
    direction = clean_seg[1] - clean_seg[0]
    norm_val = np.sqrt(direction[0]*direction[0] +
                    direction[1]*direction[1] +
                    direction[2]*direction[2])
    if norm_val < EPSILON:
        direction = np.array([1, 0, 0], dtype=np.float32)
    else:
        direction /= norm_val
    # REF_UP = (0,0,1)
    REF_UP = np.array([0, 0, 1], dtype=np.float32)
    right = np.empty(3, dtype=np.float32)
    right[0] = REF_UP[1]*direction[2] - REF_UP[2]*direction[1]
    right[1] = REF_UP[2]*direction[0] - REF_UP[0]*direction[2]
    right[2] = REF_UP[0]*direction[1] - REF_UP[1]*direction[0]
    right_norm = np.sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2])
    if right_norm < EPSILON:
        right = np.array([1, 0, 0], dtype=np.float32)
    else:
        right /= right_norm
    up = np.empty(3, dtype=np.float32)
    up[0] = direction[1]*right[2] - direction[2]*right[1]
    up[1] = direction[2]*right[0] - direction[0]*right[2]
    up[2] = direction[0]*right[1] - direction[1]*right[0]
    basis_dir = direction.copy()
    basis_right = right.copy()
    basis_up = up.copy()
    
    for i in range(N):
        current_point = clean_seg[i]
        # Compute per-vertex direction.
        if i == 0:
            d = clean_seg[1] - current_point
        elif i == N - 1:
            d = current_point - clean_seg[N - 2]
        else:
            d = clean_seg[i+1] - clean_seg[i-1]
        norm_d = np.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        if norm_d < EPSILON:
            d = basis_dir
        else:
            d /= norm_d
        if i > 0:
            basis_dir, basis_right, basis_up = rotate_basis_numba(basis_dir, basis_right, basis_up, d, EPSILON)
        else:
            basis_dir = d

        # Generate ring vertices for current_point.
        for j in range(num_segments):
            idx = vert_idx + j
            # radial vector = basis_right*cos + basis_up*sin
            radial = basis_right * cos_vals[j] + basis_up * sin_vals[j]
            vertices[idx, 0] = current_point[0] + radius * radial[0]
            vertices[idx, 1] = current_point[1] + radius * radial[1]
            vertices[idx, 2] = current_point[2] + radius * radial[2]
            normals[idx, 0] = radial[0]
            normals[idx, 1] = radial[1]
            normals[idx, 2] = radial[2]
            # Compute color from z.
            t = layer_num / num_layers
            colors[idx, 0] = t
            colors[idx, 1] = 0.0
            colors[idx, 2] = 1.0 - t
            colors[idx, 3] = 1.0
            layer_indices[idx] = layer_num

        # Connect with previous ring if i>0.
        if i > 0:
            for j in range(num_segments):
                i0 = vert_idx - num_segments + j
                i1 = vert_idx - num_segments + ((j+1) % num_segments)
                i2 = vert_idx + ((j+1) % num_segments)
                i3 = vert_idx + j
                faces[face_idx, 0] = i0; faces[face_idx, 1] = i1; faces[face_idx, 2] = i2
                face_idx += 1
                faces[face_idx, 0] = i0; faces[face_idx, 1] = i2; faces[face_idx, 2] = i3
                face_idx += 1
        vert_idx += num_segments
    return vertices[:vert_idx], normals[:vert_idx], faces[:face_idx], colors[:vert_idx], layer_indices[:vert_idx]

# Precompute ring angles once.
def precompute_ring_angles(num_segments):
    angles = np.linspace(0, 2*np.pi, num_segments, endpoint=False).astype(np.float32)
    cos_vals = np.cos(angles).astype(np.float32)
    sin_vals = np.sin(angles).astype(np.float32)
    return cos_vals, sin_vals

if __name__ == '__main__':
    app = QApplication(sys.argv)
    with open(os.path.join(HERE, 'style.qss'), "r") as f:
        app.setStyleSheet(f.read())

    window = MainWindow()
    window.show()
    # window.initializeGL()
    sys.exit(app.exec_())