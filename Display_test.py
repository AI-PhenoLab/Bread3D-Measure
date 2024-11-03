import sys
import os
import open3d as o3d
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QDialog, QLineEdit, QPushButton, QFileDialog, QComboBox, \
    QGroupBox, QVBoxLayout, QLabel, QToolTip
from PyQt5.QtGui import QWindow, QPixmap, QImage
from PyQt5.QtCore import QTimer, QCoreApplication
import win32gui
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# import torch
import math
import io
from PIL import Image
import cv2
from shapely.geometry import Polygon, Point
import Ui_display_test_1
import img_test_rc
from qt_material import apply_stylesheet


def polygon_area(vertices):
    """
    Shoelace
    :param vertices:  [(x1, y1), (x2, y2), ..., (xn, yn)]
    :return: area
    """
    n = len(vertices)  # vertice number
    area = 0.0

    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]  # The next point, the last point is connected to the first point
        area += x1 * y2 - y1 * x2

    return abs(area) / 2.0


# Determine which region each point belongs to
def get_region_index(point, x_min, y_min, x_step, y_step, num_divisions_per_axis):
    x, y = point
    x_index = int((x - x_min) // x_step)
    y_index = int((y - y_min) // y_step)
    return x_index + y_index * num_divisions_per_axis


def centroid_calculation(coords):
    # Convert two-dimensional point cloud into Open3 D point cloud object.
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(coords))

    # Calculate the centroid
    centroid = np.mean(np.asarray(pcd.points), axis=0)

    # Extract two-dimensional centroid coordinates
    centroid_2d = centroid[:2]
    return centroid_2d


#  Angle calculation
def angle(p1, p2):
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])


# Computes the rotated bounding box of the set of points
def rotate_points(points, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    return np.dot(points, rotation_matrix)


# Calculate the minimum directed boundary rectangle
def min_area_rect(hull_points):
    min_area = float('inf')
    best_rect = None
    best_angle = 0

    for i in range(len(hull_points)):
        p1 = hull_points[i]
        p2 = hull_points[(i + 1) % len(hull_points)]
        edge_angle = angle(p1, p2)

        rotated_points = rotate_points(hull_points, -edge_angle)

        min_x = np.min(rotated_points[:, 0])
        max_x = np.max(rotated_points[:, 0])
        min_y = np.min(rotated_points[:, 1])
        max_y = np.max(rotated_points[:, 1])

        area = (max_x - min_x) * (max_y - min_y)

        if area < min_area:
            min_area = area
            best_rect = [
                [min_x, min_y],
                [max_x, min_y],
                [max_x, max_y],
                [min_x, max_y]
            ]
            best_angle = edge_angle

    return np.array(best_rect), best_angle


# Statistics for each layer
def find_element_layer_and_count(n, elements):
    # Generate sequences from 1 to n * n.
    grid = np.arange(1, n * n + 1).reshape((n, n))
    num_layers = n // 2
    element_layers = {}
    layer_counts = {i: 0 for i in range(1, num_layers + 1)}

    for element in elements:
        found = False
        for layer in range(num_layers):
            # Get all the elements of the current layer
            layer_elements = []

            # Top row
            layer_elements.extend(grid[layer][layer:n - layer])
            # Right column
            layer_elements.extend(grid[i][n - layer - 1] for i in range(layer + 1, n - layer - 1))
            # Bottom row
            layer_elements.extend(grid[n - layer - 1][n - layer - 1:layer - 1:-1])
            # Left column
            layer_elements.extend(grid[i][layer] for i in range(n - layer - 2, layer, -1))

            if element in layer_elements:
                element_layers[element] = layer + 1
                layer_counts[layer + 1] += 1
                found = True
                break

        if not found:
            element_layers[element] = None

    return element_layers, layer_counts


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_display_test_1.Ui_MainWindow()
        self.ui.setupUi(self)
        self.showMaximized()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width=1500, height=1500, visible=False)  # visible=False 窗口不显示，避免启动时一闪而过
        self.winid = win32gui.FindWindow('GLFW30', None)
        self.sub_window = QWindow.fromWinId(self.winid)
        self.displayer = QWidget.createWindowContainer(self.sub_window)
        self.ui.gridLayout.addWidget(self.displayer)
        self.clock = QTimer(self)
        self.clock.timeout.connect(self.draw_update)
        self.clock.start(20)

        # global object
        # point cloud object
        self.point_cloud = o3d.geometry.PointCloud()
        # temp object
        self.tmp_cloud = o3d.geometry.PointCloud()
        # temp pore object
        self.hole_cloud = o3d.geometry.PointCloud()
        # temp pre-processing object
        self.pre_cloud = o3d.geometry.PointCloud()
        # Whether to enter the pore extraction step
        self.is_hole_detection = 0
        # Whether to enter the parameter extraction step
        self.is_para_extraction = 0
        # Line set
        self.line_set = o3d.geometry.LineSet()
        # Number of grid arrays
        self.num_divisions = 10
        # Whether to show
        # self.is_show = 0
        # Height
        self.height = 0
        # Length
        self.length = 0
        # Width
        self.width = 0
        # Surface area
        self.surface_area = 0
        # Volume
        self.volume = 0
        # Symmetry index
        self.symmetry_index = 0
        # Homogeneity index
        self.homogeneity_index = 0
        # Maximum pore diameter
        self.max_pore_diameter = 0
        # Maximum pore area
        self.max_pore_area = 0
        # Pore number
        self.hole_number = 0
        # Maximum pore depth
        self.max_pore_depth = 0
        # Maximum pore volume
        self.max_pore_volume = 0
        # Pore elongation
        self.elongation = 0
        # Pore distribution
        self.pore_distribution = ""
        # Pore layer distribution
        self.pore_layer_distribution = ""

        # Information set
        self.info_set = []

        # HintButton
        self.HintButton_1 = self.findChild(QPushButton, 'pushButton_4')
        self.HintButton_1.setToolTip('<b><span style="font-size: 22px;"> Select a point cloud file to read, support.ply.pcd format.</span></b>')
        self.HintButton_2 = self.findChild(QPushButton, 'pushButton_5')
        self.HintButton_2.setToolTip('<b><span style="font-size: 22px;"> Remove the background object.</span></b>')
        self.HintButton_3 = self.findChild(QPushButton, 'pushButton_6')
        self.HintButton_3.setToolTip('<b><span style="font-size: 22px;"> Segmentation module, with four scales to select.</span></b>')
        self.HintButton_4 = self.findChild(QPushButton, 'pushButton_8')
        self.HintButton_4.setToolTip('<b><span style="font-size: 22px;"> Pore extraction module, with five scales to select.</span></b>')
        self.HintButton_5 = self.findChild(QPushButton, 'pushButton_10')
        self.HintButton_5.setToolTip('<b><span style="font-size: 22px;"> Chart plotting module, with four charts to select.</span></b>')
        self.HintButton_6 = self.findChild(QPushButton, 'pushButton_12')
        self.HintButton_6.setToolTip('<b><span style="font-size: 22px;"> Save all parameters to the specified folder, save as .xlsx format.</span></b>')

        # -------------1.Load point cloud-------------------------
        # QLineEdit instance
        self.FileName = self.findChild(QLineEdit, 'lineEdit')
        self.FileName.setPlaceholderText("File path")

        # Open the point cloud PushButton instance
        self.openButton = self.findChild(QPushButton, 'pushButton')
        # self.pushButton.clicked.connect(self.draw_test)
        self.openButton.clicked.connect(self.show_Dialog)

        # ---------------------------------------------------------

        # --------------2. Preprocessing----------------------------
        # Preprocessing instance
        self.preprocessing_Button = self.findChild(QPushButton, 'pushButton_3')
        self.preprocessing_Button.clicked.connect(self.preprocessing_Dialog)

        # ---------------------------------------------------------

        # ----------------3. Plane detection-----------------------
        self.comboBox = self.findChild(QComboBox, 'comboBox')
        self.comboBox.addItems(['select', '0.5mm', '1mm', '2mm', '5mm'])
        self.comboBox.model().item(0).setEnabled(False)
        # QtWidgets.QComboBox.showPopup(self.comboBox)

        self.plane_detection_Button = self.findChild(QPushButton, 'pushButton_7')
        self.plane_detection_Button.clicked.connect(self.plane_detection_Dialog)

        # ---------------------------------------------------------

        # ---------------4. Pore detection-------------------------
        self.comboBox3 = self.findChild(QComboBox, 'comboBox_3')
        self.comboBox3.addItems(['select', '4 x 4', '6 x 6', '8 x 8', '10 x 10', '12 x 12'])
        self.comboBox3.model().item(0).setEnabled(False)
        self.hole_detection_Button = self.findChild(QPushButton, 'pushButton_9')
        self.hole_detection_Button.clicked.connect(self.hole_detection_Dialog)

        # ________________________________________________________

        # ---------------5. Parameter extraction------------------
        self.comboBox2 = self.findChild(QComboBox, 'comboBox_2')
        self.comboBox2.addItems(['select', 'Area bar', 'Area pie', 'Distribution bar', 'Distribution pie'])
        self.comboBox2.model().item(0).setEnabled(False)
        # QtWidgets.QComboBox.showPopup(self.comboBox2)

        # self.group_box = self.findChild(QGroupBox, 'groupBox')
        self.imageLabel = self.findChild(QLabel, 'label')
        self.chart_Button = self.findChild(QPushButton, 'pushButton_11')
        self.chart_Button.clicked.connect(self.draw_chart_Dialog)
        # --------------------------------------------------------

        # --------------6. Save point cloud-----------------------
        # Save instance
        # self.saveButton = self.findChild(QPushButton, 'pushButton_1')
        # self.saveButton.clicked.connect(self.save_Dialog)

        # Save as instance
        self.saveasButton = self.findChild(QPushButton, 'pushButton_2')
        self.saveasButton.clicked.connect(self.saveas_Dialog)

        # ---------------------------------------------------------

        # self.draw_test() # Visualization
        # Ensure maximization when the window opens
        # self.showMaximized()

        # QSS
        # self.setStyleSheet("""
        # QPushButton:hover {
        #     background-color: white;
        #     color: white;
        # }
        # QComboBox::down-arrow {
        #     image: url(E:/PycharmProjects/point-cloud-viwer-master_3/arrow.jpg);
        #     width: 20px;
        #     height: 100px;
        # }
        # """)

    # coordinate system calculation
    def coordinate_calculation(self, point_cloud):
        points = np.asarray(point_cloud.points)
        x_range = np.max(points[:, 0]) - np.min(points[:, 0])
        y_range = np.max(points[:, 1]) - np.min(points[:, 1])
        z_range = np.max(points[:, 2]) - np.min(points[:, 2])
        xyz_size = max(x_range, y_range, z_range)
        size = xyz_size / 10
        return size

    # Visualization instance, visualized according to the path
    def draw_test(self):
        print("File name is ", self.FileName)
        pcd = o3d.io.read_point_cloud(self.FileName.text())  # point cloud path
        print("point cloud is ", pcd.points)
        self.is_hole_detection = 0
        # self.point_cloud.points = o3d.utility.Vector3dVector()
        # self.point_cloud.colors = o3d.utility.Vector3dVector()
        # self.tmp_cloud.points = o3d.utility.Vector3dVector()
        # self.tmp_cloud.colors = o3d.utility.Vector3dVector()
        # self.hole_cloud.points = o3d.utility.Vector3dVector()
        # self.hole_cloud.colors = o3d.utility.Vector3dVector()
        # self.pre_cloud.points = o3d.utility.Vector3dVector()
        # self.pre_cloud.colors = o3d.utility.Vector3dVector()
        self.point_cloud = pcd
        self.vis.clear_geometries()
        self.vis.get_render_option().point_size = 2  # Set the size of the rendering point
        box = pcd.get_axis_aligned_bounding_box()
        box.color = (0, 0, 0)
        self.vis.add_geometry(box)  # Add box
        # self.vis.update_geometry(box)
        mesh_frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=self.coordinate_calculation(pcd),
                                                                         origin=[0, 0, 0])
        self.vis.add_geometry(mesh_frame)  # Add coordinate axis
        # self.vis.update_geometry(mesh_frame)
        self.vis.add_geometry(pcd)
        self.vis.update_geometry(pcd)

    # Visual instance, visualize according to point cloud changes
    def update_test(self, pcd, line_set):
        self.vis.clear_geometries()
        self.vis.get_render_option().point_size = 2  # Set the size of the rendering point
        box = pcd.get_axis_aligned_bounding_box()
        box.color = (0, 0, 0)
        self.vis.add_geometry(box)  # Add box
        # self.vis.update_geometry(box)
        mesh_frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=self.coordinate_calculation(pcd),
                                                                         origin=[0, 0, 0])
        self.vis.add_geometry(mesh_frame)  # Add coordinate axis
        self.vis.add_geometry(pcd)
        self.vis.add_geometry(line_set)
        self.vis.update_geometry(pcd)

        self.tmp_cloud = pcd

    # Define the function to open the folder directory push_button
    def show_Dialog(self):
        # fname = QFileDialog.getOpenFileName(self, 'Open file', '.')
        fname = QFileDialog.getOpenFileName(None, "Open File", "./", "All Files(*);;Pcd(*.pcd);;Ply(*.ply);;Mat(*.mat)")
        print("fname data is ", fname[0])
        if fname[0]:
            self.FileName.setText(fname[0])
            self.draw_test()
            self.is_show = 0
            # f = open(fname[0], 'r')
            # with f:
            #     data = f.read()
            #     self.FileName.setText(data)

    def save_Dialog(self):
        if len(self.point_cloud.points) == 0:
            message_box = QtWidgets.QMessageBox()
            message_box.setWindowTitle("Warning")
            message_box.setText("The point cloud is empty and cannot be saved.")
            message_box.setIcon(QtWidgets.QMessageBox.Information)
            message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
            message_box.exec_()
            return
        options = QtWidgets.QFileDialog.Options()
        if self.FileName:  # Check file name is empty
            reply = QtWidgets.QMessageBox.question(self,
                                                   'Tips',
                                                   f'Do you want to save the current point cloud in{self.FileName.text()}?',
                                                   QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                                   QtWidgets.QMessageBox.No)
            if reply == QtWidgets.QMessageBox.Yes:
                file_name = self.FileName.text()
                print('file_name is ', file_name)
                # Save the point cloud
                o3d.io.write_point_cloud(file_name, self.point_cloud)
                # Create an information prompt box
                message_box = QtWidgets.QMessageBox()
                message_box.setWindowTitle("Tips")
                message_box.setText(f'Point cloud saved to {file_name}.')
                message_box.setIcon(QtWidgets.QMessageBox.Information)
                message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
                message_box.exec_()

                # event.accept()
            # else:
            #     event.ignore()

    def saveas_Dialog(self):
        print("Height is ", self.height)
        print("Length is ", self.length)
        print("Width is ", self.width)
        print("Surface area is ", self.surface_area)
        print("Volume is ", self.volume)
        print("Symmetry Index is ", self.symmetry_index)
        print("Homogeneity Index is ", self.homogeneity_index)
        print("Maximum Pore Diameter is ", self.max_pore_diameter)
        print("Maximum Pore Area is ", self.max_pore_area)
        print("Pore Number is ", self.hole_number)
        print("Maximum Pore Depth is ", self.max_pore_depth)
        print("Maximum Pore Volume is ", self.max_pore_volume)
        print("Elongation is ", self.elongation)
        print("Pore Distribution is ", self.pore_distribution)
        print("Pore Layer Distribution is ", self.pore_layer_distribution)
        data = {
            'Parameter Names': ['Height', 'Length', 'Width', 'Surface area', 'Volume',
                      'Symmetry Index', 'Homogeneity Index', 'Maximum Pore Diameter', 'Maximum Pore Area', 'Pore Number',
                      'Maximum Pore Depth', 'Maximum Pore Depth', 'Elongation', 'Pore Distribution', 'Pore Layer Distribution'],
            'Predict Values': [self.height, self.length, self.width, self.surface_area, self.volume, self.symmetry_index, self.homogeneity_index, self.max_pore_diameter, self.max_pore_area, self.hole_number,
                       self.max_pore_depth, self.max_pore_volume, self.elongation, [self.pore_distribution], [self.pore_layer_distribution]]
        }

        # point_cloud = self.point_cloud
        if self.height == 0 or self.elongation == 0:
            message_box = QtWidgets.QMessageBox()
            message_box.setWindowTitle("Hint")
            message_box.setText("The parameters are not calculated and cannot be saved")
            message_box.setIcon(QtWidgets.QMessageBox.Information)
            message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
            message_box.exec_()
            return
        # Save as dialog box
        options = QtWidgets.QFileDialog.Options()
        file_name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            'Save Files',
            '',
            'Excel Files (*.xls);;All Files (*)',
            options=options
        )
        if file_name:  # Check file name is empty
            print('file_name is ', file_name)
            # o3d.io.write_point_cloud(file_name, point_cloud)
            df = pd.DataFrame(data)
            df.to_csv(file_name, index=False)
            message_box = QtWidgets.QMessageBox()
            message_box.setWindowTitle("Hint")
            message_box.setText(f'File has been saved in {file_name}')
            message_box.setIcon(QtWidgets.QMessageBox.Information)
            message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)

            message_box.exec_()
            print(f'File has been saved in {file_name}')

    # Pre-processing instance push_button 3 ( direct filtering in the depth direction )
    def preprocessing_Dialog(self):
        if len(self.point_cloud.points) == 0:
            message_box = QtWidgets.QMessageBox()
            message_box.setWindowTitle("Tips")
            message_box.setText("The point cloud is empty and cannot be preprocessed")
            message_box.setIcon(QtWidgets.QMessageBox.Information)
            message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
            message_box.exec_()
            return
        pcd = self.point_cloud
        # Extract the z-axis value of the point cloud
        points = np.asarray(pcd.points)
        z_values = points[:, 2]

        # Calculate the maximum value of the z-axis
        z_max = np.max(z_values)

        # Filtered by Boolean indexes
        indices = np.where((points[:, 2] >= (z_max - 14)) & (points[:, 2] <= z_max))[0]
        plane_indices = np.where((points[:, 2] >= (z_max - 22)) & (points[:, 2] <= (z_max - 18)))[0]
        plane_cloud = pcd.select_by_index(plane_indices)
        z_values = np.asarray(plane_cloud.points)[:, 2]
        plane_mean = np.mean(z_values)

        # Sort the point clouds by the y-axis
        points_sorted = points[points[:, 1].argsort()]

        # Calculate the index range of each partition
        total_points = points.shape[0]
        indices_2 = int(total_points * 0.2)
        indices_3_1 = int(total_points * 0.3)
        indices_3_2 = int(total_points * 0.3)
        indices_2_2 = total_points - indices_2 - indices_3_1 - indices_3_2

        # cloud segmentation
        split_points_1 = points_sorted[:indices_2]
        split_points_2 = points_sorted[indices_2:indices_2 + indices_3_1]
        split_points_3 = points_sorted[indices_2 + indices_3_1:indices_2 + indices_3_1 + indices_3_2]
        split_points_4 = points_sorted[indices_2 + indices_3_1 + indices_3_2:]

        hb = np.mean(np.asarray(split_points_1[:, 2]))
        hc = (np.mean(np.asarray(split_points_2[:, 2])) + np.mean(np.asarray(split_points_3[:, 2]))) / 2
        hd = np.mean(np.asarray(split_points_4[:, 2]))

        self.height = round(abs((hb + hc + hd - plane_mean * 3) / 30), 2)

        self.symmetry_index = round(abs(2 * hc - hb - hd), 2)

        self.homogeneity_index = round(abs(hb - hd), 2)

        # Create a new point cloud object that contains only filtered points
        cropped_pcd = pcd.select_by_index(indices)
        self.pre_cloud = cropped_pcd
        line_set = o3d.geometry.LineSet()
        self.point_cloud = cropped_pcd
        self.update_test(cropped_pcd, line_set)

    def plane_detection_Dialog(self):
        if self.comboBox.currentText() == "0.5mm":
            threshold = 0.05
        if self.comboBox.currentText() == "1mm":
            threshold = 0.1
        if self.comboBox.currentText() == "2mm":
            threshold = 0.2
        if self.comboBox.currentText() == "5mm":
            threshold = 0.5
        print("Threshold is ", threshold)
        print("Calculating...")

        # Here with the plane test results (the best results), it is also support to call the.pth verification accuracy
        # to test .pth
        # MODEL = importlib.import_module('pointnet2_part_seg_msg')
        # classifier = MODEL.get_model(2, normal_channel=True).cuda()
        # checkpoint = torch.load('best_model.pth')
        # classifier.load_state_dict(checkpoint['model_state_dict'], strict=False)
        # tensor_data = torch.FloatTensor(data).view(1, -1, 6)
        # tensor_data = torch.cat((tensor_data, tensor_data[:, :, :3]), dim=-1)
        # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # tensor_data = tensor_data.to(device)
        # tensor_data = tensor_data.permute(0, 2, 1)
        # classifier = classifier.eval()
        # seg_pred, _ = classifier(tensor_data)

        pcd = self.point_cloud
        # Convert the point cloud into an NumPy array
        points = np.asarray(pcd.points)

        # Calculate the maximum and minimum values in the x direction
        x_min = np.min(points[:, 0])
        x_max = np.max(points[:, 0])

        # Calculate the maximum and minimum values in the y direction
        y_min = np.min(points[:, 1])
        y_max = np.max(points[:, 1])

        self.length = round(abs(x_max - x_min), 2)
        self.width = round(abs(y_max - y_min), 2)

        # 输出结果
        print(f"Minimum value in the x direction is: {x_min}")
        print(f"Maximum value in the x direction is: {x_max}")
        print(f"Minimum value in the y direction is: {y_min}")
        print(f"Maximum value in the y direction is: {y_max}")

        x_slice = (x_max - x_min) / 10
        y_slice = (y_max - y_min) / 10

        # Generate 100 colors and divide the segmentation area.
        colors = np.random.rand(100, 3)
        plane_cloud = o3d.geometry.PointCloud()

        cnt = 0
        for i in range(10):
            for j in range(10):
                # Filters out points within a specified range of x and y using a Boolean index
                indices = np.where((points[:, 0] >= x_min + i * x_slice) & (points[:, 0] <= x_min + (i + 1) * x_slice) &
                                   (points[:, 1] >= y_min + j * y_slice) & (points[:, 1] <= y_min + (j + 1) * y_slice))[
                    0]
                # print("indices is ", indices)
                if (len(indices) > 100):
                    tmp_cloud = pcd.select_by_index(indices)
                    tmp_color = np.array(
                        [[colors[cnt][0], colors[cnt][1], colors[cnt][2]] for i in range(len(indices))])
                    tmp_cloud.colors = o3d.utility.Vector3dVector(tmp_color)

                    # Calculate the local fitting plane and merge the plane grids
                    # Fit the plane using the RANSAC algorithm
                    plane_model, inliers = tmp_cloud.segment_plane(distance_threshold=threshold,
                                                                   ransac_n=3,
                                                                   num_iterations=1000)

                    # Extract the points and outer points on the fitting plane
                    inlier_cloud = tmp_cloud.select_by_index(inliers)

                    # Merge the points in two planes
                    plane_cloud.points = o3d.utility.Vector3dVector(
                        np.vstack((np.asarray(plane_cloud.points), np.asarray(inlier_cloud.points))))
                    plane_cloud.colors = o3d.utility.Vector3dVector(
                        np.vstack((np.asarray(plane_cloud.colors), np.asarray(inlier_cloud.colors))))

                    cnt += 1

        print("Calculation complete.")
        # self.point_cloud = plane_cloud
        line_set = o3d.geometry.LineSet()
        self.update_test(plane_cloud, line_set)

    def hole_detection_Dialog(self):
        hole_num_info = self.comboBox3.currentText()
        if hole_num_info == "4 x 4":
            self.num_divisions = 4
        if hole_num_info == "6 x 6":
            self.num_divisions = 6
        if hole_num_info == "8 x 8":
            self.num_divisions = 8
        if hole_num_info == "10 x 10":
            self.num_divisions = 10
        if hole_num_info == "12 x 12":
            self.num_divisions = 12

        if self.is_hole_detection == 0:
            self.hole_cloud = self.tmp_cloud
            self.is_hole_detection = 1

        pcd = self.hole_cloud
        # The average value of Z axis
        points = np.asarray(pcd.points)
        z_mean = np.mean(points[:, 2])

        # Calculate the maximum and minimum values of the X-axis and their differences.
        x_max = np.max(points[:, 0])
        x_min = np.min(points[:, 0])

        # Calculate the maximum and minimum values of the Y-axis and their differences.
        y_max = np.max(points[:, 1])
        y_min = np.min(points[:, 1])

        # Visual two-dimensional projection image
        plt.figure(figsize=(6, 6))
        # Set the range of the coordinate axis
        plt.xlim(0, 200)
        plt.ylim(0, 200)

        # plt.title("2D Point Cloud")
        plt.scatter(points[:, 0], points[:, 1], s=1)
        # Get the current graph and coordinate axis
        # fig = plt.gcf()
        ax = plt.gca()

        # The conversion relationship between data coordinates and pixel coordinates is obtained.
        trans = ax.transData.transform
        inv_trans = ax.transData.inverted().transform

        plt.axis('off')
        # Save the drawing to the buffer in memory
        buf = io.BytesIO()
        plt.savefig(buf, format='png')

        plt.axis('on')
        # plt.show()

        buf.seek(0)
        # Read the image in the buffer and convert it to a 2D image
        img = Image.open(buf)
        image_array = np.array(img)

        # A set of edge extraction process
        gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter the peripheral pores and calculate the area and number of internal pores.
        hole_areas = []
        holes_contours = []
        contour_set = []
        tmp = []
        for i, contour in enumerate(contours):
            # Ignore peripheral pores
            if hierarchy[0][i][3] != -1:
                area = cv2.contourArea(contour)
                if area > 0:
                    hole_areas.append(area)
                    holes_contours.append(contour)
                    for j, row in enumerate(contour):
                        # print(row[0][0], " ", 600 - row[0][1], end=' ')
                        data_coords = inv_trans([row[0][0], 600 - row[0][1]])
                        contour_set.append(data_coords)
                        tmp.append(data_coords)
                    tmp.append([-1, -1])  # classified point
            # Is the peripheral pore, then calculate the surface area
            else:
                self.surface_area = round(cv2.contourArea(contour), 2)
        self.volume = round(self.surface_area * self.height, 2)
        points_2d = np.array(contour_set)
        z_coords = np.empty((1, points_2d.shape[0]))
        for i in range(len(z_coords)):
            z_coords[i] = z_mean

        # Extending 2D points to 3D points
        points_3d = np.hstack((points_2d, z_coords.reshape(-1, 1)))

        # Create an empty point cloud object and save the boundary point cloud
        boundary_cloud = o3d.geometry.PointCloud()
        boundary_cloud.points = o3d.utility.Vector3dVector(points_3d)

        # Generate color, mark the pores
        color_species = np.random.rand(len(hole_areas), 3)

        colors = np.zeros((points_2d.shape[0], 3))
        species_serial_num = 0
        colors_serial_num = 0
        for i, elem in enumerate(tmp):
            if elem[0] != -1:
                colors[colors_serial_num] = color_species[species_serial_num]
                colors_serial_num += 1
            else:
                species_serial_num += 1

        # Set the color of each point as their corresponding color.
        red_color = np.array(
            [[colors[i][0], colors[i][1], colors[i][2]] for i in range(colors.shape[0])])
        boundary_cloud.colors = o3d.utility.Vector3dVector(red_color)

        hole_number = len(hole_areas)
        self.hole_number = hole_number
        # print("hole number is ", hole_number)

        max_area = 0
        max_area_series = 0

        # Determine the location of the maximum pore
        for i, area in enumerate(hole_areas):
            if area > max_area:
                max_area = area
                max_area_series = i
            # print(f'Pore {i + 1} area: {area}')

        x_step = (x_max - x_min) / self.num_divisions
        y_step = (y_max - y_min) / self.num_divisions

        # Create dividing line
        seg_lines = []
        for i in range(self.num_divisions + 1):
            # perpendicular line
            seg_lines.append([i * x_step + x_min, y_min, z_mean])
            seg_lines.append([i * x_step + x_min, y_max, z_mean])
            # horizontal line
            seg_lines.append([x_min, i * y_step + y_min, z_mean])
            seg_lines.append([x_max, i * y_step + y_min, z_mean])

        lines = []
        cnt = 0
        # Record the beginning and end of the sequence where the maximum area is located.
        max_area_color_start = 0
        max_area_color_end = 0
        hole_area_convert = []
        centroid_pos_set = []
        centroid_area_num = []
        for i in range(len(holes_contours)):
            if i == max_area_series:
                max_area_color_start = cnt
                max_area_color_end = cnt + len(holes_contours[i])
            for j in range(len(holes_contours[i])):
                if j == len(holes_contours[i]) - 1:
                    lines.append([cnt + j, cnt])
                else:
                    lines.append([cnt + j, cnt + j + 1])
            tmp_vertices = []
            for num in range(cnt, cnt + len(holes_contours[i])):
                tmp_vertices.append(boundary_cloud.points[lines[num][0]])
            centroid_pos_set.append(centroid_calculation(tmp_vertices))
            centroid_area_num.append(
                get_region_index(centroid_calculation(tmp_vertices), x_min, y_min, x_step, y_step, self.num_divisions))
            hole_area_convert.append(polygon_area([[row[0], row[1]] for row in tmp_vertices]).round(2))
            cnt = cnt + len(holes_contours[i])

        # The location and area information are spliced.
        hole_area_convert = np.asarray(hole_area_convert)
        centroid_area_num = np.asarray(centroid_area_num)
        centroid_pos_set = np.hstack(
            (centroid_pos_set, hole_area_convert.reshape(-1, 1), centroid_area_num.reshape(-1, 1)))
        pore_distribution_arr = centroid_area_num.reshape(-1, 1)

        # self.pore_distribution = ' '.join([f'{str(dis)}' for dis in pore_distribution_arr])
        self.pore_distribution = ' '.join(str(dis) for dis in pore_distribution_arr)
        self.info_set = centroid_pos_set.tolist()
        # print("info set is ", self.info_set)

        # Maximum pore point set
        vertices = []
        for i in range(max_area_color_start, max_area_color_end):
            vertices.append(boundary_cloud.points[lines[i][0]])

        # Calculate the bounding box
        # Calculate the minimum directed boundary rectangle
        # Extract the first two columns
        vertices_2d = [[row[0], row[1]] for row in vertices]
        box_points = np.asarray(vertices_2d)
        best_rect, best_angle = min_area_rect(box_points)
        # Rotate back to the original angle
        best_rect = rotate_points(best_rect, best_angle)

        hole_area = round(polygon_area(vertices_2d) / 100, 2)
        self.max_pore_area = hole_area
        # print("max hole area is ", hole_area, "cm2")

        # Draw a bounding box
        box_line_set = o3d.geometry.LineSet()
        box_z_coords = np.full((1, 4), z_mean)

        # Extending 2D points to 3D points
        box_points_3d = np.hstack((np.asarray(best_rect), box_z_coords.reshape(-1, 1)))
        box_line_set.points = o3d.utility.Vector3dVector(box_points_3d)
        dis1 = math.sqrt((box_line_set.points[0][0] - box_line_set.points[1][0]) ** 2 + (
                box_line_set.points[0][0] - box_line_set.points[1][0]) ** 2)
        dis2 = math.sqrt((box_line_set.points[0][0] - box_line_set.points[3][0]) ** 2 + (
                box_line_set.points[0][0] - box_line_set.points[3][0]) ** 2)

        hole_width = round(max(dis1, dis2) / 10, 2)
        self.max_pore_diameter = hole_width
        # print("max pore_width is ",hole_width, "cm")

        vertices_3d = np.asarray(vertices)
        # Create Shapely polygons
        polygon = Polygon(vertices_3d)
        # Generate all points in the range of point cloud
        x_box_min, y_box_min, x_box_max, y_box_max = polygon.bounds
        x_range = np.arange(x_box_min, x_box_max, 0.5)
        y_range = np.arange(y_box_min, y_box_max, 0.5)
        xx, yy = np.meshgrid(x_range, y_range)
        grid_points = np.c_[xx.ravel(), yy.ravel()]

        # Filter out the points inside the polygon
        internal_points = np.array([point for point in grid_points if polygon.contains(Point(point))])

        internal_cloud = o3d.geometry.PointCloud()
        internal_z_coords = np.full((1, len(internal_points)), z_mean)
        internal_points = np.hstack((np.asarray(internal_points), internal_z_coords.reshape(-1, 1)))
        internal_cloud.points = o3d.utility.Vector3dVector(internal_points)
        internal_color = [[1, 0, 0] for _ in range(len(internal_points))]
        internal_cloud.colors = o3d.utility.Vector3dVector(internal_color)

        max_hole_centroid = np.mean(internal_cloud.points, axis=0)
        crop_points = np.asarray(self.pre_cloud.points)
        # Calculate the distance and filter the points
        # distances = np.linalg.norm(crop_points - max_hole_centroid, axis=1)
        # indices_within_radius = np.where(distances <= 1)[0]
        kdtree = o3d.geometry.KDTreeFlann(self.pre_cloud)
        [k, idx, _] = kdtree.search_radius_vector_3d(max_hole_centroid, 10)
        # indices_within_radius = self.pre_cloud.radius_search(max_hole_centroid, 1)

        # Get the point cloud within the specified radius range
        filtered_points = crop_points[idx]
        self.max_pore_depth = round(filtered_points[:, 2].max() - filtered_points[:, 2].min(), 2)
        avg_pore = (filtered_points[:, 2].max() + filtered_points[:, 2].min()) / 2

        points_above = filtered_points[filtered_points[:, 2] > avg_pore]
        points_below = filtered_points[filtered_points[:, 2] < avg_pore]

        interpolated_points = []
        for point in points_below:
            # Calculate the difference between x and y values
            diffs = points_above[:, :2] - point[:2]
            # Calculate the distance in the z direction of each point
            z_distances = np.abs(points_above[:, 2] - point[2])

            # Calculate the comprehensive distance(here we can only consider the distance in the z direction).
            distances = z_distances

            # Find the nearest point
            nearest_index = np.argmin(distances)
            point2 = points_above[nearest_index]
            z_diff = point2[2] - point[2]
            num_steps = int(np.ceil(abs(z_diff) / 0.2))

            # Generate interpolation points
            z_values = np.linspace(point[2], point2[2], num_steps + 1)

            for z in z_values:
                # The value of interpolation x and y
                ratio = (z - point[2]) / z_diff if z_diff != 0 else 0
                x = point[0] + ratio * (point2[0] - point[0])
                y = point[1] + ratio * (point2[1] - point[1])
                interpolated_points.append([x, y, z])

        # Create new point cloud objects
        merge_points = np.vstack([points_above, points_below, np.array(interpolated_points)])
        merge_max_z = np.max(merge_points[:, 2])
        merge_points = merge_points[merge_points[:, 2] <= (merge_max_z - 2)]
        merge_cloud = o3d.geometry.PointCloud()
        merge_cloud.points = o3d.utility.Vector3dVector(merge_points)
        hull, _ = merge_cloud.compute_convex_hull()
        # hull, triangles = o3d.geometry.TriangleMesh.create_from_point_cloud(merge_cloud)
        # hull = o3d.geometry.ConvexHull()
        # hull.create_from_points(merge_points)
        # hull.create_from_points(filtered_points)
        self.max_pore_volume = round(hull.get_volume() / 1000, 2)

        # Centralized point cloud
        centered_points = merge_points - np.mean(merge_points, axis=0)
        # Calculate the covariance matrix
        cov_matrix = np.cov(centered_points, rowvar=False)
        # Calculating Eigenvalues and Eigenvectors
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

        # Extract the maximum and minimum eigenvalues
        self.elongation = round(np.max(eigenvalues) / np.min(eigenvalues), 2)

        # Each pore centroid display
        centroid_points = centroid_pos_set[:, :2]
        centroid_z_coords = np.full((1, len(centroid_points)), z_mean)
        centroid_points = np.hstack((centroid_points, centroid_z_coords.reshape(-1, 1)))
        centroid_cloud = o3d.geometry.PointCloud()
        centroid_cloud.points = o3d.utility.Vector3dVector(centroid_points)
        centroid_color = [[1, 0, 0] for _ in range(len(centroid_points))]
        centroid_cloud.colors = o3d.utility.Vector3dVector(centroid_color)

        # combined_cloud = o3d.geometry.PointCloud()
        # combined_points = np.vstack((internal_points, centered_points))
        # combined_colors = np.vstack((internal_color, centroid_color))
        # combined_cloud.points = o3d.utility.Vector3dVector(combined_points)
        # combined_cloud.colors = o3d.utility.Vector3dVector(combined_colors)

        combined_cloud = internal_cloud + centroid_cloud

        edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        box_line_set.lines = o3d.utility.Vector2iVector(edges)

        box_color = [[0, 0, 1] for _ in range(4)]
        box_line_set.colors = o3d.utility.Vector3dVector(box_color)

        # Create a connection set
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(boundary_cloud.points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_color = np.array([[0, 0, 0] for _ in range(len(boundary_cloud.points))])
        # Dye the part with the largest area
        for i in range(max_area_color_start, max_area_color_end):
            line_color[i] = [1, 0, 0]
        line_set.colors = o3d.utility.Vector3dVector(line_color)
        # Draw the segmentation box
        seg_lines = np.array(seg_lines)
        seg_lines = np.reshape(seg_lines, (-1, 1, 3))
        seg_line_set = o3d.geometry.LineSet()
        seg_line_set.points = o3d.utility.Vector3dVector(seg_lines.reshape(-1, 3))
        seg_line_set.lines = o3d.utility.Vector2iVector(np.arange(len(seg_lines)).reshape(-1, 2))
        # Set the line color to blue
        seg_line_set.colors = o3d.utility.Vector3dVector(np.tile([0, 1, 0], (int(seg_lines.shape[0] / 2), 1)))

        combined_line = box_line_set + line_set + seg_line_set

        self.point_cloud = combined_cloud
        self.line_set = combined_line
        self.update_test(combined_cloud, combined_line)

    def draw_chart_Dialog(self):

        if self.is_para_extraction == 0:
            self.point_cloud = self.tmp_cloud
            self.is_para_extraction = 1
        chart_type = self.comboBox2.currentText()
        print("chart type is ", chart_type)
        self.imageLabel.clear()
        np_data = np.asarray(self.info_set)

        if chart_type == "Area bar":
            data = np_data[:, 2]

            # Define the distribution interval
            bins = [0, 1, 5, 10, 20, 100]
            labels = [1, 2, 3, 4, 5]  # Labels are represented by numbers
            range_labels = ['0 ~ 1 mm2', '1 ~ 5 mm2', '5 ~ 10 mm2', '10 ~ 20 mm2', '20 ~ 100 mm2']

            # Calculate the frequency of each interval
            hist, bin_edges = np.histogram(data, bins=bins)

            # Define name
            names = ['mini hole', 'small hole', 'middle hole', 'huge hole', 'oversize hole']
            # Define color
            colors = ['red', 'green', 'blue', 'orange', 'purple']

            # Draw distribution map
            fig, ax = plt.subplots()
            bars = plt.bar(labels, hist, color=colors, edgecolor='k', alpha=0.7)

            # Add labels and interval information
            for bar, color in zip(bars, colors):
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width() / 2, height, f'{int(height)}', ha='center', va='bottom',
                        fontsize=12, color=color)

            # The display range is in the upper right corner
            for i, (lbl, r, color) in enumerate(zip(names, range_labels, colors)):
                fig.text(0.8, 0.8 - i * 0.05, f'{lbl}: {r}', fontsize=16, verticalalignment='top',
                         horizontalalignment='right', fontfamily='Times New Roman', color=color)

            ax.set_title('Distribution of the pore area(mm2)', fontsize=20, fontfamily='Times New Roman')
            ax.set_xlabel('Number', fontsize=12, fontfamily='Times New Roman')
            ax.set_ylabel('Cavity area', fontsize=12, fontfamily='Times New Roman')
            ax.grid(True)
            ax.set_xticks(labels)

            # Save the plot to a temporary file
            fig.savefig('temp_plot.png', bbox_inches='tight')

        if chart_type == "Area pie":
            data = np_data[:, 2]
            # Define field
            bins = [0, 1, 5, 10, 20, 100]
            labels = ['mini hole: 0 ~ 1 mm2', 'small hole: 1 ~ 5 mm2', 'middle hole: 5 ~ 10 mm2',
                      'huge hole: 10 ~ 20 mm2',
                      'oversize hole: 20 ~ 100 mm2']

            # Segment the data
            hist, bin_edges = np.histogram(data, bins=bins)

            # Draw pie chart
            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99', '#c2c2f0']
            deep_colors = ['red', 'green', 'blue', 'orange', 'purple']

            fig, ax = plt.subplots()
            ax.pie(hist, colors=colors, autopct='%1.1f%%', startangle=140)
            ax.set_title('Pie chart of the pore area', fontsize=16)
            # Add color examples
            for i, (label, color) in enumerate(zip(labels, deep_colors)):
                ax.text(0.8, 1.0 - i * 0.1, f'{label}', color=color, fontsize=12, fontfamily='Times New Roman')

            # Save the plot to a temporary file
            fig.savefig('temp_plot.png', bbox_inches='tight')

        if chart_type == "Distribution bar":
            data = np_data[:, 3]
            # Define field
            bins = list(range(1, self.num_divisions * self.num_divisions + 1))  # 生成从1到n*n的区间，每个区间宽度为1

            # Calculate the frequency of each interval
            hist, bin_edges = np.histogram(data, bins=bins)

            # Draw a histogram to invert the x-axis and y-axis
            fig, ax = plt.subplots()
            bars = plt.barh(range(1, self.num_divisions * self.num_divisions), hist, color='skyblue',
                            edgecolor='skyblue')
            ax.set_xlabel('Number of pores', fontsize=12, fontfamily='Times New Roman')
            ax.set_ylabel('Sequence number', fontsize=12, fontfamily='Times New Roman')
            ax.set_title('Distribution of the pores', fontsize=20, fontfamily='Times New Roman')

            # sets the y axis from 1 to n * n
            ax.set_yticks(range(1, self.num_divisions * self.num_divisions + 1),
                          labels=[''] * self.num_divisions * self.num_divisions)

            # Displays the frequency value for each interval and checks whether it is 0
            for i, value in enumerate(hist):
                y_position = i + 1
                if value == 0:
                    ax.text(0, y_position, str(y_position), color='red', va='center', fontsize=8)
                else:
                    ax.text(0, y_position, str(y_position), color='black', va='center', fontsize=8)

            # Display its value on each bar
            for bar in bars:
                width = bar.get_width()
                y = bar.get_y() + bar.get_height() / 2
                if width == 0:
                    continue
                ax.text(width, y, f'{width}', va='center', ha='left', fontsize=8, color='purple')

            # Save the plot to a temporary file
            fig.savefig('temp_plot.png', bbox_inches='tight')

        if chart_type == "Distribution pie":
            data = np_data[:, 3]
            grid = np.arange(1, self.num_divisions * self.num_divisions + 1).reshape(self.num_divisions,
                                                                                     self.num_divisions)
            # # Define layers
            # outer_layer = np.concatenate([grid[0, :], grid[-1, :], grid[:, 0], grid[:, -1]])
            # middle_layers = np.concatenate(
            #     [grid[1:3, :].flatten(), grid[7:9, :].flatten(), grid[:, 1:3].flatten(), grid[:, 7:9].flatten()])
            # inner_layers = grid[3:7, 3:7].flatten()
            #
            # # Count the number of points in each layer
            # outer_count = len(outer_layer)
            # middle_count = len(middle_layers)
            # inner_count = len(inner_layers)
            element_layers, layer_counts = find_element_layer_and_count(self.num_divisions, data)

            # Draw pie chart
            # labels = ['Outer Layer point number is ', 'Middle Layer point number is ', 'Inner Layer point number is ']
            # sizes = [outer_count, middle_count, inner_count]
            # colors = ['#99ff99', '#66b3ff', '#ff9999']
            # Extract layers and count
            layers = list(layer_counts.keys())
            counts = list(layer_counts.values())

            fig, ax = plt.subplots()
            wedges, texts, autotexts = ax.pie(counts, labels=[f'Layer {layer} has {count} points' for layer, count in zip(layers, counts)], colors=plt.cm.viridis(np.linspace(0, 1, len(layers))), autopct='%1.1f%%', shadow=True, startangle=140,
                                              textprops=dict(color="black"))

            # Add legend
            percentages = [count / sum(counts) * 100 for count in counts]
            ax.legend(wedges,
                      # [f"{label}: {count} ({count / sum(counts) * 100:.1f}%)" for label, count in zip(labels, counts)],
                      [f'Layer {layer}: {percentage:.1f}%' for layer, percentage in zip(layers, percentages)],
                      loc="upper right", fontsize=8, bbox_to_anchor=(1.05, 1))
            self.pore_layer_distribution = ' '.join([f'Layer {layer}: {percentage:.1f}%' for layer, percentage in zip(layers, percentages)])

            ax.set_title('Distribution of Points in Different Layers')
            ax.axis('equal')  # Make the pie chart round

            # Save the plot to a temporary file
            fig.savefig('temp_plot.png', bbox_inches='tight')

        pixmap = QPixmap('temp_plot.png')
        pixmap = pixmap.scaled(self.imageLabel.size(), aspectRatioMode=1)
        self.imageLabel.setPixmap(pixmap)
        os.remove('temp_plot.png')

    def closeEvent(self, event):  # Rewrite the event function
        """
        Refactor the MainWindow 's function closeEvent and end all processes when you exit the software
        :param event:
        :return:
        """
        reply = QtWidgets.QMessageBox.question(self,
                                               'Dialog',
                                               "Do you want to exit?",
                                               QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                               QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
            os._exit(0)
        else:
            event.ignore()

    def draw_update(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def __del__(self):
        # self.clock.stop()
        self.vis.destroy_window()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    # apply_stylesheet(app, theme='dark_cyan.xml')
    window = MainWindow()
    window.setObjectName("MainWindow")
    window.setWindowTitle("Bread parameter extraction application")
    # window.setStyleSheet("#MainWindow{background-color:rgb(220, 220, 220);}")
    window.show()
    sys.exit(app.exec_())
