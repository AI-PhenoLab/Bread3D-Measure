@[TOC](Bread3D-Measure)

# Welcome use Bread3D-Measure

Welcome to Bread3D-Measure, we will tell you how to use each module of the application, as well as the environment configuration of the software.

[video(video-JIgXLEnd-1730605452229)(type-csdn)(url-https://live.csdn.net/v/embed/432342)(image-https://v-blog.csdnimg.cn/asset/b7f152674850ba027a3ce38033382e00/cover/Cover0.jpg)(title-Utilize demonstration)]

## Introduction of each module
The modules are linear, and the subsequent module can be executed only upon the completion of the operations of the previous module. Once you have accomplished the entire (6 steps) process, you can return to the first module and deal with the new file.
 1. **File reading module**  You can select or input a path to load point cloud files in various formats (.pcd, .ply, e.g.)
 2. **Preprocessing module**  By clicking the preprocessing button, the software will remove the point cloud where the background interference object is located and retain the point cloud where the bread surface was located.
 3.  **Plane extraction module**  This module also enabled local plane projection, providing threshold scales of 0.5mm, 1mm, 2mm, and 5mm, to extract pores with depths greater than these specified thresholds.
 4. **Pore extraction module**   Four segmentation scales were available, allowing users to divide the planar area into specified block sizes, which served as a basis for pore region and pore layer distribution parameters.
 5. **Chart plotting module** Our software offers four types of charts: pore area bar chart, pore area pie chart, pore distribution bar chart, and pore layer distribution pie chart. Select the diagram that is required to be drawn in accordance with the user's requirements.
 6. **Data export module**  All phenotypic parameters were saved to an Excel file, which was then stored at the specified location selected by user.


## Installation
To run.py files, you need to install the following libraries
```javascript
conda create -n Bread3D python=3.8
conda activate Bread3D
pip install -r requirements.txt
pip install -e .
```
## Export .exe file
If you wish to export a file in the.exe format, the following steps can be taken.
```javascript
cd /File path/Bread3D-Measure
pyinstaller -F Display_test.py
```

## Peroration
The above represents merely the first version of the software, and suggestions will be gathered and sorted out in the future for the improvement of the software.


# AI-PheneLab
