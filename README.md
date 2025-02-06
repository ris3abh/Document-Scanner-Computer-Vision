# Image Rectification and Edge Detection

## Overview
This repository contains MATLAB code for image rectification and edge detection. The primary goal is to process input images, detect edges, identify lines, and rectify the images based on detected corners. The code includes functions for edge detection, Hough transform, line identification, corner point calculation, and image rectification.

## Features
- **Edge Detection**: Uses Gaussian filtering and gradient magnitude to detect edges in images.
- **Hough Transform**: Implements the Hough transform to identify lines in edge-detected images.
- **Line Identification**: Finds relevant lines by identifying local maxima in the Hough transform space.
- **Corner Point Calculation**: Computes intersection points of identified lines to determine corner points.
- **Image Rectification**: Applies a homography transformation to rectify the image based on detected corner points.

## Usage
1. **Input Images**: Place your input images in the `images/` directory.
2. **Run the Code**: Execute the MATLAB script `Project.m` to process the images and visualize the results.
3. **Output**: The script generates rectified images and displays edge detection, Hough transform, and line plots.

## Dependencies
- MATLAB R2021a or later
- Image Processing Toolbox

## Directory Structure
```bash
project/
│
├── code/
│   ├── EdgeDetection.asv
│   ├── EdgeDetection.m
│   ├── Project.m
│   ├── Scanner.m
│   ├── rectified.jpg
│   ├── shit.m
│   └── something.jpg
│
└── images/
├── image1.jpeg
├── image2.jpeg
├── image3.jpeg
└── paper.jpg
```

## Files

### `code/EdgeDetection.m`
- **Description**: Main script for edge detection and image processing.

### `code/Project.m`
- **Description**: Main script to run the entire project, including image rectification and edge detection.

### `code/Scanner.m`
- **Description**: Script for scanning and processing images.

### `code/rectified.jpg`
- **Description**: Example output of a rectified image.

### `images/`
- **Description**: Directory containing input images for processing.
  - `image1.jpeg`
  - `image2.jpeg`
  - `image3.jpeg`
  - `paper.jpg`

## Example Usage
1. **Load Input Images**:
   ```matlab
   inputImage1 = imread('../images/image1.jpeg');
   inputImage2 = imread('../images/image2.jpeg');
   inputImage3 = imread('../images/image3.jpeg');
Run the Main Script:

```bash
run('code/Project.m');
```

View Results:

The script will display edge detection, Hough transform, and rectified images.
Rectified images will be saved in the code/ directory.

For any questions or issues, please contact `rishabh.sharma1103@gmail.com`
