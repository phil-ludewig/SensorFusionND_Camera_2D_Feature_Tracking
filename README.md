# 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

* Project goal is to evaluate the performance of different feature tracking approaches to uniquely identify object keypoints over several camera frames.

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run: `./2D_feature_tracking`.

## Project Rubric

1. Data Buffer Optimization:
* Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements).
* Approach: Created ring buffer that pushes dataframe into vector as long as the buffer size is not exceeded. On reaching buffer limit, oldest dataframe is erased and replaced by second oldest.

2. Keypoint Detection:
* Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
* Approach: Implemented detectors in detectKeypointsModern() function according to OpenCV class documentation. Detector type selectable in main().

3. Keypoint Removal:
* Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
* Approach: Looped through list of keypoints and checked whether contained in defined rectangle. If contained, keypoint was added to list.

4. Keypoint Descriptors:
* Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
* Approach: Implemented descriptors in descKeypoints() function according to OpenCV class documentation. Descriptor type selectable in main().

5. Descriptor Matching:
* Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
* Approach: Implemented FLANN in matchDescriptors() function. Implemented OpenCV datatype workaround. Added K-NN matching as well.

6. Descriptor Distance Ratio:
* Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
* Approach: Implemented K-NN matching with descriptor distance ratio filter of 0.8.

7. Performance Evaluation 1:
* Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| Detector | Keypoints Total | Keypoints in ROI |
| --- | --- | --- |
| **SHITOMASI** | 1370 | 125 |
| **HARRIS** | 115 | 17 |
| **FAST** | 1824 | 149 |
| **BRISK** | 2757 | 264 |
| **ORB** | 500 | 92 |
| **AKAZE** | 1351 | 166 |
| **SIFT** | 1438 | 138 |

8. Performance Evaluation 2:
* Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
* Note: Matches are within region of interest and examples are shown below (between frame 1 and 2).
* Additional values available in full performance report.
* "N/A" combinations mostly resulted in detector specific errors or out of memory exceptions.

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| **SHITOMASI** | 95 |N/A|N/A|86|N/A|N/A|
| **HARRIS** | 12|N/A|N/A|13|N/A|N/A|
| **FAST** | 97 |N/A|N/A|98|N/A|N/A|
| **BRISK** | 171 |N/A|N/A|160|N/A|N/A|
| **ORB** | 73 |N/A|N/A|42|N/A|N/A|
| **AKAZE** | 137 |N/A|N/A|126|N/A|N/A|
| **SIFT** | 64 |N/A|N/A|65|N/A|82|

9. Performance Evaluation 3:
* Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
* Approach: See complete performance report, fastest combinations in [ms] below.

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| **SHITOMASI** | 20 |N/A|N/A|59|N/A|N/A|
| **HARRIS** | 12|N/A|N/A|52|N/A|N/A|
| **FAST** | 3 |N/A|N/A|42|N/A|N/A|
| **BRISK** | 45 |N/A|N/A|83|N/A|N/A|
| **ORB** | 9 |N/A|N/A|47|N/A|N/A|
| **AKAZE** | 77 |N/A|N/A|110|N/A|N/A|
| **SIFT** | 124 |N/A|N/A|163|N/A|200|

* TOP3 detector/descriptor combinations:
* Old methods (SHITOMASI & HARRIS) are excluded, only modern approaches considered.

| Detector/Descriptor  | NUMBER OF MATCHED ROI KEYPOINTS | TOTAL TIME |
| -------------------- | -------------------- | -------- |
|FAST+BRISK           | 97 keypoints    | 3 ms |
|ORB+BRISK            | 73 keypoints    | 9 ms |
|BRISK+BRISK          | 171 keypoints   | 45 ms |
