% Load the images again as the images will be changed
function [point3d, bbox1, bbox2, I1, I2] = triangulate_to_face()
    % load('calibration/stereoParams.mat');

    % alternatively, https://stackoverflow.com/questions/43587226/pass-image-from-python-to-a-matlab-function
    % Read in the stereo pair of images.
    I1 = imread('static_data/left.png');
    I2 = imread('static_data/right.png');

    % Undistort the images.
    I1 = undistortImage(I1,stereoParams.CameraParameters1);
    I2 = undistortImage(I2,stereoParams.CameraParameters2);

    % Stereo rectify the images.
    [I1, I2] = rectifyStereoImages(I1, I2, stereoParams);
    
    % Save the images back.
    imwrite(I1, 'static_data/rectified_left.png');
    imwrite(I2, 'static_data/rectified_right.png');

    % Detect a face in both images.
    faceDetector = vision.CascadeObjectDetector;
    
    % Detections, returned as an M-by-4 element matrix. 
    % Each row of the output matrix contains a four-element vector, [x y width height], 
    % that specifies in pixels, the upper-left corner and size of a bounding box.
    bbox1 = faceDetector(I1);
    bbox2 = faceDetector(I2);
    
    point3d = [];
    if ~isempty(bbox1) &&  ~isempty(bbox2)
        % Find the center of the face. Test this if this works for multiple
        % faces.
        center1 = bbox1(1:2) + bbox1(3:4)/2;
        center2 = bbox2(1:2) + bbox2(3:4)/2;

        % Compute the coordinates of the point from camera 1 in millimeters.
        % 3-D locations of matching pairs of undistorted image points, specified as an M-by-3 matrix. 
        % The matrix contains M number of [x, y, z] locations of matching pairs of undistorted image
        % points from two stereo images.
        point3d = triangulate(center1, center2, stereoParams);
        distanceInMeters = norm(point3d)/1000;

        % Display the detected face and distance.
        distanceAsString = sprintf('%0.2f meters', distanceInMeters);
        I1 = insertObjectAnnotation(I1,'rectangle',bbox1, distanceAsString, 'FontSize', 18);
        I2 = insertObjectAnnotation(I2,'rectangle',bbox2, distanceAsString, 'FontSize', 18);
        I1 = insertShape(I1,'Rectangle', bbox1);
        I2 = insertShape(I2,'Rectangle', bbox2);
    end
    
    imwrite(I1, 'static_data/annotated_left.png');
    imwrite(I2, 'static_data/annotated_right.png');
    
    % imshowpair(I1, I2, 'montage');