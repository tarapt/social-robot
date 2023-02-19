% Load the images again as the images will be changed
function stereo_rectify()
    load('calibration/stereoParams.mat', 'stereoParams'); 

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