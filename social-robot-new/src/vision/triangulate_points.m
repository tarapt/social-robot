function [point3d] = triangulate_points(leftPoints, rightPoints)
    load('calibration/stereoParams.mat', 'stereoParams');

    % Compute the coordinates of the point from camera 1 in millimeters.
    % 3-D locations of matching pairs of undistorted image points, specified as an M-by-3 matrix. 
    % The matrix contains M number of [x, y, z] locations of matching pairs of undistorted image
    % points from two stereo images.
    point3d = triangulate(leftPoints, rightPoints, stereoParams);
    point3d = mean(point3d)/1000;