clear;
close all;

inputImage = imread('./images/imagex.jpeg');
grayImage = rgb2gray(inputImage);
edges = edgeDetection(grayImage, 19, 1.13);

theta_range = 0:360;
max_dist = ceil(sqrt(size(edges, 1)^2 + size(edges, 2)^2));
r_range = -max_dist:max_dist;
max_val = -inf;
r_min = 0;
r_max = 50;
hough_acc = houghAccumlator(edges, r_range, theta_range, max_dist);
[mxa, bxa, best_r, best_theta] = computeMB(hough_acc, r_range, theta_range, max_val);
img1 = HoughImageAccum(hough_acc);

figure;
subplot(2, 2, 1), imshow(inputImage), title('Original Image');
subplot(2, 2, 2), imshow(edges), title('Edges');
subplot(2, 2, 3), imshow(img1,[]), title("Hough Transform for a Line");
subplot(2, 2, 4);
imshow(inputImage);
hold on;

[selectedLines, numLines] = redLiningLOL(hough_acc);
for i = 1:numLines
    theta = deg2rad(theta_range(selectedLines(i, 1)));
    r = r_range(selectedLines(i, 2));
    x = 1:size(inputImage, 2);
    y = (r - x * cos(theta)) / sin(theta);
    plot(x, y, 'LineWidth', 2, 'Color', 'r');
end

[cornerPoints] = getCornerPoints(numLines, selectedLines,r_range, theta_range);

function[cornerPoints] = getCornerPoints(numLines, selectedLines, r_range, theta_range)
    cornerPoints = zeros(2, 2); 
    
    for i = 1:numLines
        theta1 = deg2rad(theta_range(selectedLines(i, 1)));
        r1 = r_range(selectedLines(i, 2));
        m1 = -cos(theta1) / sin(theta1);
        b1 = r1 / sin(theta1);
        for j = i+1:numLines
            theta2 = deg2rad(theta_range(selectedLines(j, 1)));
            r2 = r_range(selectedLines(j, 2));
            m2 = -cos(theta2) / sin(theta2);
            b2 = r2 / sin(theta2);
            xIntersect = (b2 - b1) / (m1 - m2);
            yIntersect = m1 * xIntersect + b1;
            cornerPoints(i, :) = [xIntersect, yIntersect];
        end
    end
end

% function to get lines and number of lines
function [selectedLines, numLines] = redLiningLOL(hough_acc)
    localMaxima = imregionalmax(hough_acc);
    threshold = 0 * max(hough_acc(:)); % Adjust the threshold value as needed
    thresholdedHough = hough_acc .* (hough_acc >= threshold);
    [maximaRows, maximaCols] = find(localMaxima & (thresholdedHough > 0));
    [maximaVals, idx] = sort(hough_acc(sub2ind(size(hough_acc), maximaRows, maximaCols)), 'descend');
    maximaRows = maximaRows(idx);
    maximaCols = maximaCols(idx);
    numLines = min(8, numel(maximaVals));
    selectedLines = [maximaCols(1:numLines), maximaRows(1:numLines)];
end



% function to calculate hough transform for a line
function hough_acc = houghAccumlator(img, r_range, theta_range, max_dist)
    hough_acc = zeros(length(r_range), length(theta_range));
    [edge_rows, edge_cols] = find(img);
    for i = 1:length(edge_rows)
        x = edge_cols(i);
        y = edge_rows(i);
        for j = 1:length(theta_range)
            theta = deg2rad(theta_range(j));
            r = round(x * cos(theta) + y * sin(theta)) + max_dist + 1;
            hough_acc(r, j) = hough_acc(r, j) + 1;
        end
    end
end

% computing the value of m, b, theta and r(best values)
function [mxa, bxa, best_r, best_theta] = computeMB(hough_acc, r_range, theta_range, max_val)
    best_r = 0;
    best_theta = 0;
    for r_idx = 1:length(r_range)
        for theta_idx = 1:length(theta_range)
            if hough_acc(r_idx, theta_idx) > max_val
                max_val = hough_acc(r_idx, theta_idx);
                best_r = r_range(r_idx);
                best_theta = deg2rad(theta_range(theta_idx));
            end
        end
    end
    mxa = -1 / tan(best_theta);
    bxa = best_r / sin(best_theta);
end

% function for hough image accumlator
function [hough_img] = HoughImageAccum(hough_acc)
    max_hough_val = max(hough_acc(:));
    hough_img = round(255 * hough_acc / max_hough_val);
    hough_img = imrotate(hough_img, 90);
end

% function to generate edge images
function edgePixels = edgeDetection(grayImage, k, sigma)
    kernel = zeros(k, k);
    center = (k + 1)/2;
    for x = 1:k
        for y = 1:k
            kernel(x, y) = exp(-((x-center)^2 + (y-center)^2) / (2 * sigma^2));
        end
    end
    kernel = kernel / sum(kernel(:));
    smoothImage = conv2(grayImage, kernel,"valid");
    dx = zeros(size(smoothImage));
    dy = zeros(size(smoothImage));
    
    for i = 2:size(smoothImage, 1) - 1
        for j = 2:size(smoothImage, 2) - 1
            dx(i,j) = (smoothImage(i + 1, j) - smoothImage(i - 1, j))/2;
            dy(i,j) = (smoothImage(i, j + 1) - smoothImage(i, j - 1))/2;
        end
    end
    magnitude = sqrt(dx.^2 + dy.^2);
    threshold = 0.4 * max(magnitude(:)); 
    edgePixels = magnitude > threshold;
end



