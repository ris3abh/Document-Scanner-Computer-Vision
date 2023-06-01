clear all;
close all;

image = imread('blank.jpeg');
grayImage = rgb2gray(image);

edgePixels = edgeDetection(grayImage);
[houghTransformLine, rRange, thetaRange] = houghTransformForLine(edgePixels);
localMaxima = findLocalMaxima(houghTransformLine);
[numberOfLines, relevantLines] = relevantLineIdentification(houghTransformLine);
plotTheStatistics(relevantLines, numberOfLines, rRange, thetaRange, image)

function edgePixels = edgeDetection(grayImage)
    k = 13;
    sigma = 1.59;
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
    
    subplot(2,2,1);
    imshow(edgePixels);
    title('Edge Detection');
end

function [houghTransformLine, rRange, thetaRange] = houghTransformForLine(edgePixels)
    thetaRange = 0:360;
    w = size(edgePixels,2);
    h = size(edgePixels,1);
    rMax = round(sqrt(w^2 + h^2));
    rRange = -rMax:rMax;
    
    % Create an empty Hough Transform array
    houghTransformLine = zeros(length(rRange), length(thetaRange));
    
    % Loop over all edge pixels in the binary image
    [y,x] = find(edgePixels);
    edges = length(x);
    for i = 1:edges
        % Loop over all theta values
        for j = 1:length(thetaRange)
            r = round(x(i)*cosd(thetaRange(j)) + y(i)*sind(thetaRange(j))) + rMax + 1;
            % Increment the corresponding bin in the Hough Transform array
            houghTransformLine(r,j) = houghTransformLine(r,j) + 1;
        end
    end

    subplot(2,2,2);
    imshow(houghTransformLine',[]);
    title('Hough Transform for Line');
end

function localMaxima = findLocalMaxima(houghTransform)
    [rows, cols] = size(houghTransform);
    localMaxima = false(size(houghTransform));

    for i = 2:rows-1
        for j = 2:cols-1
            pixelValue = houghTransform(i, j);

            % Check if the pixel is greater than its neighbors
            if pixelValue > houghTransform(i-1, j-1) && ...
               pixelValue > houghTransform(i-1, j) && ...
               pixelValue > houghTransform(i-1, j+1) && ...
               pixelValue > houghTransform(i, j-1) && ...
               pixelValue > houghTransform(i, j+1) && ...
               pixelValue > houghTransform(i+1, j-1) && ...
               pixelValue > houghTransform(i+1, j) && ...
               pixelValue > houghTransform(i+1, j+1)
                localMaxima(i, j) = true;
            end
        end
    end
end

function [numberOfLines, relevantLines] = relevantLineIdentification(houghTransformLine)
    threshold = 0.6 * max(houghTransformLine(:));
    [h, w] = size(houghTransformLine);
    localMaxima = findLocalMaxima(houghTransformLine);
    linesAboveThreshold = houghTransformLine >= threshold;
    [rows, cols] = find(localMaxima & (linesAboveThreshold));
    [~, indices] = sort(houghTransformLine(sub2ind([h, w], rows, cols)), 'descend');
    rows = rows(indices);
    cols = cols(indices);
    numberOfLines = min(12, numel(indices));
    relevantLines = [cols(1:numberOfLines), rows(1:numberOfLines)];
end

function [cornerPoints] = getCornerPoints(numberOfLines, relevantLines, rRange, thetaRange)
    cornerPoints = zeros(4, 2);

    % Compute intersection points of lines
    idx = 1;
    for i = 1:numberOfLines
        theta1 = deg2rad(thetaRange(relevantLines(i, 1)));
        r1 = rRange(relevantLines(i, 2));
        m1 = -cos(theta1) / sin(theta1);
        b1 = r1 / sin(theta1);
        for j = i + 1:numberOfLines
            theta2 = deg2rad(thetaRange(relevantLines(j, 1)));
            r2 = rRange(relevantLines(j, 2));
            m2 = -cos(theta2) / sin(theta2);
            b2 = r2 / sin(theta2);
            xIntersect = (b2 - b1) / (m1 - m2);
            yIntersect = m1 * xIntersect + b1;
            cornerPoints(idx, :) = [xIntersect, yIntersect];
            idx = idx + 1;
        end
    end
end

function plotTheStatistics(relevantLines, numberOfLines, rRange, thetaRange, image)
    % Plot lines
    subplot(2,2,3);
    imshow(image);
    hold on;
    for i = 1:numberOfLines
        theta = deg2rad(thetaRange(relevantLines(i, 1)));
        r = rRange(relevantLines(i, 2));
        x = 1:size(image, 2);
        y = (r - x * cos(theta)) / sin(theta);
        plot(x, y, 'Color', 'r');
    end

    % Get corner points
    cornerPoints = getCornerPoints(numberOfLines, relevantLines, rRange, thetaRange);
    
    % Plot corner points
    plot(cornerPoints(:, 1), cornerPoints(:, 2), 'g*', 'MarkerSize', 5);
end
