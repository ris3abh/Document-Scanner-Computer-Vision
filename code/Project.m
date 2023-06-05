clear;
close all;

inputImage = imread('image.jpeg');

% Preprocessing steps
grayImage = rgb2gray(inputImage);
[edges, k, sigma] = findOptimumEdges(myImgaussfilt(grayImage, 4));
processedImage = removeBackgroundNoise(edgeDetection(myImgaussfilt(grayImage, 4), k, sigma));
[houghTransformLine, rRange, thetaRange] = houghTransformForLine(processedImage);
localMaxima = findLocalMaxima(houghTransformLine);
[numberOfLines, relevantLines] = relevantLineIdentification(houghTransformLine);
plotTheStatistics(relevantLines, numberOfLines, rRange, thetaRange, processedImage);

subplot(2,2,1);
imshow(processedImage);
title('Edge Detection');


function filteredImage = myImgaussfilt(image, sigma)
    kSize = 2 * ceil(3 * sigma) + 1; 
    kernel = createGaussianKernel(kSize, sigma);  
    filteredImage = conv2(image, kernel, 'same'); 
end

function kernel = createGaussianKernel(kSize, sigma)
    kernel = zeros(kSize, kSize);
    center = (kSize + 1) / 2;
    sumValues = 0;
    for i = 1:kSize
        for j = 1:kSize
            distance = ((i - center)^2 + (j - center)^2) / (2 * sigma^2);
            kernel(i, j) = exp(-distance);
            sumValues = sumValues + kernel(i, j);
        end
    end
    kernel = kernel / sumValues; 
end


function [edges, k, sigma] = findOptimumEdges(image)
    bestScore = 0;
    bestEdges = [];
    bestK = 0;
    bestSigma = 0;
    for k = 3:2:30
        for sigma = 0.5:0.1:3.0
            edgePixels = edgeDetection(image, k, sigma);
            score = sum(edgePixels(:));
            if score > bestScore
                bestScore = score;
                bestEdges = edgePixels;
                bestK = k;
                bestSigma = sigma;
            end
        end
    end
    edges = bestEdges;
    k = bestK;
    sigma = bestSigma;
end

function edgePixels = edgeDetection(image, k, sigma)
    kernel = zeros(k, k);
    center = (k + 1)/2;
    for x = 1:k
        for y = 1:k
            kernel(x, y) = exp(-((x-center)^2 + (y-center)^2) / (2 * sigma^2));
        end
    end
    kernel = kernel / sum(kernel(:));
    smoothImage = conv2(image, kernel, 'same');
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

function processedImage = removeBackgroundNoise(edgePixels)
    binarizedImage = edgePixels > 0;
    labeledImage = zeros(size(binarizedImage));
    label = 1;
    [height, width] = size(binarizedImage);
    for i = 1:height
        for j = 1:width
            if binarizedImage(i, j) == 1 && labeledImage(i, j) == 0
                labeledImage = depthFirstSearch(binarizedImage, labeledImage, label, i, j);
                label = label + 1;
            end
        end
    end
    maxPixels = 0;
    maxLabel = 0;
    for label = 1:(label - 1)
        numPixels = sum(labeledImage(:) == label);
        if numPixels > maxPixels
            maxPixels = numPixels;
            maxLabel = label;
        end
    end
    processedImage = (labeledImage == maxLabel);
end

function labeledImage = depthFirstSearch(binarizedImage, labeledImage, label, i, j)
    [height, width] = size(binarizedImage);
    stack = [i, j];
    while ~isempty(stack)
        currPos = stack(end, :);
        stack(end, :) = [];

        if currPos(1) >= 1 && currPos(1) <= height && currPos(2) >= 1 && currPos(2) <= width
            if binarizedImage(currPos(1), currPos(2)) == 1 && labeledImage(currPos(1), currPos(2)) == 0
                labeledImage(currPos(1), currPos(2)) = label;
                stack = [stack; currPos(1)-1, currPos(2); currPos(1)+1, currPos(2); currPos(1), currPos(2)-1; currPos(1), currPos(2)+1];
            end
        end
    end
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
    threshold = 0.65 * max(houghTransformLine(:));
    [h, w] = size(houghTransformLine);
    localMaxima = findLocalMaxima(houghTransformLine);
    linesAboveThreshold = houghTransformLine >= threshold;
    [rows, cols] = find(localMaxima & (linesAboveThreshold));
    [~, indices] = sort(houghTransformLine(sub2ind([h, w], rows, cols)), 'descend');
    rows = rows(indices);
    cols = cols(indices);
    numberOfLines =  numel(indices);
    relevantLines = [cols(1:numberOfLines), rows(1:numberOfLines)];
end

function cornerPoints = getCornerPoints(numberOfLines, relevantLines, rRange, thetaRange, imageSize)
    % Compute intersection points of lines
    cornerPoints = zeros(4, 2);
    idx = 1;

    for i = 1:numberOfLines-1
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

            % Check if intersection point is within image bounds
            if xIntersect > 0 && xIntersect <= imageSize(2) && yIntersect > 0 && yIntersect <= imageSize(1)
                cornerPoints(idx, :) = [xIntersect, yIntersect];
                idx = idx + 1;
            end
        end
    end

    % Sort corner points based on their sum of coordinates
    cornerSums = sum(cornerPoints, 2);
    [~, topLeftIndex] = min(cornerSums);
    [~, bottomRightIndex] = max(cornerSums);

    % Find top-right and bottom-left corners
    cornerDiffs = diff(cornerPoints, [], 2);
    [~, topRightIndex] = min(cornerDiffs);
    [~, bottomLeftIndex] = max(cornerDiffs);

    % Get corner points
    topLeft = cornerPoints(topLeftIndex, :);
    topRight = cornerPoints(topRightIndex, :);
    bottomLeft = cornerPoints(bottomLeftIndex, :);
    bottomRight = cornerPoints(bottomRightIndex, :);

    % Update cornerPoints array
    cornerPoints = [topLeft; topRight; bottomRight; bottomLeft];
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
    imageSize = size(image);
    cornerPoints = getCornerPoints(numberOfLines, relevantLines, rRange, thetaRange, imageSize);
    
    % Plot lines connecting corner points
    plot(cornerPoints(1, 1), cornerPoints(1, 2), 'g*', 'MarkerSize', 10)
    plot(cornerPoints(2, 1), cornerPoints(2, 2), 'g*', 'MarkerSize', 10)
    plot(cornerPoints(3, 1), cornerPoints(3, 2), 'g*', 'MarkerSize', 10)
    plot(cornerPoints(4, 1), cornerPoints(4, 2), 'g*', 'MarkerSize', 10)
    
    text(cornerPoints(2, 1), cornerPoints(2, 2), 'Top Right', 'Color', 'g', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    text(cornerPoints(1, 1), cornerPoints(1, 2), 'Top Left', 'Color', 'g', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    text(cornerPoints(3, 1), cornerPoints(3, 2), 'Bottom Right', 'Color', 'g', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    text(cornerPoints(4, 1), cornerPoints(4, 2), 'Bottom Left', 'Color', 'g', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
    hold off;
end
