function MakeSatelliteVideo(dt)

outputFolder = fullfile('C:\Users\Niel\Desktop\1. Masters\2. System\6. Plots\Video');

% Settings
imageFolder = 'C:\Users\Niel\Desktop\1. Masters\2. System\1. Earth Tracker\DistortedImages';
outputVideoName = 'distorted_video.mp4';
videoPath = fullfile(outputFolder,outputVideoName);
frameRate = 1/dt;  % frames per second

% Get list of image files
imageFiles = dir(fullfile(imageFolder, 'distorted_image_*.png'));
imageFiles = sort_nat({imageFiles.name});  % Ensure correct numeric order

% Create VideoWriter object
v = VideoWriter(videoPath, 'MPEG-4');  % Use 'Uncompressed AVI' if you want lossless
v.FrameRate = frameRate;
open(v);

% Write frames
for i = 1:length(imageFiles)
    imgPath = fullfile(imageFolder, imageFiles{i});
    img = imread(imgPath);
    writeVideo(v, img);
end

close(v);
disp('✅ Video created successfully.');

function [cs,index] = sort_nat(c,mode)
    if ~iscellstr(c)
        error('sort_nat:invalidArgument', 'Input must be a cell array of strings.');
    end
    if nargin < 2
        mode = 'ascend';
    end
    expr = '(?<prefix>\D*)(?<number>\d+)(?<suffix>.*)';
    parsed = regexp(c, expr, 'names');
    hasNumber = ~cellfun('isempty', parsed);
    numericPart = zeros(size(c));
    for i = find(hasNumber)
        numericPart(i) = str2double(parsed{i}.number);
    end
    [~, index] = sort(numericPart, mode);
    cs = c(index);
end

end