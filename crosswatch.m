function results = crosswatch()

addpath('./config');

global DATASET_ROOT DATASET_JSON INTERSECTION_JSON INTERSECTION_ROOT

%initialize configuration
crosswatch_config();

close all

%cameraParams(1) focal length x
%cameraParams(2) focal length y
%cameraParams(3) camera height
%cameraParams(4) meters per pixel

%houghParams(1) res_d
%houghParams(2) res_t
%houghParams(3) min_w
%houghParams(4) max_w
%houghParams(5) w_step
%houghParams(6) delta_t
%houghParams(7) normGradient
%houghParams(8) nonMaxima
%houghParams(9) angleTol
%houghParams(10) minPeak
%houghParams(11) minGradient
%houghParams(12) maxGradient
%houghParams(13) theta falloff scaling factor
%houghParams(14) d falloff scaling factor

%preprocParams(1) gaussian mask vertical size
%preprocParams(2) gaussian mask horizontal size
%preprocParams(3) gaussian mask sigma

results = {};
dataset=loadjson(DATASET_JSON);
intersections = loadjson(INTERSECTION_JSON);
useFalloff = 0;

res_d = 1;          % bin size for d in the Hough Transf.
res_t = 1;          % bin size for theta in the Hough Transf.
min_w = 12;         % min stripe width
max_w = 16;         % max stripe width
w_step = 1;         % bin size for width in the Hough Transf.
delta_t = 10;       % explore gradient direction of first edge + and - this value
angleTol = 20;      % how consistent is the mates direction (in degrees, 180 +/- angleTol)
minPeak = 15;       % ignore accumulator bins in the Hough with less than this value
minGradient = .0;   % min value for gradient thresholding
maxGradient = 1.1;  % max value for gradient thresholding
normGradient = 1;   % flag to normalize the gradient between [0, 1]

houghParams(1)= res_d;
houghParams(2)= res_t;
houghParams(3)= min_w;
houghParams(4)= max_w;
houghParams(5)= w_step;
houghParams(6)= delta_t;
houghParams(7)= normGradient;
houghParams(8)= 0; %non maxima suppression, if enabled requires VLFEAT 
houghParams(9)= angleTol;
houghParams(10)= minPeak;
houghParams(11) =minGradient;
houghParams(12)= maxGradient;

display('******');

%loop for each folder in the dataset
for f = 1: length(dataset.dataset.series)
    
    display(strcat('Processing: ', dataset.dataset.series{f}.path));
    
    cameraParams(1) = dataset.dataset.series{f}.camera.focal(1);
    cameraParams(2) = dataset.dataset.series{f}.camera.focal(2);
    cameraParams(3) = str2double(dataset.dataset.series{f}.camera.height); %camera height
    
    cameraParams(4) = .118/4;   %meters per pixel (defines the resolution of the aerial view)
    cameraParams(5) = 8;        %width of the aerial view
    cameraParams(6) = 8;        %width of the aerial view
    scheme = 2;                 % 0 - uses IMU reading for each image, ...
                                % 1 - uses a reference IMU reading from the first image,
                                % 2 uses an average of all IMU readings converted to the first image
    if scheme >= 0
        [x,y] = localize(fullfile(DATASET_ROOT,            ...
            dataset.dataset.series{f}.path),    ...
            dataset.dataset.series{f}.images, ...
            houghParams, cameraParams,          ...
            useFalloff, scheme, intersections, INTERSECTION_ROOT);
        results{end+1} = [x y];
    end
    
end
