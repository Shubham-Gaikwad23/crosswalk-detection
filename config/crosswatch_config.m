function crosswatch_config

addpath(genpath('./utils/'));
addpath('./Hough/');
addpath('./jsonlab/');
global DATASET_ROOT DATASET_JSON INTERSECTION_JSON INTERSECTION_ROOT;

%path to the folder containing the data
DATASET_ROOT = pwd;

%path to the JSON file of the dataset
DATASET_JSON = '.\dataset\test_dataset.json';  

%path to JSON intersections database
INTERSECTION_ROOT = '.\DB_Intersections';
INTERSECTION_JSON = '.\DB_Intersections\intersections.json';

end
