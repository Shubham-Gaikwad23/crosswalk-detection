function [mr, mc] = localize(folder, files, houghParams, cameraParams, useFalloff, scheme, intersections, intFolder)

localize = 1;

meterXpixel = cameraParams(4);

res_d = houghParams(1);
res_t = houghParams(2);
min_w = houghParams(3);
max_w = houghParams(4);
w_step = houghParams(5);
delta_t = houghParams(6);
normGradient = houghParams(7);
nonMaxima = houghParams(8);
angleTol = houghParams(9);
minPeak = houghParams(10);
minGradient = houghParams(11);
maxGradient = houghParams(12);

if useFalloff
    thetaFalloffScaling = houghParams(13);
    dFalloffScaling = houghParams(14);
end

% files = dir(strcat(folder,'/*.jpg'));

status = fileattrib(strcat(folder,'/template.png'));

% imageCounter = 0;
F =  [];

T = [];

display('******************************')
display('* Creating Aerial Composite...')
tic
[J, Mask, ~, ~, GPS_data] = createAerialComposite(folder,files, cameraParams, scheme);
toc

H = fspecial('gaussian',[25 25],5);
Jf = imfilter(J,H,'same','replicate');
JE = edge(Jf,'canny');

display('* Haar filtering')
tic
[~, Cmax] = haarFiltering(J, Mask, 0);
toc
interpEdges = 1;
display('* Performing 3D Hough')
tic
% figure, imagesc(Cmax)
if useFalloff
    [HH, C, Cw] = ...
        HoughTransformWidthFast_CutOff(Jf, JE,              ...
        [], res_t,delta_t, res_d, min_w, max_w, w_step,     ...
        [minGradient maxGradient], normGradient,nonMaxima,  ...
        angleTol,thetaFalloffScaling, dFalloffScaling);
    
else
    [HH, C, Cw] = HoughTransformWidthFast(Jf, JE ,...
        [], res_t,delta_t, res_d, min_w, max_w, w_step,     ...
        [minGradient maxGradient], normGradient,nonMaxima,  ...
        angleTol, interpEdges, Cmax);
end
%
toc
display('* Extracting Peaks')
tic
maxd = floor(sqrt(size(J,1)^2 + size(J,2)^2));

lines = extractPeaksLinesGlobal(HH,minPeak, res_t, res_d, maxd);
toc
display('* Creating Segmentation Mask')
tic
F = fillSupportPoints(Jf,lines,C,Cw,res_d, res_t, min_w, w_step);
toc

intersection_name = [];
GPS_MASK = [];
TMASK = [];
display('* Retrieving GIS data')
tic
if GPS_data(1) ~= 0
    %find closest intersection
    [intersection_name] = findClosestIntersection(GPS_data, intersections);
    T = imread(strcat(intFolder,'/',intersection_name,'.png'));
    TMASK = imread(strcat(intFolder,'/',intersection_name,'_PRIOR.png'));
    
elseif status == 1 %we have a template of the intersection
    T = imread(strcat(folder,'/template.png'));
end
toc

if ~isempty(T)
    T = uint8(T);
    TMASK = uint8(TMASK);
    if sum(sum(T==1)) > sum(sum(T==0)) %to fix swapping caused by Gimp
        T = ~T;
    end
    
    T(T>0) = 255;
    TMASK(TMASK>0) = 255;
    
    T = imrotate(T,14.3);
    T = im2double(T);
    Tor = T;
    
    H = fspecial('gaussian',[9 9],3);
    T = imfilter(T, H, 'same');
    T = edge(T,'canny');
    
    display('* Aligning histograms')
    tic
    [maxTheta, flines] = alignHistograms(Tor, T, J, F, meterXpixel, lines);
    toc
    
    display('* Creating Binary Mask')
    tic
    F2 = fillSupportPoints(Jf,flines,C,Cw,res_d, res_t, min_w, w_step);
    toc
    
    if localize
        
        Ctot = zeros(size(T));
        display('* Localizing')
        tic
        [C, ~, Frot, ~, maxCorrAngle] = localizeOrigin(F2,logical(T), J,meterXpixel, maxTheta);
        display(strcat('Max Correlation Angle: ', num2str(maxCorrAngle)));
        toc
        
        Ctot = Ctot + C;
        if ~isempty(TMASK)
            TMASK = imrotate(TMASK,14.3);
            Ctot = Ctot .*double(TMASK);
        end
        
        maxC = max(max(Ctot));
        [mr, mc] = find(Ctot==maxC);
        % if there are more than one max correlation point, get the
        % first one
        mr = mr(1);
        mc = mc(1);
        
        figure, subplot(2,2,1), imshow(J,[]);
        if ~isempty(intersection_name)
            title(intersection_name, 'interpreter','none');
        else
            title(folder);
        end
        subplot(2,2,2), imshow(T), title('Localization Result');
        hold on, plot(mc, mr, '+r'), hold off;
        
        subplot(2,2,3), imagesc(Ctot)
        subplot(2,2,4), imshow(Frot), title('Segmentation');
    end
    
else
    if localize
        figure, subplot(2,2,1), imshow(J,[]), title(strcat('Aerial Composite', folder));
        subplot(2,2,2), imshow(F,[]), title('Segmentation');
        subplot(2,2,3), imshow(~JE,[]), title('Edge Map');
    end
end

end

