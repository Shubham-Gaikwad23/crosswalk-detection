function [Comp, W, Mapping, CompMembers, GPS_data] = createAerialComposite(folder,files, cameraParams, use_reference)
% CREATEAERIALCOMPOSITE(folder,files, cameraParams, use_reference)
% creates a composite aerial image using the scheme defined by
% use_reference. FOLDER and FILES detect the location and the names of the
% image files to use. CAMERAPARAMS contains the focal lenght and the user
% height. USE_REFERENCE = 0|1|2 defines the scheme to use (as described in
% the CRV paper).
%OUTPUT:
% COMP is the composite aerial image. The blending is obtained picking the
% median on the image stack. W is a matrix of the same size as COMP and is
% an overlap counter. MAPPING defines a mapping, for each pixel of the
% composite, from the aerial view to the original image domain. COMPMEMBERS
% contains a binary mask for each image of the composite. GPS_DATA contains
% the average GPS data.

%cameraParams(1) fx
%cameraParams(2) fy
%cameraParams(3) camera height
%cameraParams(4) meters per pixel
%cameraParams(5) Urange
%cameraParams(6) Vrange


%houghParams(1) res_d
%houghParams(2) res_t
%houghParams(3) min_w
%houghParams(4) max_w
%houghParams(5) w_step
%houghParams(6) delta_t
%houghParams(7) normGradient
%houghParams(8) nonMaxima
%houghParams(9) angleTol | angleTolSigma if USEFALLOFF=1
%houghParams(10) minPeak
%houghParams(11) minGradient
%houghParams(12) maxGradient

% if USEFALLOFF = 1
%houghParams(13) theta falloff scaling
%houghParams(14) d falloff scaling

%preprocParams(1) gaussian mask vertical size
%preprocParams(2) gaussian mask horizontal size
%preprocParams(3) gaussian mask sigma


fx = cameraParams(1);
fy = cameraParams(2);
focal = [fx fy];
h = cameraParams(3);


meterXpixel = cameraParams(4);
Urange = cameraParams(5);
Vrange = cameraParams(6);

u2s = -Urange : meterXpixel : Urange;
v2s = -Vrange : meterXpixel : Vrange;

R = [];

TJ = [];

W = zeros(length(v2s),length(u2s));

img_list = [];

upVectorList = zeros(length(files),3);
magneticList = zeros(length(files),3);


GPS = zeros(length(files),4);

isPortrait = -1;
for f = 1 :  length(files)
    
    % create files list
    img_list = [img_list, ' "', strcat(fullfile(folder,'/',files{f}.filename), '" ')];
    [~, ~, ~,magnetic , Rot, GPS(f,:)] = readSensorsData(folder, files{f}.filename);
    upVectorList(f,:) = Rot(3,1:3);
    magneticList(f,:) = Rot(2,1:3);
    if isPortrait < 0 %check it once
        isPortrait = isPortraitMode(upVectorList(f,:));
    end
end

GPS_data = double(zeros(1,3));
m = min(GPS(:,4));
iidx = find(GPS(:,4) == m);

GPS_data(1) = mean(GPS(iidx,1)); %latitude
GPS_data(2) = mean(GPS(iidx,2)); %longitude
GPS_data(3) = mean(GPS(iidx,4)); %radius

R{1} = NaN;
if ~strcmp(img_list,' ')
    % check if we already have the rotation matrices
    status = fileattrib(strcat(folder,'/','R_stitching.mat'));
    
    if use_reference ~= 0
        if status ~= 1
            [~,idx,R] = getStitchMatrices(folder,img_list,[], isPortrait );
        else
            load(strcat(folder,'/','R_stitching.mat'));
            load(strcat(folder,'/','Indices_stitching.mat'));
        end
    else
        idx = 1 : length(files);
    end
    
    if (use_reference ~= 0)
        if isPortrait
            
            Q = qGetQ( [0 1 0;-1 0 0; 0 0 1] * R{1} );
        else
            Q = qGetQ(R{1} );
        end
        Q = [Q(1) -Q(3) -Q(2) -Q(4)];
        
        R1 = (qGetR( Q ));
        
        %take average sensor reading
        for r = 1 : length(idx)
            if isPortrait
                R{r} = [0 1 0;-1 0 0; 0 0 1] * R{r};
                Q = qGetQ( R{r} );
            else
                Q = qGetQ(R{r});
            end
            Q = [Q(1) -Q(3) -Q(2) -Q(4)];
            
            Rr = (qGetR( Q ))';
            upVectorList(idx(r),1:3) = R1*(Rr) * upVectorList(idx(r),1:3)';
            magneticList(idx(r),1:3) = R1*(Rr) * magneticList(idx(r),1:3)';
        end
    end
    
    if use_reference == 1 % SCHEME 1
        upVector = upVectorList(1,1:3);
        magnetic = magneticList(1,1:3);
    elseif use_reference == 2 % SCHEME 2
        upVector = mean(upVectorList(:,1:3));
        magnetic = mean(magneticList(:,1:3));
        
    end
    
    CompMembers = zeros(length(v2s),length(u2s),length(idx));
    Mapping = cell(length(idx));
    for f = 1 : length(idx)
        Ic = imread(strcat(folder,'/',files{idx(f)}.filename));
        
        
        if use_reference ~= 0 % if using SCHEME 1 or 2
            Q = qGetQ( R{f} );
            Q = [Q(1) -Q(3) -Q(2) -Q(4)];
            Rr = qGetR( Q );
            
            upVectCurrent = (Rr)*R1'*upVector(1:3)';
            magnCurrent = (Rr)*R1'* magnetic(1:3)';
            
            upVectCurrent = upVectCurrent / norm(upVectCurrent);
            magnCurrent = magnCurrent  / norm(magnCurrent );
            Rot = zeros(3,3);
            Rot(3,:) = upVectCurrent;
            Rot(2,:) = magnCurrent;
            Rot(1,:) = cross(magnCurrent,upVectCurrent);
            
            yaw = getYawFromRotationMatrix(Rot);
            [~, J, Mask, Mapping{f}] = ...
                reconstructAerialViewFast(Ic, focal,...
                h, upVectCurrent(1:3)', yaw,meterXpixel,Urange, Vrange);
            
        else % if SCHEME 0
            [~, ~, ~, ~,  Rot, ~] = readSensorsData(folder, files{idx(f)}.filename);
            
            yaw = getYawFromRotationMatrix(Rot);
            [~, J, Mask, Mapping{f}] = ...
                reconstructAerialViewFast(Ic, focal,...
                h, upVectorList(idx(f),1:3), yaw,meterXpixel,Urange, Vrange);
        end
        
        W = W + Mask;
        
        CompMembers(find(Mask==1)+(f-1)*size(Mask,1)*size(Mask,2)) = idx(f);
        
        J(Mask==0) = -1;
        J(isnan(J)) = -1;
        
        TJ(Mask==1,end+1) = J(Mask==1);
        TJ(Mask==0,end) = -1;
    end
    
    TJ(TJ == -1) = nan;
    MComp = nanmedian(TJ,2);
    Comp = reshape(MComp,size(J));
    Comp(isnan(Comp)) = 255;
end
end


function isPortrait = isPortraitMode(upVect)
%find on which axis gravity is acting
[~, i] = max(upVect);
isPortrait = i == 2;
end

