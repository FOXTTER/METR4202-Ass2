delete(imaqfind)
calibrate = 0;
detect_pen = 1;

%%
f = 1056.88172825152;
cx = 954.508643076111;
cy = 553.024914356277;
r = cameraParams.RotationMatrices(:,:,end);
t = cameraParams.TranslationVectors(end,:);
rot = [[0 1 0];[1 0 0];[0 0 -1]];
offset = [0 -200 -35];
framePos = [0 0 0];
vidRGB = videoinput('kinectv2imaq', 1, 'RGB32_1920x1080');
vidDepth = videoinput('kinectv2imaq', 2, 'MONO12_512x423');
vidRGB.FramesPerTrigger = 1;
vidRGB.TriggerRepeat = Inf;
triggerconfig(vidRGB, 'manual');
vidDepth.FramesPerTrigger = 1;
vidDepth.TriggerRepeat = Inf;
triggerconfig(vidDepth, 'manual');

start(vidRGB);
start(vidDepth);

%%Color calibration
if (calibrate)
    trigger(vidRGB);
    col = getdata(vidRGB);
    imwrite(col,'col.png');
    T = estimate_color_matrix('col.png', 'GretagMacbeth', 4);
end

trigger(vidRGB);
trigger(vidDepth);
fprintf('Position of Camera: ');
disp((rot*r*(0-t)' + offset' + framePos' + [0 100 0]')');
%%

%avg = gpuArray((getdata(vidRGB)));
avg = getdata(vidRGB);
%avg = ti_hom_lin( T, avg );
avg = undistortImage(avg,cameraParams);
    avg = imcrop(avg,[284 0 512*3 1080]);
    avg = imresize(avg,1/3);
    avg = imresize(avg, [361 512]);
    avg = flip(avg,2);
%
avg = int32(rgb2gray(avg));
%avg = rgb2hsv(avg);
%%
%avg_depth = getdata(vidDepth);
%for i = 1:10
%    img = uint16(rgb2gray(snapshot(cam)));
%    avg = avg + img;
%end
%avg = uint8(avg/10);
se = strel('disk',3);
while(1)
    trigger(vidRGB);
    trigger(vidDepth);
    RGB = getdata(vidRGB);
    %RGB = ti_hom_lin( T, RGB);
    RGB = undistortImage(RGB,cameraParams);
    depth = getdata(vidDepth);
    depth = imcrop(depth,[0 77/3 512 1080/3]);
    RGB = imcrop(RGB,[284 0 512*3 1080]);
    RGB = imresize(RGB,1/3);
    imgRaw = imresize(RGB, [361 512]);
    imgRaw = flip(imgRaw,2);
    depth = flip(depth,2);
    %imgRaw = gpuArray((getdata(vidRGB)));
    %depth = getdata(vidDepth);
    img = rgb2hsv(imgRaw);
    newBack = int32(rgb2gray(imgRaw));
    imgBin = img(:,:,1) < 50/360 & img(:,:,1) > 0/360;
%     avg = 0.98* avg + 0.02*newBack;

    back = abs((avg)-(newBack)) > 10;
    %back = abs(avg(:,:,1) - img(:,:,1)) > 0.1;
    res = imgBin & back;
    res = imerode(res,se);
    %res = imdilate(res,se);
    %CH = bwconvhull(hand,'objects');
    st = regionprops(res, 'ConvexHull','Area','Centroid','Solidity','Extrema', 'BoundingBox', 'Eccentricity');
    se = strel('disk',1);
    %back = imerode(back,se);
    
    
    %res = uint16(res);
    %imshow(depth*20)
                penImage = xor(back, res);
        figure(5)
        imshow(penImage);
        title('Pen image')
    figure(1)
    imshow(res);
    title('Final filter')
    %imshow(depth*30);
    %figure(2)
    %imshow(back);
    %title('Background subtraction')
    %title('Background');
    %figure(3)
    %imshow(imgBin);
    %title('Color filter')
    %title('Color filter');
    %imshow(img);
    figure(4)
    imshow(imgRaw);
    title('Raw image')
    %title('Combined');

    hold on
    hands = 0;
    xm = 0;
    ym = 0;
    zm = 0;
    i = 0;
    clc;
    pens = 0;

    if (detect_pen)
        pen = regionprops(penImage, 'Area', 'Solidity','BoundingBox', 'Eccentricity', 'Centroid');
        for k = 1 : length(pen)
            
            if (pen(k).Eccentricity > 0.95 && pen(k).Area > 50)
                rectangle('Position', pen(k).BoundingBox );
                pens = pens + 1;
                XY = pen(k).Centroid;
                zm = double(depth(round(XY(2)),round(XY(1))));
                [px, py] = getP(XY(1), XY(2), cx, cy);
                xm = (px/f)*zm;
                ym = (py/f)*zm;
                rectangle('Position', pen(k).BoundingBox );
                plot(XY(1),XY(2),'o');   
                hands = hands +1;
                fakePos = [xm, ym, zm];
                realPos = r*(fakePos-t)';
                % The offset values should be recalculted after new setup!
                realPos = rot*realPos + [-20 -150 -15]' ;
                %fprintf('Position: %f, %f, %f)\n', xm,ym,zm);
                fprintf('Position pen(%d): %f, %f, %f)\n', pens, realPos(1),realPos(2),realPos(3));
            end
        end
    end
    for k = 1 : length(st);
        %thisBB = st(k).ConvexHull;
        if (st(k).Solidity < 0.8 && st(k).Area > 1000)
            i= i+1;
            XY = st(k).Centroid;
            zm = double(depth(round(XY(2)),round(XY(1))));
            [px, py] = getP(XY(1), XY(2), cx, cy);
            xm = (px/f)*zm;
            ym = (py/f)*zm;
            rectangle('Position', st(k).BoundingBox );
            plot(XY(1),XY(2),'o');   
            hands = hands +1;
            fakePos = [xm, ym, zm];
            realPos = r*(fakePos-t)';
            % The offset values should be recalculted after new setup!
            realPos = rot*realPos + offset'+ framePos' ;
            %fprintf('Position: %f, %f, %f)\n', xm,ym,zm);
            fprintf('Position hand(%d): %f, %f, %f)\n', i, realPos(1),realPos(2),realPos(3));
        end
    end
    hold off
end
