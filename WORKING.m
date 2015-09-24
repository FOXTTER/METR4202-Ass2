delete(imaqfind)
detect_pen = 1;

%%
f = cameraParams.FocalLength(1);
cx = cameraParams.PrincipalPoint(1);
cy = cameraParams.PrincipalPoint(2);
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

trigger(vidRGB);
trigger(vidDepth);
fprintf('Position of Camera: ');
disp((rot*r*(0-t)' + offset' + framePos' + [0 100 0]')');
%%
avg = getdata(vidRGB);
avg = undistortImage(avg,cameraParams);
avg = imcrop(avg,[284 0 512*3 1080]);
avg = imresize(avg,1/3);
avg = imresize(avg, [361 512]);
avg = flip(avg,2);
avg = int32(rgb2gray(avg));
se = strel('disk',3);
while(1)
    trigger(vidRGB);
    trigger(vidDepth);
    RGB = getdata(vidRGB);

    RGB = undistortImage(RGB,cameraParams);
    depth = getdata(vidDepth);
    depth = imcrop(depth,[0 77/3 512 1080/3]);
    RGB = imcrop(RGB,[284 0 512*3 1080]);
    RGB = imresize(RGB,1/3);
    imgRaw = imresize(RGB, [361 512]);
    imgRaw = flip(imgRaw,2);
    depth = flip(depth,2);

    img = rgb2hsv(imgRaw);
    newBack = int32(rgb2gray(imgRaw));
    imgBin = img(:,:,1) < 50/360 & img(:,:,1) > 0/360;

    back = abs((avg)-(newBack)) > 10;

    res = imgBin & back;
    res = imerode(res,se);

    st = regionprops(res, 'ConvexHull','Area','Centroid','Solidity','Extrema', 'BoundingBox', 'Eccentricity');
    se = strel('disk',1);

    
    

    penImage = xor(back, res);
    figure(1)
    imshow(res);
    title('Final filter')
    figure(4)
    imshow(imgRaw);
    title('Raw image')

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
                realPos = rot*realPos + offset' ;
                fprintf('Position pen(%d): %f, %f, %f)\n', pens, realPos(1),realPos(2),realPos(3));
            end
        end
    end
    for k = 1 : length(st);
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
            realPos = rot*realPos + offset'+ framePos' ;
            fprintf('Position hand(%d): %f, %f, %f)\n', i, realPos(1),realPos(2),realPos(3));
        end
    end
    hold off
end
