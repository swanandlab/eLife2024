%% Clear previous data
clc; close; clear
%% Select targetlocation Manually from one file
[file, ~] = uigetfile('*.mat');
file=file';
load(file)
imshow(BackgroundCropped)
hold on
targetlocation = drawpoint('Color','g');
mask=false(size(ClearOutsideMaskCropped));
targetlocation = (targetlocation.Position(1,:))
close
%% Select Files
[filenames, ~] = uigetfile('*.mat',...
    'Select One or More Files', ...
    'MultiSelect', 'on');
filenames=filenames';
%%  If only one file selected
if ischar(filenames)
num=1
else
num=length(filenames)    
end
%% Parameters
%fps=30;
StartTime=0; % in sec
EndTime=61; % in sec
ResultName='BM';
targetradius=10;
centerradius=36;
MaskArea=pi*46*46;
walking_speed=5;
running_speed=9;
%% Manual targetlocation location coordinate
 %targetlocation(1)=455
 %targetlocation(2)=646
%% Batch processing
FileNames=[];
AllData=[];
for f=1:num
    if num==1
        load(filenames)
        name={filenames}
        name=name{1}(1:end-4)';
    else
        filename=filenames{f}(1:end-4);
        load(filename)
        name=filename
    end
    pixels=sum(ClearOutsideMaskCropped,'all');
    scale=sqrt(MaskArea/pixels);
    Cmaze = regionprops(ClearOutsideMaskCropped,'Centroid','MajorAxisLength','Area');
    slope = (Cmaze.Centroid(2) - targetlocation(2)) ./ (Cmaze.Centroid(1) - targetlocation(1));
    if Cmaze.Centroid(1)<targetlocation(1)
        angle1 = atand(slope);
    else
        angle1 = 180+atand(slope);
    end
    % Distance Travelled
    startframe=StartTime*fps+1;
    endframe=(EndTime*fps)+1;
    if endframe>length(filteredback)
       endframe=length(filteredback);
    end
    XY=filteredback(startframe:endframe,:);
    x=XY(:,1);
    y=XY(:,2);
    sx=x(1:end-1,:);
    sy=y(1:end-1,:);
    Distance=[];
    Proximity=0;
    for i=1:length(x)-1
        d = sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2)*scale; % calculate distance
        Distance=[Distance;d];
        d = sqrt((x(i+1)-targetlocation(1))^2+(y(i+1)-targetlocation(1))^2)*scale; % Proximity to centre
        Proximity=[Proximity;d]; % Proximity to centre
    end
    imshow(BackgroundCropped)
    MeanProximity=mean(Proximity);
    Timeintarget=0;
    h = drawcircle('Center',[targetlocation(1),targetlocation(2)],...
        'Radius',targetradius/scale,'StripeColor','g');
    tf = inROI(h,sx,sy);
    Timeintarget=sum(tf)/fps;
    Speed=Distance;
    DistanceTravelled=sum(Speed(Speed>(walking_speed/fps)));
    AverageSpeed=mean(Speed(Speed>(walking_speed/fps)))*fps;
    if sum(Speed(Speed>(running_speed/fps)))==0
        Rapid_exploratory_behaviour=0;
    else
        Rapid_exploratory_behaviour=mean(Speed(Speed>(running_speed/fps)))*fps;
    end
    Entries=find(diff(tf)==1);
    No_Entries=length(Entries);
    timesec=(endframe-startframe)/fps
    FirstEntry = find(tf==1, 1 )/fps;
    DistanceTravelledTarget=sum(Speed(Speed(1:(find(tf==1, 1 )))>(walking_speed/fps)));
    if isempty(FirstEntry)
        FirstEntry=NaN;
    end
    Exits=find(diff(tf)==-1);
    No_Exits=length(Exits);
    FirstExit = find(tf==0, 1 )/fps;
    if isempty(FirstExit)
        FirstExit=NaN;
    end
    % Calculate coordinate after first exit
    imshow(BackgroundCropped)
    TimeinCenter=0;
    h = drawcircle('Center',[Cmaze.Centroid(1),Cmaze.Centroid(2)],...
        'Radius',centerradius/scale,'StripeColor','g');
    tf = inROI(h,sx,sy);
    TimeinCenter=sum(tf)/fps;
    CenterExit = find(tf==0, 1 )/fps;
    if isempty(CenterExit)
        CenterExit=NaN;
        Exit_angle=NaN;
        P0 = Cmaze.Centroid;
        P1 = XY(find(tf==0, 1 ),:);
        P2 = targetlocation;
    else
        % calculate first exit angle from centre region (p1p0p2 angle)
        P0 = Cmaze.Centroid;
        P1 = XY(find(tf==0, 1 ),:);
        P2 = targetlocation;
        n1 = (P2 - P0) / norm(P2 - P0);  % Normalized vectors
        n2 = (P1 - P0) / norm(P1 - P0);
        Exit_angle = atan2(norm(det([n2; n1])), dot(n1, n2))*(180/pi) ;
    end
    
    % for each equal quadrant
    th = linspace( -45, 45, 20);
    % quadrant 1
    %hold on
    angle=angle1;
    oRad = Cmaze.MajorAxisLength/2;  %outside radius for full
    x = oRad*cosd(th+angle) + Cmaze.Centroid(1);
    y = oRad*sind(th+angle) + Cmaze.Centroid(2);
    xCenter = Cmaze.Centroid(1);  % Let's have the center/tip of the sector be at the middle of the image.
    yCenter = Cmaze.Centroid(2);
    x1 = [xCenter, x, xCenter];
    y1 = [yCenter, y, yCenter];
    thisROI=[x1;y1]';
    Quadrant=drawpolygon('Position',thisROI,'FaceAlpha',.30,'Color','g','MarkerSize',.1);
    Qroi = inROI(Quadrant,sx,sy);
    TimeQ1=(sum(Qroi)/fps)
    % quadrant 2
    angle=angle1+90;
    oRad = Cmaze.MajorAxisLength/2;  %outside radius for full
    x = oRad*cosd(th+angle) + Cmaze.Centroid(1);
    y = oRad*sind(th+angle) + Cmaze.Centroid(2);
    xCenter = Cmaze.Centroid(1);  % Let's have the center/tip of the sector be at the middle of the image.
    yCenter = Cmaze.Centroid(2);
    x2 = [xCenter, x, xCenter];
    y2 = [yCenter, y, yCenter];
    thisROI=[x2;y2]';
    Quadrant=drawpolygon('Position',thisROI,'FaceAlpha',.15,'Color','g','MarkerSize',.1);
    Qroi = inROI(Quadrant,sx,sy);
    TimeQ2=(sum(Qroi)/fps)
    % quadrant 3
    angle=angle1+180;
    oRad = Cmaze.MajorAxisLength/2;  %outside radius for full
    x = oRad*cosd(th+angle) + Cmaze.Centroid(1);
    y = oRad*sind(th+angle) + Cmaze.Centroid(2);
    xCenter = Cmaze.Centroid(1);  % Let's have the center/tip of the sector be at the middle of the image.
    yCenter = Cmaze.Centroid(2);
    x3 = [xCenter, x, xCenter];
    y3 = [yCenter, y, yCenter];
    thisROI=[x3;y3]';
    Quadrant=drawpolygon('Position',thisROI,'FaceAlpha',.10,'Color','g','MarkerSize',.1);
    Qroi = inROI(Quadrant,sx,sy);
    TimeQ3=(sum(Qroi)/fps)
    % quadrant 4
    angle=angle1+270;
    oRad = Cmaze.MajorAxisLength/2;  %outside radius for full
    x = oRad*cosd(th+angle) + Cmaze.Centroid(1);
    y = oRad*sind(th+angle) + Cmaze.Centroid(2);
    xCenter = Cmaze.Centroid(1);  % Let's have the center/tip of the sector be at the middle of the image.
    yCenter = Cmaze.Centroid(2);
    x4 = [xCenter, x, xCenter];
    y4 = [yCenter, y, yCenter];
    thisROI=[x4;y4]';
    Quadrant=drawpolygon('Position',thisROI,'FaceAlpha',.05,'Color','g','MarkerSize',.1);
    Qroi = inROI(Quadrant,sx,sy);
    TimeQ4=(sum(Qroi)/fps)
    TotalTime=TimeQ1+TimeQ2+TimeQ3+TimeQ4
    % analysis
    FileNames=[FileNames;{name}];
    T1=table(TimeQ1,TimeQ2,TimeQ3,TimeQ4,FirstEntry,DistanceTravelledTarget,Timeintarget,MeanProximity,No_Entries,CenterExit,...
        Exit_angle,DistanceTravelled,AverageSpeed,Rapid_exploratory_behaviour,TotalTime,startframe,endframe);
    AllData=[AllData;T1];
    %plotting tracks
    close all
    imshow(BackgroundCropped)
    hold on
    plot(XY(:,1),XY(:,2),'y-','LineWidth',2)
    hold on
    plot(XY(:,1),XY(:,2),'k-','LineWidth',1)
    hold on
    scatter(P2(1),P2(2),10,'MarkerEdgeColor','Y',...
        'MarkerFaceColor','Y',...
        'LineWidth',1.5)
    hold on
    scatter(P0(1),P0(2),10,'MarkerEdgeColor','M',...
        'MarkerFaceColor','M',...
        'LineWidth',1.5)
    hold on
    exit=XY(find(tf==0, 1 ),:);
    if ~isempty (exit)
    scatter(exit(1),exit(2),10,'MarkerEdgeColor','C',...
        'MarkerFaceColor','C',...
        'LineWidth',1.5)
    end
    hold on
    if ~isempty (exit)
    plot([P2(1);P0(1);exit(1)],[P2(2);P0(2);exit(2)],'b-','LineWidth',1)
    end
    thisROI=[x1;y1]';
    Quadrant=drawpolygon('Position',thisROI,'LineWidth',.001,'FaceAlpha',.10,'Color','g','MarkerSize',.1);
    hold on
    thisROI=[x2;y2]';
    Quadrant=drawpolygon('Position',thisROI,'LineWidth',.001,'FaceAlpha',.10,'Color','m','MarkerSize',.1);
    hold on
    thisROI=[x3;y3]';
    Quadrant=drawpolygon('Position',thisROI,'LineWidth',.001,'FaceAlpha',.10,'Color','M','MarkerSize',.1);
    hold on
    thisROI=[x4;y4]';
    Quadrant=drawpolygon('Position',thisROI,'LineWidth',.001,'FaceAlpha',.10,'Color','m','MarkerSize',.1);
    hold on
    h = drawcircle('Center',[Cmaze.Centroid(1),Cmaze.Centroid(2)],...
        'Radius',centerradius/scale,'LineWidth',.001,'FaceAlpha',.10,'Color','y','MarkerSize',.1);
    TH = drawcircle('Center',[targetlocation(1),targetlocation(2)],...
        'Radius',targetradius/scale,'LineWidth',.001,'FaceAlpha',.20,'Color','g','MarkerSize',.001);
    hold off
    fig=gcf;
    exportgraphics(fig,[name,ResultName, '.tif'],'Resolution',300)
    close all
end
T2=table(FileNames);
CombinedData=[T2,AllData];
writetable(CombinedData,[ResultName,' Results.csv']);