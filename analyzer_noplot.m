clear all
%% Opens window to select files to analyse
[filetoanalyse, pathname] = uigetfile('*.txt','Pick all files to analyse','Multiselect','On');  
%% Creates folder to store MATLAB files
fileFolder = [pathname,'\Summary'];
if ~isdir(fileFolder)
    mkdir(fileFolder)
end    
%% Runs analysis on each file  
for f = 1:length(filetoanalyse)     
formatSpec = '%*f %f %f %*f %*f %*f %*f %*f %*f %*f %*f %*s %*s %*s '; 
fileID = fopen(filetoanalyse{f});
videoIDs=zeros(1,length(filetoanalyse));
D = textscan(fileID,formatSpec,'HeaderLines',18,'Delimiter','\t');
filecontent = [D{1},D{2}];
PosData(:,1)=filecontent(:,1);
PosData(:,2)=filecontent(:,2);
[rows,cols]=size(PosData);
strat=zeros(1,length(filetoanalyse));
clear D fileID filecontent formatSpec

% number of trials in datafile (usually one)
trialmax=cols/2;

% the gridsize affects the heatmap's resolution. Higher values lead to more
% interpolation and thus more processing time. We prefer values from 10-10. 
gridsize=20;

% ENTER ACTUAL GOAL POSITION HERE! - ie position of platform.
goalx = 775;% Determines goal location automatically based on end position
goaly = 1010;

% If your protocol includes a goal reversal, ENTER PREVIOUS GOAL POSITION HERE!
% If there is no previous goal position, enter a position outside your testing arena.
oldgoalx=1000.0;
oldgoaly=1000.0;

% variable initialization
trial=1;xpos=zeros(size(PosData,1),trialmax);ypos=zeros(size(PosData,1),trialmax);
datalength=zeros(1,trialmax);
xpos2=zeros(size(PosData,1),trialmax);ypos2=zeros(size(PosData,1),trialmax);x=1;y=2;
d=zeros(size(PosData,1),trialmax);xsum=zeros(size(PosData,1),trialmax);
ysum=zeros(size(PosData,1),trialmax);POGX=zeros(1,trialmax);POGY=zeros(1,trialmax);
minX=zeros(1,trialmax);maxX=zeros(1,trialmax);
inzone=zeros(3,trialmax); minY=zeros(1,trialmax);maxY=zeros(1,trialmax);
covsurface=zeros(1,trialmax);
dtPOG=zeros(1,trialmax);dtCENTER=zeros(1,trialmax);dtGOAL=zeros(1,trialmax);
dtOLDGOAL=zeros(1,trialmax);
px1=zeros(1,trialmax);wallzonerel=zeros(1,trialmax);
py1=zeros(1,trialmax);px2=zeros(1,trialmax);py2=zeros(1,trialmax);
fstx=zeros(1,trialmax);
fsty=zeros(1,trialmax);
px1g=zeros(1,trialmax);
py1g=zeros(1,trialmax);
px2g=zeros(1,trialmax);py2g=zeros(1,trialmax);
outliers=zeros(1,trialmax);
meanalpha=zeros(1,trialmax);eff=zeros(1,trialmax);wallzone=0;centerzone=0;
annuluszone=0;
stepsize=1/gridsize;n=1; pmt_all=zeros(gridsize+1,gridsize+1);

% annulus zone borders; setting the values allows defining a wide or narrow
% circular zone around the platform's distance to the wall; accordingly, the
% space between annulus and wall is considered as wall-zone, everything bet-
% ween the annulus and the pool center as center-zone 

z0_radius=sqrt((0.14-0.5)^2+(0.5-0.5)^2);%% determines area of platform...
z1_radius=sqrt((0.29-0.5)^2+(0.5-0.5)^2);

% assigning x/y coordinates to own variables
while trial<trialmax+1;
k=1;nc1=0;nc2=0;
while k<rows+1
    nc1=isnan(PosData(k,x));
    nc2=isnan(PosData(k,y));
    if nc1==1 || nc2==1
       PosData(k,x)=PosData(k-1,x);
       PosData(k,y)=PosData(k-1,y);
    end
    nc1=0;nc2=0;
    xpos(k,trial)=PosData(k,x);
    ypos(k,trial)=PosData(k,y);
    k=k+1;
end;
% determination of respective min/max values
xmin=1000;xmax=-1;ymin=1000;ymax=-1;j=1;
    while j < rows+1
        if xpos(j,trial)<xmin
            xmin=xpos(j,trial);
        else
        end
        if xpos(j,trial)>xmax
            xmax=xpos(j,trial);
        else
        end
        if ypos(j,trial)<ymin
            ymin=ypos(j,trial);
        else
        end
        if ypos(j,trial)>ymax
            ymax=ypos(j,trial);
        else
        end
        j=j+1;
    end;           

% removal of overhanging zeroes for xpos
xpos_temp=xpos(:,trial);
xpos_temp(xpos_temp==0)=[];
ypos_temp=ypos(:,trial);
ypos_temp(ypos_temp==0)=[];
[rows_real,cols]=size(xpos_temp);
datalength(1,trial)=rows_real;

% % coordinate-normalization; refer to your tracking software to get the
% min/max x/y values
    j=1;
    while j<rows_real+1
           xpos2(j,trial)=((xpos(j,trial)-461)/(1658-461));
           ypos2(j,trial)=((ypos(j,trial)-157)/(1658-157));
           j=j+1;
    end
    GOALX=((goalx-455)/(1658-461));
    GOALY=((goaly-150)/(1658-157));
    OLDGOALY=((oldgoalx)/(1658));
    OLDGOALX=((oldgoaly)/(1658));         
     
% removal of overhanging zeroes for xpos2
z=1;
for i=1:size(xpos2,1)
   if xpos2(i,1)~=0
       xpos2_temp(z,1)=xpos2(i,1);
       z=z+1;
   else
   end
end
z=1;
for i=1:size(ypos2,1)
   if ypos2(i,1)~=0
       ypos2_temp(z,1)=ypos2(i,1);
       z=z+1;
   else
   end
end

% distance of datapoints to pool center
k=1;
while k<rows
    d(k,trial)=sqrt((xpos2(k,trial)-0.5)^2+(ypos2(k,trial)-0.5)^2);
    k=k+1;
end;

% assorting datapoints to zones
k=1;
while k<rows
    if d(k,trial)>z0_radius
        inzone(1,trial)=inzone(1,trial)+1;
        wallzone=wallzone+1;
    elseif d(k,trial)>z1_radius
        inzone(2,trial)=inzone(2,trial)+1;
        annuluszone=annuluszone+1;
    else
        inzone(3,trial)=inzone(3,trial)+1;
        centerzone=centerzone+1;
    end;
    wallzonerel(1,trial)=wallzone/rows_real;
    annuluszonerel(1,trial)=annuluszone/rows_real;

% precalculation of x/y-sums for centroid
             xsum(1,trial)=xsum(1,trial)+xpos2(k,trial);
             ysum(1,trial)=ysum(1,trial)+ypos2(k,trial);
    k=k+1;    
end

% centroid(=point of gravity, POG)
POGX(1,trial)=xsum(1,trial)/rows_real;
POGY(1,trial)=ysum(1,trial)/rows_real;

% surface coverage - ellipsoid; [%] relative to pool surface
xpos3=zeros(rows_real,1);ypos3=zeros(rows_real,1);
j=1;
while j<rows_real+1
    xpos3(j,1)=xpos2_temp(j,1);
    ypos3(j,1)=ypos2_temp(j,1); 
    j=j+1;
end;
minX(1,trial)=min(xpos3(:,1));
maxX(1,trial)=max(xpos3(:,1));   
minY(1,trial)=min(ypos3(:,1));
maxY(1,trial)=max(ypos3(:,1));   
xdist=abs(minX-maxX);
ydist=abs(minY-maxY);
covsurface(1,trial)=pi*(xdist(1,trial)*ydist(1,trial));
covsurfacemax=pi*1*1;
covsurfacerel=(covsurface/covsurfacemax);

% distance to POG, goal, center [...]
j=1;dtPOGsum=0;dtCENTERsum=0;dtGOALsum=0;dtOLDGOALsum=0;
while j<rows_real+1
    dtPOGim=sqrt((xpos3(j,1)-POGX(1,trial))^2+(ypos3(j,1)-POGY(1,trial))^2);
    dtPOGsum=dtPOGsum+dtPOGim;
    dtPOG(1,trial)=dtPOGsum/rows_real;
    dtCENTERim=sqrt(((xpos3(j,1)-0)^2))+(((ypos3(j,1)-0)^2));
    dtCENTERsum=dtCENTERsum+dtCENTERim;  
    dtCENTER(1,trial)=dtCENTERsum/rows_real;
    dtGOALim=sqrt(((xpos3(j,1)-GOALX)^2))+(((ypos3(j,1)-GOALY)^2));
    dtGOALsum=dtGOALsum+dtGOALim;  
    dtGOAL(1,trial)=dtGOALsum/rows_real;
    dtOLDGOALim=sqrt(((xpos3(j,1)-OLDGOALX)^2))+(((ypos3(j,1)-OLDGOALY)^2));
    dtOLDGOALsum=dtOLDGOALsum+dtOLDGOALim;
    dtOLDGOAL(1,trial)=dtOLDGOALsum/rows_real;
    j=j+1;
end;

% goal-directed corridor

%this defines the angle directed toward the goal
phi=22.5; 
j=1;
fstx(1,trial)=xpos2(1,trial);
fsty(1,trial)=ypos2(1,trial);
nGOALX=GOALX-fstx(1,trial);
nGOALY=GOALY-fsty(1,trial);
while j<size(xpos2,1)+1

% constructing the angle
    px1(1,trial)=(nGOALX*cosd(phi)+nGOALY*sind(phi));
    py1(1,trial)=(-nGOALX*sind(phi)+nGOALY*cosd(phi));
    px2(1,trial)=(nGOALX*cosd(-phi)+nGOALY*sind(-phi));
    py2(1,trial)=(-nGOALX*sind(-phi)+nGOALY*cosd(-phi));
    px1g(1,trial)=px1(1,trial)+fstx(1,trial);
    py1g(1,trial)=py1(1,trial)+fsty(1,trial);
    px2g(1,trial)=px2(1,trial)+fstx(1,trial);
    py2g(1,trial)=py2(1,trial)+fsty(1,trial);
    
% looking for outliers
    x1a=[fstx(1,trial) px1g];
    y1a=[fsty(1,trial) py1g];
    x2=[xpos2(j,trial) GOALX];
    y2=[ypos2(j,trial) GOALY];
    x1b=[fstx(1,trial) px2g];
    y1b=[fsty(1,trial) py2g];
    [xi1,yi1]=polyxpoly(x1a,y1a,x2,y2);
    [xi2,yi2]=polyxpoly(x1b,y1b,x2,y2);
    if isempty(xi1)==0 || isempty(yi1)==0
        outliers(1,trial)=outliers(1,trial)+1;
    elseif isempty(xi2)==0 || isempty(yi2)==0 
        outliers(1,trial)=outliers(1,trial)+1;
    else    
    end
    j=j+1;
end
outliers(1,trial)=(outliers(1,trial)/size(xpos,1))*100;

% average absolute heading error
j=1;alpha=zeros(rows-1,1);
    while j<rows
         m1=(xpos2(j+1,trial)-xpos2(j,1))/(ypos2(j+1,trial)-ypos2(j,trial));
         m2=((GOALX)-xpos2(j,1))/((GOALY)-ypos2(j,1));
         alpha(j,1)=atand((m2-m1)/(1+m1*m2));
    j=j+1;
    end
meanalpha(1,trial)=nanmean(abs(alpha(:,1)));

% path efficiency index
j=1;
    while j<rows
        if abs(alpha(j,1))<15
            eff(1,trial)=eff(1,trial)+1;
        end
        j=j+1;
    end
eff(1,trial)=(eff(1,trial)/rows)*100;

%############################################################################################
% classification
% adjust these values to fit your strategy definitions
if meanalpha(1,trial) < 20  & eff(1,trial) > 80
    strat(1,f)=7;
elseif dtPOG(1,trial)<0.1 & dtGOAL(1,trial)<0.2
    strat(1,f)=6;
elseif dtOLDGOAL(1,trial)<0.25 & dtPOG(1,trial)< 0.2 & dtGOAL > 0.3
    strat(1,f)=8;
elseif outliers(1,trial)<25
    strat(1,f)=5;
elseif annuluszonerel(1,trial) > 0.5
    strat(1,f)=4;
elseif covsurfacerel(1,trial) < 0.75 & wallzone/rows_real < 0.75 & dtCENTER(1,trial)<1.1
    strat(1,f)=3;
elseif wallzone/rows_real>0.7 & dtCENTER(1,trial)>0.6
    strat(1,f)=1;
elseif covsurfacerel(1,trial) > 0.75 & wallzone/rows_real < 0.7
    strat(1,f)=2;
else strat(1,f)=0;
end
%############################################################################################
strat;
wallzone=0;centerzone=0;annuluszone=0;
%% heatmap calculation
i=1;dist=1/gridsize;
while i<gridsize+2
   xg(i,1)=0+((i*dist)-dist);
   yg(i,1)=0+((i*dist)-dist);
   i=i+1;
end
pmt=zeros(gridsize+1,gridsize+1);
j=1;
while j<rows_real+1
    pm_xpos(j,1)=round(xpos3(j,1)/stepsize);
    pm_ypos(j,1)=round(ypos3(j,1)/stepsize);
    if pm_xpos(j,1)<1
        pm_xpos(j,1)=1;
    end
    if pm_ypos(j,1)<1
        pm_ypos(j,1)=1;
    end
    if j>1
    if pm_xpos(j,1)==pm_xpos(j-1,1) && pm_ypos(j,1)==pm_ypos(j-1,1)
    else
       pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)=pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)+1;
    end
    else
    pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)=pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)+1;%error - attempted to access pmt(37,153); index out of bounds because size(pmt) = [21, 21] when j = 1
    end
    j=j+1;
end
pmt_all=pmt_all+pmt; 
clear pm_xpos pm_ypos pmt 

%% Plots graphs showing path taken 

clear xpos_temp ypos_temp;
trial=trial+1;x=x+2;y=y+2;
end

%% gathering results into results-array
results.fileNumber(1,f)= str2num(filetoanalyse{f}(10:end-8));
results.strats(1,f)=strat(1,f);
results.dtCENTER(1,f)=dtCENTER;
results.dtGOAL(1,f)=dtGOAL;
results.dtOLDGOAL(1,f)=dtOLDGOAL;
results.dtPOG(1,f)=dtPOG;
results.eff(1,f)=eff;
results.meanalpha(1,f)=meanalpha;
results.outliers(1,f)=outliers;

clear PosData
end

%% Generates averaged heatmap
figure
[X,Y]=meshgrid(1:1:gridsize+1);
[XI,YI]=meshgrid(1:0.05:gridsize+1);
Z=pmt_all/trialmax;
ZI=interp2(X,Y,Z,XI,YI,'cubic');
cl=[0:0.001:12];
contourf(XI,YI,ZI,500,'LineColor','none');
colormap(jet);
axis square;
rectangle('Position',[1,1,(gridsize),(gridsize)],'Curvature',[1,1],'EdgeColor','w')
%% Saves results in MATLAB file
save([pathname, 'Summary/results'], 'results')
%% Saves results in Excel file
strategy_summary = 'strategy_summary.xlsx';
xl_A = 'VIDEO_ID';
xl_B = 'STRATEGY';
xlswrite(strategy_summary,xl_A,'A1:A1');
xlswrite(strategy_summary,xl_B,'B1:B1');
xlswrite(strategy_summary,results.fileNumber','A2:A99999');
xlswrite(strategy_summary,results.strats','B2:B99999');