clear all

% reading in data from '.xlsx' files in current directory
foldercontent=dir('*.xlsx');
files={foldercontent.name};
strat=zeros(1,size(files,2));

for dataset=1:size(files,2)

% SPECIFY SOURCE DATA FRAME HERE! 
filecontent = xlsread(files{1,dataset}, 'B8:C1504');
data(:,1)=filecontent(:,1);
data(:,2)=filecontent(:,2);
[rows,cols]=size(data);

% number of trials in datafile (usually one)
trialmax=cols/2;

% the gridsize affects the heatmap's resolution. Higher values lead to more
% interpolation and thus more processing time. We prefer values from 10-10. 
gridsize=20;

% ENTER ACTUAL GOAL POSITION HERE! 
goalx=18.07;
goaly=33.48;

% If your protocol includes a goal reversal, ENTER PREVIOUS GOAL POSITION HERE!
% If there is no previous goal position, enter a position outside your testing arena.
oldgoalx=1000.0;
oldgoaly=1000.0;

% variable initialization
trial=1;xpos=zeros(size(data,1),trialmax);ypos=zeros(size(data,1),trialmax);
datalength=zeros(1,trialmax);
xpos2=zeros(size(data,1),trialmax);ypos2=zeros(size(data,1),trialmax);x=1;y=2;
d=zeros(size(data,1),trialmax);xsum=zeros(size(data,1),trialmax);
ysum=zeros(size(data,1),trialmax);POGX=zeros(1,trialmax);POGY=zeros(1,trialmax);
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
z0_radius=sqrt((0.14-0.5)^2+(0.5-0.5)^2);
z1_radius=sqrt((0.29-0.5)^2+(0.5-0.5)^2);

% assigning x/y coordinates to own variables
while trial<trialmax+1;
k=1;nc1=0;nc2=0;
while k<rows+1
    nc1=isnan(data(k,x));
    nc2=isnan(data(k,y));
    if nc1==1 || nc2==1
       data(k,x)=data(k-1,x);
       data(k,y)=data(k-1,y);
    end
    nc1=0;nc2=0;
    xpos(k,trial)=data(k,x);
    ypos(k,trial)=data(k,y);
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
           xpos2(j,trial)=((xpos(j,trial)+81.0)/(81.0+81.0));
           ypos2(j,trial)=((ypos(j,trial)+81.0)/(81.0+81.0));
           j=j+1;
    end
    GOALX=((goalx+81.0)/(81.0+81.0));
    GOALY=((goaly+81.0)/(81.0+81.0));
    OLDGOALY=((oldgoalx+81.0)/(81.0+81.0));
    OLDGOALX=((oldgoaly+81.0)/(81.0+81.0));        
     
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
    strat(1,dataset)=7;
elseif dtPOG(1,trial)<0.1 & dtGOAL(1,trial)<0.2
    strat(1,dataset)=6;
elseif dtOLDGOAL(1,trial)<0.25 & dtPOG(1,trial)< 0.2 & dtGOAL > 0.3
    strat(1,dataset)=8;
elseif outliers(1,trial)<25
    strat(1,dataset)=5;
elseif annuluszonerel(1,trial) > 0.5
    strat(1,dataset)=4;
elseif covsurfacerel(1,trial) < 0.75 & wallzone/rows_real < 0.75 & dtCENTER(1,trial)<1.1
    strat(1,dataset)=3;
elseif wallzone/rows_real>0.7 & dtCENTER(1,trial)>0.6
    strat(1,dataset)=1;
elseif covsurfacerel(1,trial) > 0.75 & wallzone/rows_real < 0.7
    strat(1,dataset)=2;
else strat(1,dataset)=0;
end
%############################################################################################
strat;
wallzone=0;centerzone=0;annuluszone=0;
%heatmap calculation
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
    pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)=pmt(pm_ypos(j,1)+1,pm_xpos(j,1)+1)+1;
    end
    j=j+1;
end
pmt_all=pmt_all+pmt; 
clear pm_xpos pm_ypos pmt 

% plotting
h=figure('Position',[50 50 600 300]);
set(gca,'xtick',[0 0.5 1],'ytick',[]);
plot(xpos2_temp(:,1),ypos2_temp(:,1),'k')
set(gca,'xtick',[0 1],'ytick',[0 1]);
axis([0 1 0 1])
title('swimpath')
rectangle('Position',[0,0,1,1],'Curvature',[1,1])
line([0.5 0.5],[1 0],[0 0],'LineStyle','--','Color','k')
line([1 0],[0.5 0.5],[0 0],'LineStyle','--','Color','k')
%rectangle('Position',[0.65,0.25,0.1,0.1],'Curvature',[0,0],'EdgeColor','r')
rectangle('Position',[0.14,0.14,0.72,0.72],'Curvature',[1,1],'EdgeColor','g')
rectangle('Position',[0.29,0.29,0.42,0.42],'Curvature',[1,1],'EdgeColor','g')
line([POGX(1,trial) POGX(1,trial)],[POGY(1,trial) POGY(1,trial)],[0 0],'Color','m','Marker','x')
line([GOALX GOALX],[GOALY GOALY],[0 0],'Color','b','Marker','o')
line([fstx(1,trial) px1g(1,trial)],[fsty(1,trial) py1g(1,trial)],'Linestyle',':','Color',[0 0 0]);
line([fstx(1,trial) px2g(1,trial)],[fsty(1,trial) py2g(1,trial)],'Linestyle',':','Color',[0 0 0]);
%line([fstx(1,trial) px1rg(1,trial)],[fsty(1,trial) py1rg(1,trial)],'Linestyle',':','Color',[0 0 0]);
%line([fstx(1,trial) px2rg(1,trial)],[fsty(1,trial) py2rg(1,trial)],'Linestyle',':','Color',[0 0 0]);
axis('square');
clear xpos_temp ypos_temp;
trial=trial+1;x=x+2;y=y+2;
end

% gathering results into results-array
results.strats(1,dataset)=strat(1,dataset);
results.dtCENTER(1,dataset)=dtCENTER;
results.dtGOAL(1,dataset)=dtGOAL;
results.dtOLDGOAL(1,dataset)=dtOLDGOAL;
results.dtPOG(1,dataset)=dtPOG;
results.eff(1,dataset)=eff;
results.meanalpha(1,dataset)=meanalpha;
results.outliers(1,dataset)=outliers;
end
%heatmap
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
files=files';results