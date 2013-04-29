%opengl software;
clear all;
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/cylXYRH/sample* ../data/cylXYRH/');
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/uniformH/r* ../data/uniformH/');
%prefix = '../data/test/';
prefix = '../data/cylXYRH/';
%prefix = '../data/uniformH/';

data = dir(fullfile('.',strcat(prefix,'*tmp')));

Nfiles = size(data,1)
LABEL_POS= 5;
cylindrical=0;
THRESHOLD_DB=0.0; %% at what distance do we consider it to be a collision?
MIN_RAD = 5; %% at which radius do we cut the image


k=2;
clf;
figure(1);
set(gcf,'color','w');
fname = strcat(prefix,data(k).name)
position_state = sscanf(data(k).name, 'sample_%d_%d_%d',[3]);
A=load(fname);
%A(:,1)=A(:,1) - 1;%sqrt(A(:,1).^2+A(:,2).^2);
%A(:,2)=A(:,2) - 1;%sqrt(A(:,1).^2+A(:,2).^2);

if cylindrical
	XD = to_cylindrical([A(:,1:3)]);
else
	XD = [A(:,1:3)];
end
YD = [A(:,LABEL_POS)>1]; %% labels '1' = feasible, '0' = nonfeasible

N_in = find(A(:,LABEL_POS)<=THRESHOLD_DB);
N_out = find(A(:,LABEL_POS)>THRESHOLD_DB);

NGRID=25;
Lb=-2;
Lh=3;
[X,Y,Z] = meshgrid(linspace(Lb,Lh,NGRID),linspace(Lb,Lh,NGRID), linspace(Lb,Lh,NGRID));
X = X(:); Y = Y(:); Z=Z(:);

XN = [XD(N_in,1) XD(N_in,2) XD(N_in,3) ones(size(XD(N_in),1),1)];
XY = [XD(N_out,1) XD(N_out,2) XD(N_out,3) ones(size(XD(N_out),1),1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PLOT_DATA = XD;
%PLOT_DATA = XD(find(XD(:,3)<=0.3),:)
INDATA = A(N_in,:);
OUTDATA = A(N_out,:);
PD_F = INDATA(find(INDATA(:,3)<MIN_RAD),:);
PD_FN = OUTDATA(find(OUTDATA(:,3)<MIN_RAD),:);
%PD_FN = A(find(A(N_out,3)<=0.1),:);
%feasible = plot3(PLOT_DATA(N_in,1),PLOT_DATA(N_in,2),PLOT_DATA(N_in,3));
%hold on;
%nfeasible = plot3(PLOT_DATA(N_out,1),PLOT_DATA(N_out,2),PLOT_DATA(N_out,3));
%hold on;

%not_feasible = plot3(PD_F(:,1), PD_F(:,2), PD_F(:,3));
%hold on;
%feasible = plot3(PD_FN(:,1), PD_FN(:,2), PD_FN(:,3));
%hold on;
%set(not_feasible,'Marker','o','MarkerEdgeColor','r','MarkerFaceColor','r','LineStyle','none');
%set(feasible,'Marker','o','MarkerEdgeColor','g','MarkerFaceColor','g','LineStyle','none');
%
%set(gca,'FontSize',fontsize)
%lgnd = legend('non-feasible', 'feasible',...
%		'Location','NorthEast');
%a=get(lgnd,'children');
%set(a([1:3:end]),'MarkerSize',20);

%set(lgnd,'FontAngle','italic','FontSize',23,'TextColor',[.3,.2,.1])
minZ = min(XD(:,3));

%rotate( feasible, [0 0 1], 90);
%rotate( nfeasible, [0 0 1], 90);
%

%axis tight;
%grid on;
%hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% SURF ON POINTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=A(:,1);
y=A(:,2);
z=A(:,3);

dx=0.3;
dy=0.3;

x_edge=[floor(min(x)):dx:ceil(max(x))];
y_edge=[floor(min(y)):dy:ceil(max(y))];
[X,Y]=meshgrid(x_edge,y_edge);
Z=griddata(x,y,z,X,Y,'cubic');

h = surf(X,Y,Z,'FaceColor','red','EdgeColor','none','LineStyle','none');
grid on;
camlight 
set(h,'FaceLighting','phong','FaceColor','red',...
      'AmbientStrength',0.5)
view(3); 
axis tight;

labelFontSize=50;
xlabel('x','FontSize',labelFontSize,...
	'Interpreter','latex','rot',0,'Position',[0 -7.5 0]);
ylabel('y','FontSize',labelFontSize,...
	'Interpreter','latex','rot',0,'Position',[-7.5 0 0]);
zlabel('r','FontSize',labelFontSize,...
	'Interpreter','latex','rot',0,'Position',[-6 6 2] );
set(gcf, 'PaperPositionMode', 'auto');
set(gca,'FontSize',30,'LineWidth',2,...
	'FontName','Times',...
	'GridLineStyle',':',...
	'MinorGridLineStyle','--');
addpath('extern');
hold on;



% x0       Estimate of the point on the axis. 
%          Dimension: 3 x 1. 
x0 = [0 0 0]';
% a0       Estimate of the axis direction. 
%          Dimension: 3 x 1. 
a0 = [0 0 1]';
% phi0     Estimate of the cone angle.
%          Dimension: 1 x 1. 
phi0 = 30;
% r0       Estimate of the cone radius at x0. 
%          Dimension: 1 x 1. 
r0 = 1;
% tolp     Tolerance for test on step length. 
%          Dimension: 1 x 1. 
tolp = 0.1;
% tolg     Tolerance for test on gradient.
%          Dimension: 1 x 1. 
tolg = 0.01;

%[x0n an phin rn d]=lscone([x y z], x0,a0,phi0,r0);
[x0n, an, phin, rn, d, sigmah, conv, Vx0n, Van, uphin]=lscone([x y z], x0,a0,phi0,r0,tolp,tolg);
% x0n      Estimate of the point on the axis. 
%          Dimension: 3 x 1. 
% an       Estimate of the axis direction. 
%          Dimension: 3 x 1. 
% phin     Estimate of the cone angle.
%          Dimension: 1 x 1. 
% rn       Estimate of the cone radius at x0n. 
%          Dimension: 1 x 1. 
% d        Vector of weighted distances from the points to the cone.
%          Dimension: m x 1. 

h = rn/tan(phin);
x00 = x0n + h*an;

r1=rn;
h1=abs(h);
m1=h1/r1;

h2=5-x00(3);
r2=r1*h2/h1;
m2=h2/r2;


[R,A] = meshgrid(linspace(0,r2,11),linspace(0,2*pi,41));
X = R .* cos(A) + x00(1);
Y = R .* sin(A) + x00(2);
Z = m2*R + x00(3);
P=find(Z(1,:)>-0.5);
X = X(:,P);
Y = Y(:,P);
Z = Z(:,P);
% Cone around the z-axis, point at the origin
%cone=mesh(X,Y,Z)
%set(cone,'FaceLighting','phong','FaceColor','green',...
%	'FaceAlpha',0.3,...
%	'EdgeAlpha',0.2,...
%	'EdgeColor','green',...
%      'AmbientStrength',0.5)



hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% ISOSURFACE OF CONE'S IO FUNCTIOn
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = x00(1);
y0 = x00(2);
r0 = x00(3);
theta = phin;

f=@(x,y,r) ((x-x0).^2 + (y-y0).^2).*cos(theta)^2 - ((r-r0).^2).*(sin(theta)^2);

N=60;
ll=-5;
lh=6;
xx = linspace(ll,lh,N);
yy = linspace(ll,lh,N);
zz = linspace(0,lh,N);

[X,Y,Z]=meshgrid(xx,yy,zz);

%reset(gca);
%reset(gcf);
%set(gca,'FontSize',fontsize)
%set(gca,'color','w');
%set(gcf,'color','w');

v = f(X(:),Y(:),Z(:));
v = reshape(v,size(X));
p=patch(isosurface(X,Y,Z,v,1));
%set(p,'FaceColor','red','EdgeColor','none');
set(p,'FaceLighting','phong','FaceColor','black',...
	'FaceAlpha',0.1,...
	'EdgeAlpha',0.0,...
	'EdgeColor','green',...
      'AmbientStrength',0.5)
view(3);
camorbit(-10, 0);
print -dpng -opengl -r300 ~/git/13humanoids-aorthey/images/cylinder_RXYH.png
