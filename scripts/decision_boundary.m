%opengl software;
clear all;
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/cylXYRH/sample* ../data/cylXYRH/');
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/uniformH/r* ../data/uniformH/');
%prefix = '../data/cylXYRH/';
prefix = '../data/uniformH/';

data = dir(fullfile('.',strcat(prefix,'*tmp')));

Nfiles = size(data,1)
LABEL_POS= 5;
cylindrical=0;
THRESHOLD_DB=0.0; %% at what distance do we consider it to be a collision?
MIN_RAD = 5; %% at which radius do we cut the image
HEIGHT=0.05;


k=1;
A=[];

for k=1:Nfiles
	fname = strcat(prefix,data(k).name)
	B=load(fname);
	B(:,3)=ones(size(B(:,3)))*HEIGHT;
	HEIGHT=HEIGHT+0.05;
	A=[A;B];
end;
	clf;
	figure(1);
	set(gcf,'color','w');
	%position_state = sscanf(data(k).name, 'sample_%d_%d_%d',[3]);

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
	not_feasible = plot3(PD_F(:,1), PD_F(:,2), PD_F(:,3));
	hold on;
	%feasible = plot3(PD_FN(:,1), PD_FN(:,2), PD_FN(:,3));
	hold on;
	set(not_feasible,'Marker','o','MarkerEdgeColor','r','MarkerFaceColor','r','LineStyle','none');
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

	%axis tight;
	%grid on;
	%hold on;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%% CIRCLE FITTING
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	x=PD_F(:,1);
	y=PD_F(:,2);
	z=PD_F(:,3);

	dx=0.05;
	dy=0.05;

	x_edge=[floor(min(x)):dx:ceil(max(x))];
	y_edge=[floor(min(y)):dy:ceil(max(y))];
	[X,Y]=meshgrid(x_edge,y_edge);
	Z=griddata(x,y,z,X,Y,'cubic');
	C=ones(size(Z,1),size(Z,2),3);
	C(:,:,1)=C(:,:,1)*1.0;
	C(:,:,2)=C(:,:,2)*1.0;
	C(:,:,3)=C(:,:,3)*1.0;

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%% SURF ON POINTS
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	h = surf(X,Y,Z,'FaceColor','red','EdgeColor','none','LineStyle','none');
	grid on;
	camlight 
	%set(h,'FaceLighting','phong','FaceColor','red',...
	      %'AmbientStrength',0.5)
	view(3); 
	axis tight;

	labelFontSize=60;
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
	camorbit(-10, 0);
	%pause;

%print -dpng -opengl -r300 ~/git/13humanoids-aorthey/images/cylinder_RXYH.png
