%% INPUT: X -- data Nx3 with x,y,z coordinates of each point
function plot_cone_from_data(Xin)
	addpath('extern');
	set(gcf,'color','w');

	x=Xin(:,1);
	y=Xin(:,2);
	z=Xin(:,3);
	% x0       Estimate of the point on the axis. 
	%          Dimension: 3 x 1. 
	x0 = [0 0 -0.5]';
	% a0       Estimate of the axis direction. 
	%          Dimension: 3 x 1. 
	a0 = [0 0 1]';
	% phi0     Estimate of the cone angle.
	%          Dimension: 1 x 1. 
	phi0 = 40;
	% r0       Estimate of the cone radius at x0. 
	%          Dimension: 1 x 1. 
	r0 = 1.2;
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

	% conv     If conv = 1 the algorithm has converged, 
	%          if conv = 0 the algorithm has not converged
	x0n
	an
	rn
	phin
	x0n
	plot3(x0n(1),x0n(2),x0n(3),'*b','MarkerSize',20);

	axis=[x0n 0.1*an/norm(an)+x0n];

	plot3(axis(1,:),axis(2,:),axis(3,:),'-b','MarkerSize',20,'LineWidth',3);

	radial = [an(2);-an(1);0];

	axis=[x0n rn*radial/norm(radial)+x0n];
	plot3(axis(1,:),axis(2,:),axis(3,:),'-b','MarkerSize',20,'LineWidth',3);
	hold on;

	h = rn/tan(phin*pi/180);
	x00 = x0n - h*an;
	%plot3(x00(1),x00(2),x00(3),'*b','MarkerSize',30);

	r1=rn;
	h1=h;
	m1=h1/r1;

	maxData=max([x y z])';

	h2=norm(maxData-x00);
	r2=r1*h2/h1;
	m2=h2/r2;

	x02 = x00+h2*an/norm(an)

	axis=[x02 r2*radial/norm(radial)+x02];
	plot3(axis(1,:),axis(2,:),axis(3,:),'-b','MarkerSize',20);
	hold on;

	theta = phin*pi/180.0;
	[R,A] = meshgrid(linspace(0,r2,11),linspace(0,2*pi,41));

	X = R .* cos(A) + x00(1);
	Y = R .* sin(A) + x00(2);
	Z = m2*R + x00(3);

	%plot3(X,Y,Z);

	%%% restrict cone to be positive
	%P=find(Z(1,:)>-0.0);
	%X = X(:,P);
	%Y = Y(:,P);
	%Z = Z(:,P);

	hold on;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%% ISOSURFACE OF CONE'S IO FUNCTIOn
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	x0 = x00(1);
	y0 = x00(2);
	r0 = x00(3);

	f=@(x,y,r) ((x-x0).^2 + (y-y0).^2).*cos(theta)^2 - ((r-r0).^2).*(sin(theta)^2);

	N=60;
	xL=min(x);xH=max(x);
	yL=min(y);yH=max(y);
	zL=min(z);zH=max(z);

	xx = linspace(xL,xH,N);
	yy = linspace(yL,yH,N);
	zz = linspace(zL,zH,N);

	[X,Y,Z]=meshgrid(xx,yy,zz);

	v = f(X(:),Y(:),Z(:));
	v = reshape(v,size(X));
	p=patch(isosurface(X,Y,Z,v,0));
	set(p,'FaceLighting','phong','FaceColor','black',...
		'FaceAlpha',0.5,...
		'EdgeAlpha',0.0,...
		'EdgeColor','green',...
	      'AmbientStrength',0.5)
	view(3);
	camorbit(-10, 0);
	hold on;
end
