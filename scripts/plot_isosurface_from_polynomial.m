function plot_isosurface_from_polynomial()
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
	p=patch(isosurface(X,Y,Z,v,0));
end
