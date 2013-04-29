function plotInsideOutsideFunction
	clear all;
	global e1;
	global e2;
	e1=0.1;
	e2=0.1;

	plotter(e1,e2);

end
function plotter(e1,e2)
	clf;
	uicontrol('Style','text',...
		'Position',[500 20 320 20],...
		'String',['Epsilon1 = ',num2str(e1)]);
	uicontrol('Style', 'slider',...
	'Min',0,'Max',2,'Value',e1,...
        'Position',[500 45 320 20],...
	'Callback', {@e1changer}); 
	uicontrol('Style','text',...
		'Position',[0 20 320 20],...
		'String',['Epsilon2 = ',num2str(e2)]);
	uicontrol('Style', 'slider',...
	'Min',0,'Max',2,'Value',e2,...
        'Position',[0 45 320 20],...
	'Callback', {@e2changer}); 
	theta = e1;

	f_cone=@(x,y,z) (x.^2 + y.^2).*cos(theta)^2 - (z.^2).*(sin(theta)^2);
	f_sphere=@(x,y,z) (x.^2 + y.^2 + z.^2);

	A=1;B=1;C=1;
	f_superellipsoid=@(x,y,z) ((x./A).^(2.0/e2)+(y./B).^(2.0/e2)).^(e2/e1)+(z./C).^(2.0/e1);
	f = f_cone;

	N=60;
	ll=-4;
	lh=4;
	xx = linspace(ll,lh,N);
	yy = linspace(ll,lh,N);
	zz = linspace(ll,lh,N);

	[X,Y,Z]=meshgrid(xx,yy,zz);

	fontsize = 18;
	figure(1);
	%reset(gca);
	%reset(gcf);
	set(gca,'FontSize',fontsize)
	set(gca,'color','w');
	set(gcf,'color','w');

	v = f(X(:),Y(:),Z(:));
	v = reshape(v,size(X));
	p=patch(isosurface(X,Y,Z,v,1));

	set(p,'FaceColor','red','EdgeColor','none');
	daspect([1 1 1])
	view(3); 
	axis tight;
	xlim([ll lh]);
	ylim([ll lh]);
	zlim([ll lh]);
	grid on;
	camlight 
	lighting gouraud
end
function e2changer(hObj,event) %#ok<INUSL>
	global e1;
	global e2;
	e2 = get(hObj,'Value');
	plotter(e1,e2);
end
function e1changer(hObj,event) %#ok<INUSL>
	global e1;
	global e2;
	e1 = get(hObj,'Value');
	plotter(e1,e2);
end
