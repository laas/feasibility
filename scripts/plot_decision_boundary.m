function plot_decision_boundary(Xin)
	%input NxD, whereby D(end) is the evaluation

	N=nthroot(size(Xin,1),3);


	%reset(gca);
	%reset(gcf);
	%set(gca,'FontSize',fontsize)
	%set(gca,'color','w');
	%set(gcf,'color','w');

	x=Xin(:,1);
	y=Xin(:,2);
	z=Xin(:,3);
	v=Xin(:,4);
	X = reshape(x,[N N N]);
	Y = reshape(y,[N N N]);
	Z = reshape(z,[N N N]);
	V = reshape(v,size(X));
	p=patch(isosurface(X,Y,Z,V,0));

	%h = surf(X,Y,Z,'FaceColor','red','EdgeColor','none','LineStyle','none');
	grid on;
	camlight 
	set(p,'FaceLighting','phong','FaceColor','red','EdgeColor','none','LineStyle','none',...
	      'AmbientStrength',0.5)

end
