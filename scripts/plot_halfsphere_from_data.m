function plot_halfsphere_from_data(X)

	[Center_LSE,Radius_LSE] = sphereFit(X);

	[Xbs, Ybs, Zbs] = sphere(50);
	zmax = max(X(:,3))

	IDX = Radius_LSE*Zbs(:,1)+Center_LSE(3)<zmax & Radius_LSE*Zbs(:,1)+Center_LSE(3)>0;

	Xbs = Xbs(IDX,:);
	Ybs = Ybs(IDX,:);
	Zbs = Zbs(IDX,:);

	surf(Radius_LSE*Xbs+Center_LSE(1),...
	    Radius_LSE*Ybs+Center_LSE(2),...
	    Radius_LSE*Zbs+Center_LSE(3),'faceAlpha',0.3,'Facecolor','m','EdgeColor','none')

	hold on;
end
