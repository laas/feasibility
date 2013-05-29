function plot_sphere_from_data(X)

	[Center_LSE,Radius_LSE] = sphereFit(X);

	[Base_X,Base_Y,Base_Z] = sphere(20);
	size(Base_X)

	surf(Radius_LSE*Base_X+Center_LSE(1),...
	    Radius_LSE*Base_Y+Center_LSE(2),...
	    Radius_LSE*Base_Z+Center_LSE(3),'faceAlpha',0.3,'Facecolor','m','EdgeColor','none')

	hold on;
end

