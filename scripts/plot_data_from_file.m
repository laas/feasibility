function plot_data_from_file(file)
	A=load(file);
	x=A(:,1);
	y=A(:,2);
	z=A(:,3);

	X=[x y z];
	plot_surface_from_data(X);
	hold on;
	plot_cone_from_data(X);
end
