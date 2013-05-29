function plot_points_from_file(f,threshold)
	X=load(f);
	%X=X(X(:,5)<0.1 & X(:,5)>-0.1,:);
	plot_points_from_data(X, threshold)
end
