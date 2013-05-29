function plot_convexhull_from_data(X)
	
	%X=X(1:end,:);
	T=0.0001;
	IN=X(X(:,5)<T,:);
	OUT=X(X(:,5)>T,:);

	B=X;

	%Xin = sortrows(B,-3); %%sort according to z values and remove everything
	%%above the lowest value
	%x=Xin(:,1);
	%y=Xin(:,2);
	%z=Xin(:,3);
	%[junk,i]=unique(x+sqrt(-1)*y);
	%x=x(i);y=y(i);z=z(i);
	%B=Xin(i,:);


	%plot_points_from_data(B, 0.0)
	X=[B(:,1)';B(:,2)';B(:,3)']';
	Xin=[IN(:,1)';IN(:,2)';IN(:,3)']';
	Xout=[OUT(:,1)';OUT(:,2)';OUT(:,3)']';

	K=convhulln(X);
	Ki=convhulln(Xin);
	Ko=convhulln(Xout);

	p=trisurf(Ko,Xout(:,1),Xout(:,2),Xout(:,3))
	set(p,'FaceLighting','phong','FaceColor','green',...
		'FaceAlpha',0.1,...
		'EdgeAlpha',0.0,...
		'EdgeColor','green',...
	      'AmbientStrength',0.5)
	hold on;
	
	%axis tight;


	%%%%%%% Kmeans clustering of the points inside the non-feasible space
	IDX=kmeans(Xin,2);
	Xin(IDX==2)

	SF = Xin(IDX==1,1:3);
	FF = Xin(IDX==2,1:3);
	Ksf=convhulln(SF);
	Kff=convhulln(FF);
	p=trisurf(Ksf,SF(:,1),SF(:,2),SF(:,3))
	set(p,'FaceLighting','phong','FaceColor','red',...
		'FaceAlpha',0.4,...
		'EdgeAlpha',0.0,...
		'EdgeColor','red',...
	      'AmbientStrength',0.5)

	hold on;
	p=trisurf(Kff,FF(:,1),FF(:,2),FF(:,3))
	set(p,'FaceLighting','phong','FaceColor','red',...
		'FaceAlpha',0.4,...
		'EdgeAlpha',0.0,...
		'EdgeColor','red',...
	      'AmbientStrength',0.5)
	hold on;
	%plot3(X(IDX==2,1), X(IDX==2,2),X(IDX==2,3),'*g');
	%plot3(X(IDX==1,1), X(IDX==1,2),X(IDX==1,3),'*r');

	%plot_halfsphere_from_data(FF);
	%plot_halfsphere_from_data(SF);
end
