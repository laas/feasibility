function plot_approximation_from_data(X)
	X=X(1:end,:);

	X=X(X(:,5)>=0.01,:);

	%Xin = sortrows(B,-3); %%sort according to z values and remove everything
	%%above the lowest value
	x=X(:,1);
	y=X(:,2);
	z=X(:,3);
	%[junk,i]=unique(x+sqrt(-1)*y);
	%x=x(i);y=y(i);z=z(i);
	%B=Xin(i,:);



	%K=convhulln(Xin);
	%x=K(:,1);
	%y=K(:,2);
	%z=K(:,3);
	%sf = fit( [x, y], z, 'poly44');
	%f=fit([x,y],z,'poly44','Normalize','off','Robust','Bisquare')
	%plot(f,[x,y],z)



	x = x(:);
	y = y(:);
	z = z(:);

%	% Convert all inputs to column vectors.
%	x = x(:);
%	y = y(:);
%	z = z(:);
%
%	% Initialize arrays to store fits and goodness-of-fit.
%	fitresult = cell( 2, 1 );
%	gof = struct( 'sse', cell( 2, 1 ), ...
%	    'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
%
%	%% Fit: 'untitled fit 11'.
%	ft = fittype( 'lowess' );
%	opts = fitoptions( ft );
%	opts.Weights = zeros(1,0);
%	opts.Normalize = 'on';
%	[fitresult{2}, gof(2)] = fit( [x, y], z, ft, opts );
%
%	% Plot fit with data.
%	figure( 'Name', 'untitled fit 11' );
%	h = plot( fitresult{2}, [x, y], z );
%	fitresult{2}.p
%	%legend( h, 'untitled fit 11', 'z vs. x, y', 'Location', 'NorthEast' );
	% Label axes




	%fit_pca_to_data(SF(SF(:,5)<0.01,:));

	%obj = gmdistribution.fit(SF(:,1:3),3,'Options',options)
	%plot_cone_from_data(SF);
	%plot_cone_from_data(FF);
	%h = ezcontour(@(x,y,z)pdf(obj,[x y z]),[-1 1],[-1 1],[0 0.2]);

	%for i=1:size(obj.mu)
	%	plot_gaussian_ellipsoid(obj.mu(i,:), obj.Sigma(:,:,i));
	%end
	%set(gca,'proj','perspective');
	%contour3(X,Y,Z,30)
	%surface(X,Y,Z,'EdgeColor',[.8 .8 .8],'FaceColor','none')
	%group = X(:,5)>0;


	%[xx,yy,zz] = meshgrid(linspace(-1,1),linspace(-1,1),linspace(0,0.2));
	%xx=xx(:);
	%yy=yy(:);
	%zz=zz(:);
	%[C,err,P,logp,coeff] = classify([xx yy zz],[X(:,1) X(:,2) X(:,3)],...
	%                                group,'quadratic');

	%K = coeff(1,2).const;
	%L = coeff(1,2).linear; 
	%Q = coeff(1,2).quadratic;

	%xv = linspace(-1,1,10); % vectors to cover the range of each column
	%yv = linspace(-1,1,10);
	%zv = linspace(0,0.2,10);
	%[xx,yy,zz] = meshgrid(xv,yv,zv);
	%f = @(x,y,z) K + [x y z]*L + sum([x y z] .* ([x y z]*Q), 2);
	%v = f(xx(:),yy(:),zz(:));
	%v = reshape(v,size(xx));
	%%plot3(x(a),y(a),z(a),'rv', x(b),y(b),z(b),'b^');
	%hold on
	%isosurface(xx,yy,zz,v,0);

	%hold off
	%svmStruct = svmtrain(X,group,'showplot',true);
	%plot(sf, [x,y], z)
	%Ksf=convhulln(SF);
	%Kff=convhulln(FF);
	%hold on;
	%p=trisurf(Ksf,SF(:,1),SF(:,2),SF(:,3))
	%set(p,'FaceLighting','phong','FaceColor','blue',...
	%	'FaceAlpha',0.4,...
	%	'EdgeAlpha',0.0,...
	%	'EdgeColor','blue',...
	%      'AmbientStrength',0.5)

	%hold on;
	%p=trisurf(Kff,FF(:,1),FF(:,2),FF(:,3))
	%set(p,'FaceLighting','phong','FaceColor','blue',...
	%	'FaceAlpha',0.4,...
	%	'EdgeAlpha',0.0,...
	%	'EdgeColor','blue',...
	%      'AmbientStrength',0.5)
	%hold on;
	%plot3(Xin(IDX==2,1), Xin(IDX==2,2),Xin(IDX==2,3),'ob','MarkerSize',20);
	%hold on;
	%plot3(Xin(IDX==1,1), Xin(IDX==1,2),Xin(IDX==1,3),'om','MarkerSize',20);

	%plot_halfsphere_from_data(FF);
	hold on;
	%plot_cone_from_data(FF);
	%plot_cone_from_data(SF);
	%plot_halfsphere_from_data(SF);
end
