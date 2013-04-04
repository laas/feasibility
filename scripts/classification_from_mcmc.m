clear all;
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/mcmc/sample* ../data/mcmc/');
prefix = '../data/mcmc/';
data = dir(fullfile('.',strcat(prefix,'*tmp')));
Nfiles = size(data,1)

plot = 0;
if plot
	NplotsX = 1;
	NplotsY = 1;
	clf;
	figure(1);
	set(gcf,'color','w');
	fontsize = 15;
	title({'Feasible vs. Non-feasible object positions',''}, 'FontSize', fontsize);
	Nfiles=NplotsX*NplotsY;
end
for k=1:Nfiles
	%XD = [A(:,1:2) zeros(size(A,1))]; %% data samples
	fname = strcat(prefix,data(k).name);
	position_state = sscanf(data(k).name, 'sample_%d_%d_%d',[3]);
	A=load(fname);

	XD = to_cylindrical([A(:,1:3)]);
	YD = [A(:,4)>1]; %% labels '1' = feasible, '0' = nonfeasible

	N_in = find(A(:,4)<=1);
	N_out = find(A(:,4)>1);

	NGRID=25;
	Lb=-2;
	Lh=3;
	[X,Y,Z] = meshgrid(linspace(Lb,Lh,NGRID),linspace(Lb,Lh,NGRID), linspace(Lb,Lh,NGRID));
	X = X(:); Y = Y(:); Z=Z(:);
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Q = 0;
	[C,err,P,logp,coeff] = classify([X Y Z],XD,YD,'linear');
	%[C,err,P,logp,coeff] = classify([X Y Z],XD,YD,'diagquadratic');
	%Q = coeff(1,2).quadratic;
	K = coeff(1,2).const;
	L = coeff(1,2).linear;

	%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%% compute PCA
	%%%%%%%%%%%%%%%%%%%%%%%

	%XN=[XD(N_in,:);XD(N_out,:)];
	%[COEFF,SCORE] = princomp(XN);
	%COEFF
	%subplot(Nplots, Nplots, k);
	%size(N_in)
	%size(XN)
	%size(SCORE)
	%Na=size(N_in,1);
	%Nb=size(N_out,1)+Na;

	%plot(SCORE(1:Na,1), SCORE(1:Na,2), '*r');
	%hold on;
	%plot(SCORE(Na+1:Nb,1), SCORE(Na+1:Nb,2), '*g');


	% for each unique group in 'g', set the ZData property appropriately

	% Function to compute K + L*v + v'*Q*v for multiple vectors
	% v=[x;y]. Accepts x and y as scalars or column vectors.
	f = @(x,y,z,W) W(4) + [x y z]*W(1:3)  + sum([x y z] .* ([x y z]*Q), 2);
	d = @(x,y,z,W) (W(4) + [x y z]*W(1:3))/sum(W(1:3));



	while size(find(d(XD(N_in,1), XD(N_in,2), XD(N_in,3),[L;K])>0),1) > 0 && K<20
		K = K+0.01;
	end

	W=[L;K];
	plane(k,:)=[W' position_state'];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%% OPTIMIZE
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%XX=[XD ones(size(XD,1),1)];

	XN = [XD(N_in,1) XD(N_in,2) XD(N_in,3) ones(size(XD(N_in),1),1)];
	XY = [XD(N_out,1) XD(N_out,2) XD(N_out,3) ones(size(XD(N_out),1),1)];

	%%%%%objective= @(W, X) sum(abs(W'*X')/Ls(W));% - sum(abs(W'*XN(W,X)')/Ls(W));

	%%options = optimset('Algorithm','interior-point');%,'TolFun',1e-10,'MaxIter',2000, 'Display','off'); %
	%options = optimset('Algorithm','sqp'); %
	%W = fmincon(@(W) objective(W, XX, XN),[5;0;0;4],[],[],[],[],[],[],@(W) constrain(W, XN),options);

	%%%% make sure that the normal of the plane is on the 'right' side
	%FF = size(find(W'*XY'>0),2); %non-feasible|wrong decision
	%FR = size(find(W'*XY'<0),2); %non-feasible|right decision
	%%if FF < FR
	%	%W=-W;
	%%end
	%                            
	%%Ws = sqrt(W(1)*W(1) + W(2)*W(2) + W(3)*W(3));
	%%W=W/Ws;
	N = size(find(W'*XN'),2);
	NR = size(find(W'*XN'>0),2); %non-feasible|right decision
	NF = size(find(W'*XN'<0),2); %non-feasible|wrong decision
	%FF = size(find(W'*XY'>0),2); %non-feasible|wrong decision
	FR = size(find(W'*XY'<0),2); %non-feasible|right decision
	%
	%fprintf('Non-Feasible solutions: Right class %d Wrong Class %d\n',NR,NF);
	%fprintf('Feasible solutions: Right class %d Wrong Class %d\n',FR,FF);
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%fprintf('Plane output %f %f %f %f %f\n', W(1),W(2),W(3),W(4), 0);

	fprintf('Plane[%d] output %f %f %f %f (%f false positive)\n', k, W(1),W(2),W(3),W(4), FR/N);



	if plot
		subplot(NplotsX, NplotsY, k);
		xv=linspace(0,5,NGRID);
		yv=linspace(-3.14,3.14,NGRID);
		zv=linspace(-4,4,NGRID);
		[xx,yy,zz] = meshgrid(xv,yv,zv);

		%v = d(xx(:),yy(:),zz(:),W);
		%v = reshape(v,size(xx));
		%hold on;
		%%isosurface(xx,yy,zz,v,0,'Color','black');
		%p = patch(isosurface(xx,yy,zz,v,0)); %%at data value 0
		%%isonormals(xx,yy,zz,v,p)
		%orange=[1,0.6,0];
		%set(p,'FaceColor',orange,'EdgeColor','none');
		%alpha(0.8);
		%isosurface(xx,yy,v,0,'Color','green');
		v = d(xx(:),yy(:),zz(:),[L;K]);
		v = reshape(v,size(xx));
		hold on;
		p = patch(isosurface(xx,yy,zz,v,0)); %%at data value 0
		set(p,'FaceColor','black','EdgeColor','none');
		hold on;
		%nbins=30;
		%din = d(XD(N_in,1), XD(N_in,2), XD(N_in,3),W);
		%dout = d(XD(N_out,1), XD(N_out,2), XD(N_out,3),W);
	%	figure(2);
	%	set(gcf,'color','w');
	%	[nelem,xcen]=hist(dout,nbins);
	%	bar(xcen, nelem);
	%	[nelem,xcen]=hist(din,nbins);
	%	hold on;
	%	bar(xcen, nelem);
	%	h = findobj(gca,'Type','patch');
	%	set(h(1),'FaceColor','g','EdgeColor','k');
	%	set(h(2),'FaceColor','r','EdgeColor','k');

		%set(h2,'Color','m','LineWidth',2)
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%%%%%% PLOTTING
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		plot3(XD(N_in,1),XD(N_in,2),XD(N_in,3),'*r');
		hold on;
		plot3(XD(N_out,1),XD(N_out,2),XD(N_out,3),'*g');
		hold on;
		xlabel('r', 'FontSize', fontsize, 'position', [2,-2.3*pi,0]);
		ylabel('t', 'FontSize', fontsize, 'position', [-1,-0.6*pi,0]);
		zlabel('z', 'FontSize', fontsize,'rot',180,'position',[-1,1.3*pi,2]);
		set(gca,'YTick',-pi:pi/2:pi)
		set(gca,'YTickLabel',{'-pi','-pi/2','0','pi/2','pi'})

		axis tight
		campos([1,-6,10]);
		camtarget([2,0,2]);
		grid on;

		hold on;
	end

end
plane
csvwrite('planeparams.dat',plane);

%plot2svg('samples.svg');
