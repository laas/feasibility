clear all;
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/mcmc/sample* ../data/mcmc/');
%prefix = '../data/mcmc/';
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/mcmc_bar/sample* ../data/mcmc_bar/');
%prefix = '../data/mcmc_bar/';
%system('scp aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/data/mcmc_cyl/sample* ../data/mcmc_cyl/');
%prefix = '../data/mcmc_cyl/';
prefix = '../data/test/';



data = dir(fullfile('.',strcat(prefix,'*tmp')));




Nfiles = size(data,1)

%Nfiles = 1;

plot = 1;
plot_plane = 0;
cylindrical = 1;

LABEL_POS= 5;
if plot
	NplotsX = 1;
	NplotsY = 1;
	clf;
	figure(1);
	set(gcf,'color','w');
	fontsize = 18;
	title({'Feasible vs. Non-feasible object positions',''}, 'FontSize', fontsize);
	Nfiles=NplotsX*NplotsY;
end
for k=1:Nfiles
	%XD = [A(:,1:2) zeros(size(A,1))]; %% data samples
	fname = strcat(prefix,data(k).name);
	position_state = sscanf(data(k).name, 'sample_%d_%d_%d',[3]);
	A=load(fname);

	if cylindrical
		XD = to_cylindrical([A(:,1:3)]);
	else
		XD = [A(:,1:3)];
	end
	size(XD)
	YD = [A(:,LABEL_POS)>1]; %% labels '1' = feasible, '0' = nonfeasible

	N_in = find(A(:,LABEL_POS)<=1);
	N_out = find(A(:,LABEL_POS)>1);

	NGRID=25;
	Lb=-2;
	Lh=3;
	[X,Y,Z] = meshgrid(linspace(Lb,Lh,NGRID),linspace(Lb,Lh,NGRID), linspace(Lb,Lh,NGRID));
	X = X(:); Y = Y(:); Z=Z(:);
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Q = 0;
	%[C,err,P,logp,coeff] = classify([X Y Z],XD,YD,'linear')
	[C,err,P,logp,coeff] = classify([X Y Z],XD,YD,'diagquadratic');
	%Q = coeff(1,2).quadratic;
	%K = coeff(1,2).const;
	%L = coeff(1,2).linear;
	L=[0;0;0];
	K=0;

	%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%% compute PCA
	%%%%%%%%%%%%%%%%%%%%%%%
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

		v = d(xx(:),yy(:),zz(:),[L;K]);
		v = reshape(v,size(xx));
		hold on;

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%%%%%% PLOTTING
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		PLOT_DATA = XD;
		feasible = plot3(PLOT_DATA(N_in,1),PLOT_DATA(N_in,2),PLOT_DATA(N_in,3));
		hold on;
		nfeasible = plot3(PLOT_DATA(N_out,1),PLOT_DATA(N_out,2),PLOT_DATA(N_out,3));
		hold on;
		%xlabel('r', 'FontSize', fontsize, 'position', [2,-2.3*pi,0]);

		set(feasible,'Marker','o','MarkerEdgeColor','r','MarkerFaceColor','r','LineStyle','none');
		set(nfeasible,'Marker','o','MarkerEdgeColor','g','MarkerFaceColor','g','LineStyle','none');

		set(gca,'FontSize',fontsize)
		lgnd = legend('non-feasible', 'feasible',...
				'Location','NorthEast');
		a=get(lgnd,'children');
		set(a([1:3:end]),'MarkerSize',20);

		%set(lgnd,'FontAngle','italic','FontSize',23,'TextColor',[.3,.2,.1])
		minZ = min(XD(:,3));
		%xlabel('r', 'Interpreter','latex','FontSize', fontsize, 'position', [0,0,minZ]);
		%ylabel('$$\theta $$', 'Interpreter','latex','FontSize', fontsize, 'position', [-1,0,minZ]);
		%zlabel('z', 'Interpreter','latex','FontSize', fontsize,'rot',180,'position',[-1,minY]);
		labelFontSize=35;

		if cylindrical
			xlabel('r','FontSize',labelFontSize);
			ylabel('\phi','FontSize',labelFontSize);
			zlabel('z','FontSize',labelFontSize,'rot',180);
			set(gca,'YTick',-pi:pi/2:pi)
			set(gca,'YTickLabel',{'-pi','-pi/2','0','pi/2','pi'})

		else
			xlabel('x','FontSize',labelFontSize);
			ylabel('y','FontSize',labelFontSize);
			zlabel('r','FontSize',labelFontSize,'rot',180);
			%set(gca,'ZTick',-pi:pi/2:pi)
			%set(gca,'ZTickLabel',{'-pi','-pi/2','0','pi/2','pi'})

		end

		axis tight;

		if plot_plane
			p = patch(isosurface(xx,yy,zz,v,0)); %%at data value 0
			set(p,'FaceColor','black','EdgeColor','none');
			hold on;
		end

		%campos([1,-6,10]);
		%camtarget([2,0,2]);
		grid on;

		hold on;
	end

end
plane
csvwrite('planeparams.dat',plane);

%plot2svg('samples.svg');
