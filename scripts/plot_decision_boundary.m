function plot_decision_boundary(Xin,color)
	%input NxD, whereby D(end) is the evaluation
	DIST=5; %dimension of the distance


	ch=0.1;
	cl=0.01;
	%B=Xin(Xin(:,3)<ch & Xin(:,3)>cl,:);
	%%B=[B;repmat(B(1,:),size(Xin,1)-size(B,1),[])];
	%%Xin=B;
	%size(B)
	%while mod(nthroot(size(B,1),3),1)~=0
	%	%B=Xin(Xin(:,3)<c,:);
	%	K=floor(rand*size(B,1))+1;
	%	B=[B;B(K,:)];
	%%	c=c+0.01;
	%end
	%size(B)
	%Xin=B;
	N=nthroot(size(Xin,1),3);


	%reset(gca);
	%reset(gcf);
	%set(gca,'FontSize',fontsize)
	%set(gca,'color','w');
	%set(gcf,'color','w');

	x=Xin(:,1);
	y=Xin(:,2);
	z=Xin(:,3);
	v=Xin(:,DIST);
	X = reshape(x,[N N N]);
	Y = reshape(y,[N N N]);
	Z = reshape(z,[N N N]);
	V = reshape(v,size(X));
	%A=Xin(Xin(:,DIST)<0.0,:);
	%plot3(A(:,1),A(:,2),A(:,3),'*r');
	hold on;
	[faces,verts,ciso] = isosurface(X,Y,Z,V,0.5,X); 
%idx=verts(:,3)<0.1;
%size(faces)
%size(verts)
%size(ciso)
%verts=verts(idx,:);
%faces=faces(faces(:,3)<size(verts,1) & faces(:,2)<size(verts,1),:)
%ciso=ciso(idx,:);
%
%size(faces)
%size(verts)
%size(ciso)
	p=patch('Vertices', verts, 'Faces', faces, ... 
		'FaceVertexCData', ciso, ... 
		'FaceColor','red', ... 
		'edgecolor', 'none',...
		'AmbientStrength',0.9,...
		'FaceAlpha',0.5);
	%p=patch(isosurface(X,Y,Z,V,0.5));

	grid on;
	camlight;
	fontsize=15;
	fontsizeLabels=25;
	set(gcf,'color','w');
	%set(p,'FaceLighting','phong','FaceColor',color,'EdgeColor','none','LineStyle','none',...
	%      'AmbientStrength',0.9,...
	%      'FaceAlpha',0.5);
	set(gca,'FontSize',fontsize);
	set(gca, 'xlim',[-1.1 1.1]);
	ylim([-1.1 1.1]);
	set(gca, 'zlim',[0.0 0.1]);
	xlabel('x','FontSize',fontsizeLabels,'Position',[-0.1 1.4 0]);
	ylabel('y','FontSize',fontsizeLabels,'Position',[1.5,-0.3,0]);
	zlabel('r','FontSize',fontsizeLabels,'Position',[1.4,-1.3,0.13],'rot',-1);
	axis manual;
	view(120,60);
	%set(lgnd,'FontAngle','italic','FontSize',23,'TextColor',[.3,.2,.1])

end
