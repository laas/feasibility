%% INPUT: X -- data Nx3 with x,y,z coordinates of each point
function plot_surface_from_data(Xin)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%% SURF ON POINTS
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	x=Xin(:,1);
	y=Xin(:,2);
	z=Xin(:,3);


	%% resolution
	dx=0.3;
	dy=0.3;

	x_edge=[floor(min(x)):dx:ceil(max(x))];
	y_edge=[floor(min(y)):dy:ceil(max(y))];
	[X,Y]=meshgrid(x_edge,y_edge);
	Z=griddata(x,y,z,X,Y,'cubic');

	h = surf(X,Y,Z,'FaceColor','red','EdgeColor','none','LineStyle','none');
	grid on;
	camlight 
	set(h,'FaceLighting','phong','FaceColor','red',...
	      'AmbientStrength',0.5)
	view(3); 
	axis tight;

	labelFontSize=50;
	xlabel('x','FontSize',labelFontSize,...
		'Interpreter','latex','rot',0,'Position',[0 -7.5 0]);
	ylabel('y','FontSize',labelFontSize,...
		'Interpreter','latex','rot',0,'Position',[-7.5 0 0]);
	zlabel('r','FontSize',labelFontSize,...
		'Interpreter','latex','rot',0,'Position',[-6 6 2] );
	set(gcf, 'PaperPositionMode', 'auto');
	set(gca,'FontSize',30,'LineWidth',2,...
		'FontName','Times',...
		'GridLineStyle',':',...
		'MinorGridLineStyle','--');
end

