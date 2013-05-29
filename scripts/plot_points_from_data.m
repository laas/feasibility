function plot_points_from_data(A, threshold)
	LABEL_POS= 5;
	cylindrical=0;
	THRESHOLD_DB=threshold; %% at what distance do we consider it to be a collision?
	MIN_RAD = 5; %% at which radius do we cut the image


	set(gcf,'color','w');
	fontsize=20;
	if cylindrical
		XD = to_cylindrical([A(:,1:3)]);
	else
		XD = [A(:,1:3)];
	end

	N_in = find(A(:,LABEL_POS)<=THRESHOLD_DB);
	N_out = find(A(:,LABEL_POS)>THRESHOLD_DB);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%% PLOTTING
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	INDATA = XD(N_in,:); %%all samples, which are below the threshold
	OUTDATA = XD(N_out,:);
	PD_F = INDATA(find(INDATA(:,3)<MIN_RAD),:);
	PD_FN = OUTDATA(find(OUTDATA(:,3)<MIN_RAD),:);

	not_feasible = plot3(PD_F(:,1), PD_F(:,2), PD_F(:,3));
	hold on;
	feasible = plot3(PD_FN(:,1), PD_FN(:,2), PD_FN(:,3));
	hold on;
	set(feasible,'Marker','o','MarkerSize',5,'MarkerEdgeColor','g','MarkerFaceColor','g','LineStyle','none');
	set(not_feasible,'Marker','o','MarkerSize',5,'MarkerEdgeColor','r','MarkerFaceColor','r','LineStyle','none');
	set(gca,'FontSize',fontsize)
	lgnd = legend('non-feasible', 'feasible',...
			'Location','NorthEast');
	a=get(lgnd,'children');
	%set(a([1:3:end]),'MarkerSize',20);

	set(lgnd,'FontAngle','italic','FontSize',23,'TextColor',[.3,.2,.1])
end
