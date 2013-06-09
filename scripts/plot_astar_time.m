%fid=fopen('planning_june_06.tmp');
%system('scp -r aorthey@fujisan.laas.fr:/home/aorthey/git/feasibility/planning* .');
system('scp -r aorthey@trac.laas.fr:/home/aorthey/git/feasibility/planning* .');

dd = dir('planning_*obj.tmp');

dd(1).name
fid=fopen(dd(1).name);
D=textscan(fid,'%s %d %d %f %f %f %f','CollectOutput', true);
fclose(fid);

if size(dd,1)>1
	for f=2:size(dd,1)
		dd(f).name
		fid=fopen(dd(f).name);

		D2=textscan(fid,'%s %d %d %f %f %f %f','CollectOutput', true);

		D{1}=[D{1};D2{1}];
		D{2}=[D{2};D2{2}];
		D{3}=[D{3};D2{3}];
		fclose(fid);
	end
end

%fid=fopen('planning_60obj.tmp');


%fid=fopen('planning.tmp');
%D2=textscan(fid,'%s %d %d %f %f %f %f','CollectOutput', true);
%D{1}=[D{1};D2{1}];
%D{2}=[D{2};D2{2}];
%D{3}=[D{3};D2{3}];
%fclose(fid);
D

%% OBJECTS | TIME | SUCCESS | STEPS | FEASIBILITY_CHECKS
A=[double(D{2}(:,1)) D{3}(:,1) double(D{2}(:,2)) double(D{3}(:,2)) D{3}(:,4)];

ANN4=A(strcmp(D{1},'ann4'),:);
ANN5=A(strcmp(D{1},'ann5'),:);
ANN6=A(strcmp(D{1},'ann6'),:);
SV=A(strcmp(D{1},'sv'),:);

A6s = arrayfun(@(x) ANN6(ANN6(:,1) == x, :), unique(ANN6(:,1)), 'uniformoutput', false)
A5s = arrayfun(@(x) ANN5(ANN5(:,1) == x, :), unique(ANN5(:,1)), 'uniformoutput', false)
A4s = arrayfun(@(x) ANN4(ANN4(:,1) == x, :), unique(ANN4(:,1)), 'uniformoutput', false)
Ss = arrayfun(@(x) SV(SV(:,1) == x, :), unique(SV(:,1)), 'uniformoutput', false)

Ta=[];
Ts=[];
%Nstd=0.67812;
Nstd=0.01;
for i=1:size(A6s,1)
	Avidx = A6s{i}(:,3)>0;

	meanTime = mean(A6s{i}(Avidx,2));
	stdTime = Nstd*std(A6s{i}( Avidx ,2));
	meanSteps = mean( A6s{i}( Avidx, 4));
	stdSteps = Nstd*std( A6s{i}( Avidx,4));

	meanSuccess = mean(A6s{i}(:,3));
	meanChecksPerTime = mean(A6s{i}(Avidx,5)./A6s{i}(Avidx,2));
	T6a(i,:)=[A6s{i}(1,1) meanTime stdTime meanSteps ...
		stdSteps meanSuccess meanChecksPerTime];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Avidx = A5s{i}(:,3)>0;

	meanTime = mean(A5s{i}(Avidx,2));
	stdTime = Nstd*std(A5s{i}( Avidx ,2));
	meanSteps = mean( A5s{i}( Avidx, 4));
	stdSteps = Nstd*std( A5s{i}( Avidx,4));

	meanSuccess = mean(A5s{i}(:,3));
	meanChecksPerTime = mean(A5s{i}(Avidx,5)./A5s{i}(Avidx,2));
	T5a(i,:)=[A5s{i}(1,1) meanTime stdTime meanSteps ...
		stdSteps meanSuccess meanChecksPerTime];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Avidx = A4s{i}(:,3)>0;

	meanTime = mean(A4s{i}(Avidx,2));
	stdTime = Nstd*std(A4s{i}( Avidx ,2));
	meanSteps = mean( A4s{i}( Avidx, 4));
	stdSteps = Nstd*std( A4s{i}( Avidx,4));

	meanSuccess = mean(A4s{i}(:,3));
	meanChecksPerTime = mean(A4s{i}(Avidx,5)./A4s{i}(Avidx,2));
	T4a(i,:)=[A4s{i}(1,1) meanTime stdTime meanSteps ...
		stdSteps meanSuccess meanChecksPerTime];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	if i<=size(Ss,1)
		Svidx = Ss{i}(:,3)>0;
		%%SV
		meanTime = mean(Ss{i}(Svidx,2));
		stdTime = Nstd*std(Ss{i}( Svidx,2));
		meanSteps = mean( Ss{i}(  Svidx,4));
		stdSteps = Nstd*std( Ss{i}(Svidx ,4));

		meanChecksPerTime = mean(Ss{i}(Svidx,5)./Ss{i}(Svidx,2));
		meanSuccess = mean(Ss{i}(:,3));
		Ts(i,:)=[Ss{i}(1,1) meanTime stdTime meanSteps ...
			stdSteps meanSuccess meanChecksPerTime];
	else
		Ts(i,:) = NaN*T4a(i,:);
	end
end
clf;
Ts
T4a
T5a
T6a
set(gcf,'color','w');
splotXdim = 1;
splotYdim = 3;
lwidth=3.5;
xlimL=0;
xlimH=max(T4a(:,1))+20;


legendFontSize=5;
msize=11;
typeANN4 = '--or';
typeANN5 = '--om';
typeANN6 = '--xm';
typeSV = '-xg';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
sp1=subplot(splotYdim,splotXdim,1);
set(gca,'FontSize',20)
h=errorbar(T4a(:,1),T4a(:,2),T4a(:,3),typeANN4,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(T5a(:,1),T5a(:,2),T5a(:,3),typeANN5,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(T6a(:,1),T6a(:,2),T6a(:,3),typeANN6,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(Ts(:,1),Ts(:,2),Ts(:,3),typeSV,'LineWidth',lwidth,'MarkerSize',msize);
ylim([0 40]);
xlim([xlimL xlimH]);
ylabel('Time');
%legend('ANN(4)','ANN(5)','ANN(6)', 'SV','Location',[0.9 0.8 0.1 0.1],'FontSize',legendFontSize,'MarkerSize',msize);

hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sp2=subplot(splotYdim,splotXdim,2);
set(gca,'FontSize',20)
h=errorbar(T4a(:,1),T4a(:,4),T4a(:,5),typeANN4,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(T5a(:,1),T5a(:,4),T5a(:,5),typeANN5,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(T6a(:,1),T6a(:,4),T6a(:,5),typeANN6,'LineWidth',lwidth,'MarkerSize',msize);
set(h,'MarkerSize',msize);
hold on;
h=errorbar(Ts(:,1),Ts(:,4),Ts(:,5),typeSV,'LineWidth',lwidth,'MarkerSize',msize);

xlim([xlimL xlimH]);
%legend('ANN(4)','ANN(5)','ANN(6)', 'SV','Location','NorthWest','FontSize',legendFontSize);
ylabel('Steps');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sp3=subplot(splotYdim,splotXdim,3);
set(gca,'FontSize',20)
plot(T4a(:,1), T4a(:,6),typeANN4,'LineWidth',lwidth,'MarkerSize',msize);
hold on;
plot(T5a(:,1), T5a(:,6),typeANN5,'LineWidth',lwidth,'MarkerSize',msize);
hold on;
plot(T6a(:,1), T6a(:,6),typeANN6,'LineWidth',lwidth,'MarkerSize',msize);
hold on;
plot(Ts(:,1), Ts(:,6),typeSV,'LineWidth',lwidth,'MarkerSize',msize);
ylim([0 1]);
xlim([xlimL xlimH]);
%legend('ANN(4)','ANN(5)','ANN(6)', 'SV','Location','NorthWest','FontSize',legendFontSize);
ylabel('Success');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%sp4=subplot(splotYdim,splotXdim,4);
%set(gca,'FontSize',20)
%plot(Ta(:,1), Ta(:,7),'-or','LineWidth',lwidth);
%hold on;
%plot(Ts(:,1), Ts(:,7),'-og','LineWidth',lwidth);
%xlim([xlimL xlimH]);
%legend('ANN', 'SV','Location','SouthWest');
%ylabel('CC/s');

xlabel('Objects');

%# find current position [x,y,width,height]
spos1 = get(sp1,'Position');
spos2 = get(sp2,'Position');
spos3 = get(sp3,'Position');
%spos4 = get(sp4,'Position');

set(sp1,'XTickLabel','')
set(sp2,'XTickLabel','')
%set(sp3,'XTickLabel','')

spos2(2) = spos1(2) - spos2(4) - 0.06;
set(sp2,'Position',spos2)
spos3(2) = spos2(2) - spos3(4) - 0.06;
set(sp3,'Position',spos3)
%spos4(2) = spos3(2) - spos4(4) - 0.06;
%set(sp4,'Position',spos4)
print -painters -dpdf -r600 planner_astar.pdf
system('pdfcrop planner_astar.pdf');
