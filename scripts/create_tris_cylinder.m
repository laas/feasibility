x=0;
y=0;
r=2;
z=1;

arg_list = argv();


%for i = 1:nargin
%	    printf("%s\n", arg_list{i});
%end
%printf("\n");
if nargin != 2
	printf("usage: cylinder <r> <z>");
	exit
end
r=str2double(arg_list{1});
z=str2double(arg_list{2});

N=20;%number of corners
M_PI=3.14159265;

file='/home/aorthey/git/fastReplanningData/data/part.tris';
FD = fopen(file,'w');

%% ground plate

fprintf(FD,'%d\n',N+N+2*N);
fprintf(FD,'\n');
oldx = x+r;
oldy = y;
floorz=0;
for t=linspace(0,2*M_PI,N)
	%% first vertex at middle point
	first = [x y floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',x, y,floorz);
	
	%% second vertex at old pos
	second = [oldx oldy floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',oldx, oldy, floorz);

	%%third vertex at
	%t= 2*M_PI/(N-i-1);
	newx = cos(t)*r + x;
	newy = sin(t)*r + y;
	third = [newx newy floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',newx, newy, floorz);

	oldx = newx;
	oldy = newy;
	fprintf(FD,'\n');
	fprintf(FD,'\n');
end
oldx = x+r;
oldy = y;
floorz=z;
for t=linspace(0,2*M_PI,N)
	%% first vertex at middle point
	first = [x y floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',x, y,floorz);
	
	%% second vertex at old pos
	second = [oldx oldy floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',oldx, oldy, floorz);

	%%third vertex at
	%t= 2*M_PI/(N-i-1);
	newx = cos(t)*r + x;
	newy = sin(t)*r + y;
	third = [newx newy floorz];
	fprintf(FD,'%.6f %.6f %.6f\n',newx, newy, floorz);

	oldx = newx;
	oldy = newy;
	fprintf(FD,'\n');
	fprintf(FD,'\n');
end


oldx = x+r;
oldy = y;
zfloor=z;
for t=linspace(0,2*M_PI,N)
	%% first vertex at middle point
	newx = cos(t)*r + x;
	newy = sin(t)*r + y;


	first = [oldx oldy 0];
	second = [oldx oldy zfloor];
	third = [newx newy 0];

	fprintf(FD,'%.6f %.6f %.6f\n',first(1),first(2),first(3));
	fprintf(FD,'%.6f %.6f %.6f\n',second(1),second(2),second(3));
	fprintf(FD,'%.6f %.6f %.6f\n',third(1),third(2),third(3));
	fprintf(FD,'\n');
	fprintf(FD,'\n');


	first = [newx newy 0];
	second = [oldx oldy zfloor];
	third = [newx newy zfloor];
	fprintf(FD,'%.6f %.6f %.6f\n',first(1),first(2),first(3));
	fprintf(FD,'%.6f %.6f %.6f\n',second(1),second(2),second(3));
	fprintf(FD,'%.6f %.6f %.6f\n',third(1),third(2),third(3));


	oldx = newx;
	oldy = newy;
	fprintf(FD,'\n');
	fprintf(FD,'\n');
end















fclose(FD);
command= 'cat %s';
str = sprintf(command, file);
%s = system(str)
