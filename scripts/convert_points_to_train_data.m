function convert_points_to_train_data(X, fname, rmax)
	N=size(X,1);
	dimI = 3;
	dimO = 1;
	V=reshape([X(:,1:3) X(:,5) zeros(size(X,1),2)]',3,[])';
	F=fopen(strcat(fname,'.train'),'w');

	fprintf(F, '%d %d %d\n', N, dimI, dimO);

	for i=1:2*N
		if mod(i,2)==0
			
			if V(i,1)<0.01
				fprintf(F, '-1\n');
			else
				fprintf(F, '1\n');
				%if V(i,1)>0
				%	fprintf(F, '1\n');
				%else
				%	fprintf(F, '1\n');
				%end
			end
			%fprintf(F, '%f\n', V(i,1));
		else
			fprintf(F, '%f %f %f\n', V(i,1), V(i,2), V(i,3));
		end
	end
	fclose(F);
	F=fopen(strcat(fname,'.test'),'w');
	N=40;
	X=linspace(-1.5,1.5,N);
	Y=linspace(-1.5,1.5,N);
	R=linspace(0,rmax,N);


	Xs = size(X,2);
	Ys = size(Y,2);
	Rs = size(R,2);

	fprintf(F, '%d %d %d\n', Xs*Ys*Rs, dimI, dimO);
	for x=1:size(X,2)
		for y=1:size(Y,2)
			for r=1:size(R,2)
				fprintf(F, '%f %f %f\n', X(x),Y(y),R(r));
				fprintf(F, '0\n');
			end
		end
		x
	end
	fclose(F);

end
