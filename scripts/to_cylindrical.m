function SD=to_cylindrical(XD)
	X=XD(:,1);
	Y=XD(:,2);
	Z=XD(:,3);
	R=sqrt(X.^2.+Y.^2);
	T=atan2(Y,X);
	Z=Z;
	SD=[R T Z];
end
