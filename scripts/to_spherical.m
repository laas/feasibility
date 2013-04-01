function SD=to_spherical(XD)
	X=XD(:,1);
	Y=XD(:,2);
	Z=XD(:,3);
	R=sqrt(X.^2.+Y.^2.+Z.^2);
	T=atan2(sqrt(X.^2+Y.^2),Z);
	P=atan2(Y,X);
	SD=[R T P];
end
