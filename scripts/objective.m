function [J]=objective(W,XY,XN)
	Ws = sqrt(W(1)*W(1) + W(2)*W(2) + W(3)*W(3));
	%J= sum(abs(W'*X')/Ws);%- sum(abs(W'*XN')/Ws);
	
	J = size(find(W'*XN'<0),2);
	%W=W/Ws;
	%NR = size(find(W'*XN'>0),2); %non-feasible|right decision
	%NF = size(find(W'*XN'<0),2); %non-feasible|wrong decision
	%FF = size(find(W'*XY'>0),2); %feasible|wrong decision
	%FR = size(find(W'*XY'<0),2); %feasible|right decision
	%J =  - size( find(W'*XY'/Ws > 0), 2 )
	%J = -sum( (W'*XN').^2);
	%J =  (sum( (W'*XY').^2) + sum( (W'*XN').^2));
	%J =  size( (W'*XN').^2);
end
