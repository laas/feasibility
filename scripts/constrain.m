function [c,ceq]=constrain(W,XN)
	Ws = sqrt(W(1)*W(1) + W(2)*W(2) + W(3)*W(3));
	NF = size(find(W'*XN'<0),2); %non-feasible|wrong decision
	%ceq= NF;
	ceq = [];
	c = max( W'*XN' );
end
