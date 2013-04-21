clear all;
prefix = '../data/test/';
data = dir(fullfile('.',strcat(prefix,'*tmp')));

Nfiles = size(data,1)

k=1;
fname = strcat(prefix,data(k).name);
A=load(fname);
dist = A(:,5);

%plot(dist, randn(dist, 0.17), 80);

