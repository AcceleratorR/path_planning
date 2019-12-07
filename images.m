clear all
clc
A1 =im2bw(imread('lianjie.jpg'));
% B = [0 1 0 
%       1 1 1
%       0 1 0];
  B = ones(10);
  A2 =bwmorph(~A1,'thicken',inf);
%    A3 = bwmorph(A1,'skel',inf);
%   A4=bwmorph(A1,'thin',inf);
[a,b] = find(A2 ==0);


  figure(1);
  imshow(A1);
  for  i = 1:length(a)
    A1(a(i),b(i)) =0;
  end
  imshow(A1);
%    figure(3);
%   imshow(A3);
%    figure(4);
%   imshow(A4);
  