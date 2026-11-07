function q=image_quality(im,corners)
% Features from pixels alone; neither degradation label nor true pose.
x=max(1,floor(min(corners(:,1)))):min(size(im,2),ceil(max(corners(:,1))));y=max(1,floor(min(corners(:,2)))):min(size(im,1),ceil(max(corners(:,2))));patch=double(im(y,x));s=sort(patch(:));
lo=s(max(1,round(.1*numel(s))));hi=s(max(1,round(.9*numel(s))));contrast=hi-lo;
lap=conv2(patch,[0 1 0;1 -4 1;0 1 0],'valid');sharp=mean(abs(lap(:)))/max(contrast,1)*sqrt(numel(patch));q=[hi/255 contrast sharp];
end
