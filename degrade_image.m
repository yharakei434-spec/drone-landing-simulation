function im=degrade_image(im,condition,t,seed)
% Pixel manipulation only. No estimator access to scenario/severity labels.
if t<8 || t>=18,return;end
switch condition
 case 'blur',im=imfilter(im,fspecial('motion',7,20),'replicate');
 case 'dark',im=uint8(.18*double(im)+4*randn(RandStream('mt19937ar','Seed',seed+90000),size(im)));
 case 'occlusion',im(round(end*.38):round(end*.58),round(end*.46):round(end*.54))=uint8(100);
end
end
