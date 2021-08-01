clear all
clc

# Author Trushar Shah, Dhaval Patel
%im1 - original image and its processing.
%im2 - image with check points.
%im3 - image joining two points on line for test.
%im4 - Testing for obstacle/s.

%h -filter
%i - index          %j - index
%m - size           %n - size
%ns - number of cities.

im1=imread('C:\Users\Dhaval\Pictures\Camera Roll\field.png');
im1=rgb2gray(im1);
im1=im2bw(im1);
im1=1-im1;
[m,n]=size(im1);

[L, num] = bwlabel(im1, 4);
b=zeros(m,n);
im2=b;

for i=1:num
    [x,y]=find(L==i);
    x1=min(x)-2;
    y1=min(y)-2;
    x2=max(x)+2;
    y2=max(y)+2;
   
    im2(x1,y1)=1;
    im2(x2,y2)=1;
    im2(x1,y2)=1;
    im2(x2,y1)=1;
 end

[c1,r1,p1] = impixel(im1);
[c2,r2,p2] = impixel(im1);

% c1=946;
% r1=327;
% c2=397;
% r2=182;

close all
tic
%A1 Assumption : All points are distinct.
[im2L,num] = bwlabel(im2,4);
ns=num+2;
co_or=zeros(ns,2);
co_or(1,1)=r1;
co_or(1,2)=c1;
co_or(ns,1)=r2;
co_or(ns,2)=c2;

for i=2:ns-1;
    [x,y]=find(im2L==i-1);
    co_or(i,:)=[x,y];
end

dist=zeros(ns,ns);
dist(:,:)=Inf;
im3=im1;

% Note : Following code should be optimize.
for i=1:ns
    for j=i+1:ns
        im3(:,:)=0;
%         if(i~=j)
            r1=co_or(i,1);
            c1=co_or(i,2);
            r2=co_or(j,1);
            c2=co_or(j,2);     
                        
            m=(r2-r1)./(c2-c1);
            co=r2-m*c2;

            for j1=c1:sign(c2-c1):c2
                i1=round(m*j1+co);
                im3(i1,j1)=1;
            end
            
            im4=im3 & im1;
            if(i==1 && j==1)
                imshow(im4);
            end
            count1=sum(sum(im4));
            
            if(count1~=0)
                dist(i,j)=Inf;
            else
                dist(i,j)=sqrt((r1-r2).^2 + (c1-c2).^2);
            end           
%         end
    end
end

[sdx,sdy]=size(dist);
for i=1:sdx
    for j=1:sdy
        if(j<=i)
            dist(i,j)=dist(j,i);
        end
    end
end

count=1;
j=1;
while(j~=ns)
    j
    path(count)=j;
    t=dist(j,:)

    [j1,i1]=find(t==min(t));
    chk = find(path==i1);
    while 1
        [j1,i1]=find(t==min(t));
        chk = find(path==i1)
        if(isempty(chk)==1)
            j=i1;
            break;         
        else
            t(j1,i1)=Inf;
        end
    end
            count=count+1;
end
    
    

[szx,szy] = size(j);
if(szy>1)
    j=ns;
    path(count)=j;
else
    path(count)=j;
end


% if(j~=ns)
%     path(count)=j;
% else
%      path(count)=ns;
% end

[m1,n1]=size(path);
im3=im1;
im3(:,:)=0;
for ind=1:n1
           if((ind+1)<=n1)
            i=path(ind);
            
            j=path(ind+1)
            r1=co_or(i,1);
            c1=co_or(i,2);
            r2=co_or(j,1);
            c2=co_or(j,2);     
                        
            m=(r2-r1)./(c2-c1);
            co=r2-m*c2;

            for j1=c1:sign(c2-c1):c2
                i1=round(m*j1+co);
                im1(i1,j1)=1;
            end
           end
end
figure
imshow(im1)

[pathx,pathy]= size(path);
toc
sum=0;
for i=1:pathy-1
    if(path(1,i)~=path(1,i+1))
        sum=sum+dist(path(1,i),path(1,i+1));
    end
end               
