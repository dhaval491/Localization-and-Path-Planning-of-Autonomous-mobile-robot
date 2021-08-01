% clear all
% clc

# Author Dhaval Patel,
# Verified by Trushar Shah

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

% [c1,r1,p1] = impixel(im1);
% [c2,r2,p2] = impixel(im1);

% For manual source and destination selection.
% c1=49;
% r1=261;
% 
% c2=821;
% r2=222;

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

%             rmax=sqrt((r2-r1).^2+(c2-c1).^2);
%             theta = atan(m);
%             for r=1:rmax
%                 i1=ceil(r*cos(theta));
%                 j1=ceil(r*sin(theta));
%                 im3(i1,j1)=1;
%             end
            
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


level(1,1)=1;
[slx,sl]=size(level);
nodes(1,1)=1;
chk=1;
ptr=1;

%Code to be optimize.
while(sl<=10)
    [snx,sny]=size(nodes);
    level(1,sl)=sny;
    sl=sl+1;
    for i=1:sny
        [x,y]=find(dist(nodes(1,i),:) ~= Inf);
        y
        [syx,syy]=size(y);
        for j=1:syy
            ans=find(nodes==y(j));
            if(isempty(ans)==1)
                nodes(1,ptr+1)=y(j);
                ptr=ptr+1;
            end
            ans=find(nodes==ns);
            if(isempty(ans)~=1)
                break;
            end
        end
        ans=find(nodes==ns);
        if(isempty(ans)~=1)
            break;
        end
    end
    ans=find(nodes==ns);
    if(isempty(ans)~=1)
        break;
    end
end
[snx,sny]=size(nodes);
level(1,sl)=sny;

[slx,sly]=size(level);

nnodes(1,1)=1;
for i=1:sly-1
    l=level(i+1)-level(i);
    nnodes(i+1,1:l)=nodes(1,level(i)+1:level(i+1));
end

path(1,1)=ns;
path(1,sly)=1;
for i=2:sly-1
    [snnx,snny]=size(nnodes(sly-i+1,:));
    mind=Inf;
    for j=1:snny
        if(nnodes(sly-i+1,j)==0)
            continue;
        end
        d=dist(path(1,i-1),nnodes(sly-i+1,j));
        if(mind>d)
            mind=d;
            snode = nnodes(sly-i+1,j);
        end
    end
    path(1,i)=snode;
end

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
toc
[pathx,pathy]= size(path);
sum=0;
for i=1:pathy-1
    sum=sum+dist(path(1,i),path(1,i+1));
end
    
