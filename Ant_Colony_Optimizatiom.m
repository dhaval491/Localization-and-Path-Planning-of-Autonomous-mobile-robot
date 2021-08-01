clear all
clc
# Author : Dhaval Patel, Thrushar Shah


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
tic
% For manual source and destination selection.
% c1=63;
% r1=47;
% 
% c2=1049;
% r2=455;

close all

%A1 Assumption : All points are distinct.
[im2L,num] = bwlabel(im2,4);
ns=num+2;
co_or=zeros(ns,2);
co_or(1,1)=r1;
co_or(1,2)=c1;
co_or(ns,1)=r2;
co_or(ns,2)=c2;

for i=2:ns-1
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

ants=0;
if(ns<=10)
    n=ns-2;
    for i=0:n
        ants=ants+(factorial(n)./factorial(n-i));
    end
    mem = zeros(ants,1);
else
    n=8;
    for i=0:n
        ants=ants+(factorial(n)./factorial(n-i));
    end    
    mem = zeros(ants,1);
end

travel = mem;
i=2;

mem(:,1)=1;
while(i<8) % iterations... Assuming path containing maxmimum 10 nodes
    clear prevpos
%     k=1;
%     for j=1:ants
%         if(j==1)
    tmem = diff(mem(:,i-1));
    if(i~=2)
        pprev=find(tmem);
        prevpos(1,:)=mem(pprev,i-1);
        [psx,psy]=size(prevpos);
        prevpos(1,psy+1)=mem(ants,i-1);
    else
        pprev=ants;
        prevpos(1,:)=mem(1,i-1);
    end

 
%             k=k+1;
%             continue;
%         else
%             ans=find(prevpos==mem(j,i-1));
%             if(isempty(ans)==1)
%                 prevpos(1,k)=mem(j,i-1);
%                 k=k+1;
%             end

%         end
%     end
    [psx,psy]=size(prevpos);
    start = 1;
    px=1;
    for j=1:psy
       if(prevpos(1,j)~=0) 
        [nextposx,nextposy]=find(dist(prevpos(1,j),:)~=Inf);
%         [px,py]=find(mem(:,i-1)==prevpos(1,j));
        if(j~=psy)
            px=max(px)+1:pprev(j);
        else
            px=max(px)+1:ants;
        end
        
        [cantx,canty]=size(nextposy);
        for j1=1:canty
            ans=find(mem(px,1:i-1)==nextposy(1,j1));
            if(isempty(ans)==1)
                nextposy(1,j1)=nextposy(1,j1);
            else
                nextposy(1,j1)=0;
            end
        end
        [cantx1,canty2]=find(nextposy(1,:)==0);
        nextposy(canty2)=[];
        [cantx,canty]=size(nextposy);
        
%         [cantx1,canty2]=find(mem(:,i-1)==prevpos(1,j));
        if(j~=psy && j~=1)
            tants1 = pprev(j)-pprev(j-1);
        elseif(j==1)
            tants1=pprev(j);
        elseif(j==psy)
            tants1=ants-pprev(j-1);
        end
%         [tants1,tants2]=size(canty2);
        inc = floor(tants1./canty);
        endp=start+tants1-1;
        
        for j1=1:canty
            if(j1~=canty)
                ans=find(mem(start:start+inc-1,:)==nextposy(1,j1));
            else
                ans=find(mem(start:endp,:)==nextposy(1,j1));
            end
            
            if(isempty(ans)==1)
                if(prevpos(1,j)~=ns)
                    if(j1~=canty)
                        mem(start:start+inc-1,i)= nextposy(1,j1);
                        start=start+inc;
                    else
                        mem(start:endp,i)=nextposy(1,j1);
                        start=endp+1;                
                    end
                else
                    if(j1~=canty)
                        mem(start:start+inc-1,i)= ns;
                        start=start+inc;
                    else
                        mem(start:endp,i)=ns;
                        start=endp+1;                
                    end
                end
            end
        end
        [x,y]=find(mem(start:tants1,i)==0);
        mem(x,i)=max(mem(:,i));
       end
    end
    i=i+1;
end

[smemx,smemy]=size(mem);

for i=1:ants
    sum=0;
    for j=1:smemy-1
        if(mem(i,j)~=mem(i,j+1))
            if(mem(i,j)~=0 && mem(i,j+1)~=0)
                sum=sum+dist(mem(i,j),mem(i,j+1));
            else
                if(mem(i,:)~=ns)
                    sum=inf;
                end
            end
        end
    end
    travel(i,1)=sum;
end

stop=1;
while(stop~=0)
    [mintra,te]=find(travel==min(travel));
    if(mem(mintra,smemy)==ns)
        stop=0;
    else
        travel(mintra,1)=Inf;
    end
end

path = mem(mintra(1,1),:);
    
  
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

sum=0;
for i=1:pathy-1
    if(path(1,i)~=path(1,i+1))
        sum=sum+dist(path(1,i),path(1,i+1));
    end
end
toc
    
