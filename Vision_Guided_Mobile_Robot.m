% 
% Project Name: <Vision Guided Autonomous Mobile Robot In Static Environment>
% Author: 	<Dhaval Patel> 
% 
% Filename: 	<Path Planning.m>
% 	
% Functions: 	<getrect,editnode,orderofhit,Straightline,overcome1ob,angle,anglematrix,transform_path_to_string,Sendstring>
% 	
%  Global Variables:	<b,url,im,imCoin,areas,Centroid,major,minor,O,R,l,ox,oy,Ox,Oy,sx,sy,dx,dy,cx,PATH,Angle,D> 
% 			

%--------Camera calibration process------%

[imagePoints, boardSize] = detectCheckerboardPoints('C:\Users\Dhaval\Desktop\New folder\p.jpg');
% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 89.5;              %declaring squaresize on checkerboard pattern in millimeters%
worldPoints = generateCheckerboardPoints(boardSize, squareSize); %creating worldpoints according to squaresize in checkerboard%
[Rx, tx] = extrinsics(imagePoints, worldPoints, cameraParams);  % Compute rotation and translation of the camera.

%-------------Image processing, Obstacle detection, Destination input-------------------%

url = 'http://192.168.1.2:8080/photo.jpg'; % declaring url of IPwebcam app that uses phone camera to capture image of the environment%
im=imread(url); % reading the image from IP Webcam and storing it into a variable%
 imHSV = rgb2hsv(im); % Get the saturation channel for segmentation&
saturation = imHSV(:, :, 2); % saturating the image%
t = graythresh(saturation);  % Threshold the image
imCoin = (saturation > t);
k=90000; % declaring a value for minimum area to be considered as obstacle connecting 0's in binary image after segementation
         % the value is so choosen the area of robot is less than k so that
         % robot cannot be detected as object. From this we can note that
         % obstacles that have greater area than robot are only detected
         
areas=0;Centroid=0;major=0;minor=0;O=0;R=0;l=0; % initializing blob Analysis variables%
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,... % this will set a port to generate a matrix with all filtered out areas of connected points 
    'CentroidOutputPort', true,... % this will set a port that will generate a matrix with pixel coordinates of centroid of areas 
    'MajorAxisLengthOutputPort',true,...% this will set a port to generate a matrix with lengths of major axis of all areas
    'MinorAxisLengthOutputPort',true,...% this will set a port to generate a matrix with lengths of minor axis of all areas
    'BoundingBoxOutputPort', true,...   % this will set a port to generate a matrix with bounding boxes that surrounds obstacles' areas
    'OrientationOutputPort',true,...    % this will set a port to generate a matrix that will determine the angular orientations of major axis of all areas
    'EquivalentDiameterSquaredOutputPort',true,... % this will set a port to generate a matrix that determins equivalent daimeter of all the areas(if needed)
    'LabelMatrixOutputPort',true,...    % this will set a port to generate a matrix that will label all the selected areas
    'MinimumBlobArea', k, 'ExcludeBorderBlobs', true); % this will set a port to generate a matrix that will let us to choose areas greater than k value defined above
    
[areas,Centroid,boxes,major,minor,O,R,l] = step(blobAnalysis, imCoin); % this will detect areas of connected point and will output all the parameters that are required
[~, idx] = sort(areas, 'Descend');  % Sort connected components in descending order by area
imshow(im); % display the picture of environment taken %
hold on
P=0;ox=0;oy=0;Ox=0;Oy=0; % Initializing detected obstacles polygons matrix that contains x and y cordinates 
                         % of corners surrounding obsacles
clear ox oy Ox Oy        % Formating the above declared variables

% 
%     Function Name: getrect
%     Input:		 Angular Orientation,x coordinate of Centroid, y coordinate of centroid, major axis lenght,
%                    minor axis length, of jth area
%     Output:		 Pixel Coordinates of rectangle that surrounds areas and plots all cordinates 
%                    around a given obstacle area
%     Logic:		 Using geometry and concepts of straight lines, from
%                    the values of orientation of major axis of a area and
%                    length of major and minor axis, this function
%                    determines the coordinates of corner points
%                    surrounding the area. The corner points are located in such a way that 
%                    there is enough space for robot so that while robot is
%                    on this node(pixel) it can rotate ad move withut
%                    colliding the obstacle
%     Example Call:  getrect(O,Centroid(j,1),Centroid(j,2)],major(j,1),minor(j,1));
%	
for j=1:size(areas,1) % loop to get two matrices one containing x coordinates of all corners of all areas
                      % j values are till the total number of areas
                      % filtered
    [P]= getrect(O(j,1),[Centroid(j,1),Centroid(j,2)],major(j,1),minor(j,1)); % temporary polygon matrix to store pixel coordinates of 
                                                                              % corners current chosen jth area
    ox(:,j)=P(:,1); % matrix to store x coordinates in jth column of jth area of matrix ox
    oy(:,j)=P(:,2); % matrix to store y coordinates in jth column of jth area of matrix oy

end
for i=1:size(ox,2) % loop to lable all the columns of matrices ox and oy according to lable given by blob analysis 
    ox(5,i)=i;
    oy(5,i)=i;
end
% At this point all x and y coordinates of the corners surrounding
% obstacles' areas are obtained in ox and oy matrices

% 
%     Function Name: editnode
%     Input:		 x and y coordinates of a pixel to be shifted away from
%                    centre of the chosen area, matrix of corner ponts that
%                    cointains the pixel to be shifted.
%     Output:		 x and y coordinates of pixel that is shifted away from
%                    the centroid of the given area by a given pixel length
%     Logic:		 for a chosen pixel (that is a corner point of rectangle 
%                    surrounding a given area) to be shifted away from the  
%                    centre a given area,the diagonally opposite
%                    pixel is taken and a straight line is determined  usig 
%                    this two points. On this sraight line by using geometry 
%                    a pixel is evaluated that is a given distance from 
%                    the chosen pixel(node) away from the centre.
%     Example Call:  getrect(O,Centroid(j,1),Centroid(j,2)],major(j,1),minor(j,1));
%	

for i=1:size(ox,2) % loop to edit corner nodes of all n obstacles' rectangles 
    for j=1:4
        [Ox(j,i),Oy(j,i)]=editnode(ox(j,i),oy(j,i),[ox(1:4,i) oy(1:4,i)]);% [Ox(i,j),Oy(i,j)]=editnode(ox(i,j),oy(i,j),[ox(1:4,i) oy(1:4,i)]);
    end
    Ox(5,i)=i; % Ox and Oy matrix are similar to ox and oy matrix but difference is coordinates are edited
    Oy(5,i)=i; % labling all the columns of Ox and Oy with their respective rectangle lables according to blob analysis
end
hold on
sceneImage=im; % storing image variable into another variable for detecting robot in the image
cx=detectCheckerboardPoints(sceneImage); % matrix that detects the checkerboard points from che
%                                          checkerboard that is mounted on robot to detect robot                                                                                       
sx=cx(1,1);  % first detected corner of the matrix is chosen which is on the centre of robot
sy=cx(1,2);  % sx and sy are coordinates of centre points detected on robot
plot(sx,sy,'r*') % plots the centre of robot on the image
[dx,dy,Z] = impixel();% destination is selected using mouse click on the image , where robot is to be localized
hold on
plot(dx,dy,'r*')% plot the destination on the image



%----------------------Calculating Shortest Path---------------------%

% 
%     Function Name: Straightline
%     Input:		 pixel coordinates of two points by which Straight line
%                    is to evaluated
%     Output:		 pixel coordinates of two points on image bewteen whom straight line is to be generated 
%     Logic:		 Finding the slope of straight line using two point
%                    form and using bresenham algorith points on straight
%                    line between the two points are obtained in a matrix.
%     Example Call:  Straightline(y2,y1,x2,x1);
%	
[X,Y]=Straightline(sy,dy,sx,dx); % generating a straight line matrix from source to destination to
                                 % determine which obstacles are coming in
                                 % way of robot from its current position
                                 % to given destination as seen by robot.

% 
%     Function Name: orderofhit
%     Purpose:       To determine the order of obstacles that are coming in
%                    way of robot to reach its destination
%     Input:		 Coordinates on straightline , matrices containing
%                    x and y coordinates of all the selected object areas
%     Output:		 pixel coordinates of two points on image bewteen whom straight line is to be generated 
%     Logic:		 First set of coordinates on straightline containing two selected points are taken 
%                    Now each polygon coordinates(or say one polygon at a time) from ox and oy are taken
%                    and one cordinate at a time is checked if present in taken polygon. If present, then sequence 
%                    number of that coorinate and lable of that polygon is also stored in a n*2 matrix.
%                    now that we have sequence number of every straightline coordinate alongside the lable of polygon 
%                    in which it is present the sequcence numbers are sorted in a matrix alongside the obstacle lable in
%                    descending order. The row of this matrix containing sorted lables of polygon is obtained in the order as                   
%                    seen from initial point to final point.
%                    
%     Example Call:  orderofhit(X,Y,ox,oy);    (X and Y are matrices with x and y coordinates of points on straight line 
%                    between two points from where order of hit of polygons that come in the way between two points; ox 
%                    and ox are matices that contain x and y coordinates of rectangles surrounding the obstacles)
%


%     Function Name: overcome1ob
%     Purpose:       To find a temporary path when one obstacle is between two points and give a node(corner point on rectangle surrounding
%                    that obstacle)on this path that is closest to destination. this helps to find the temporary destination path
%                    for a given ts(temporary source taken) and next ts.
%                    
%     Input:		 Two points between whom there is one obstacle, and the
%                    corner points of rectangle(matrix form) that surrounds chosen obstacle
%     Output:		 temporary shortest path(matrix form)between two points avoiding the obstacle that comes in between
%     Logic:		 Using the cases proposed in document of proposed idea a path is derived and a node nearest to temporary destination is
%                    obtained
%     Example Call:  overcome1ob(x1,y1,x2,y2,Obstacle); (x1,y and x2,y2 are pixel coordinates of the two selected points
%                    and Obstacle is a matrix containing coordinates of corners surrounding the obstacle)                      
%


[A]=orderofhit(X,Y,ox,oy); % finding the order of obstacles that come in the way from robot's source to destination
tic
tsx=0;tsy=0;               % initializing temporary source's x and y coordinates
tsx(1,1)=sx;tsy(1,1)=sy;   % At first temporary source is the robot's initial position

%-------- the next loop is the main loop to find shortest path coordinates-------%
% In this loop on every iteration there is addition of next coordinate on the path from source to destination
% Every next node found is added to matrices tsx and tsy.
for ii=1:10 % here as processes is iteration independent we have chosen maximum 10 iterations
    X=0;Y=0;% declaring straight line coordinates matrices between temporary source and temporary destination point
    [X,Y]=Straightline(dy,tsy(size(tsy,1),1),dx,tsx(size(tsx,1),1)); % generating matrices for x and y coordinates 
    %                                                                % on straightline(made between temporary source ts and 
    %                                                                 temporary destination td)points between ts(temporary source 
    %                                                                                                                                                                                                                                      
    [a]=orderofhit(X,Y,ox,oy); % now finding the order of obstacles coming in the way between ts and td
    if a==0 % condition to be checked and executed every time if no obstacle is there 
        tsx(size(tsx,1)+1,1)=dx; % join the current two points td and ts as there is no obstacle in between
        tsy(size(tsy,1)+1,1)=dy;
        break;
    else
        % if there are obstacle in way from td and ts then this loop finds
        % the temporary path from destination to temporary source point
        tdx=0;tdy=0;% declaring the temporary destination coordinates variables after every iteration
        tdx(1,1)=dx;% first temporary destination is always the main destination
        tdy(1,1)=dy;
        % Next is a loop to find temporary path from destination to source 
        % in this loop path is obtained from main destination to temporary
        % source point that is taken at a time
        for j=1:15
            X=0;Y=0; % again to check the order of hit of obstacle from current temporary source to temporary destination
            [X,Y]=Straightline(tsy(size(tsy,1),1),tdy(size(tdy,1),1),tsx(size(tsx,1),1),tdx(size(tdx,1),1)); 
            % making a straight line between above two points
            [A]=orderofhit(X,Y,ox,oy);% Determining the order of hit as seen from current ts to td  
            if A==0  % if no obstacle then order of hit gives zero so we can directly connect both points on current path
                tdx(size(tdx,1)+1,1)=sx; % if no obstacle then connect source to that particular td(temporary destination) point
                tdy(size(tdy,1)+1,1)=sy;
                break;
            else
                i=1;
                [path,node]=ovrcome1ob(tsx(size(tsx,1)),tsy(size(tsy,1)),Px(size(Px,1),1),Py((size(Py,1)),1),[Ox(1:4,A(1,(size(A,2)+1-i)))...
                    Oy(1:4,A(1,size(A,2)+1-i))],[ox(1:4,A(1,(size(A,2)+1-i))) oy(1:4,A(1,size(A,2)+1-i))]);
                % finding temorary path to overcome one obstacle between
                % two points and getting a node on this nearest to the chosen destination 
                tdx(size(tdx,1)+1,1)=node(1,1);% after every loop new nodes are added to connect path between td and ts
                tdy(size(tdy,1)+1,1)=node(1,2);
            end
            plot(tdx(size(tdx,1),1),tdy(size(tdy,1),1),'b*') % ploting current nodes that are on temporary path
        end
        % this is end of loop that derives temporary destination path according to idea proposal document
        % After this loop a path is generated from temporary source to main destination so the first node after the temporary source point on
        % this path is chosen as next temporary source node
        tsx(size(tsx,1)+1,1)=tdx(size(tdx,1)-1,1); % storing the last node found to reach temporary source from td as the next node in ts matrix
        tsy(size(tsy,1)+1,1)=tdy(size(tdy,1)-1,1); 
        % after this loop a new source point or next point for robot to
        % move on is obtained and in next iteration this source is taken as
        % temporary source to reach main destination
    end
end
toc
 line(tsx(:,1),tsy(:,1),'color','b') % ploting path in for of line from main source point to main destination point %
 path(:,2)=tsy(:,1);% story tsx and tsy (now as coordinates of optimum path) into variable named path
 path(:,1)=tsx(:,1);
 D=0; % initializing distance matrix which will give distacnes in cm robot has to travel on its path from on node to another node
for i=1:size(path,1)-1 % finding Distance matrix from camera calibration parameters
    imagePoints1=[path(i,1),path(i,2);path(i+1,1),path(i+1,2)]; % taking two adjecent points on path to find distance betweeen them
    worldPoints1 = pointsToWorld(cameraParams, Rx, tx, imagePoints1); % converting those two points in to coordinates of physical units using
                                                                      % camera parameters obtained by caliberation process             
    D(i,1)=dist(worldPoints1(1,1),worldPoints1(1,2),worldPoints1(2,1),worldPoints1(2,2)); 
    % finds the distance between given two pixel coordinates in cm
end
% all the linear distances the robot has to travel are now obtained. now
% our next task is to find angular displacements to be made by robot
 S=complex(cx(4,1),cx(4,2)); % from the matrix containing detected checkerboard point on robot another 
                             % point next to souce point is selected such that line connecting this to point 
                             % determines the initial orientation of robot from the origin.
                             % This point is converted into complex number to find angular rotation
                             
s=complex(sx,sy); % now source point's coordinates are converted into complex number 
 % reason behind converting all coordinates into complex numbers is that
 % using coni method in complex numbers theory smaller rotation angle between two
 % vectors having common tail is find out. From this we can find angular
 % displacements the robot hast to move at every point on path
 
% 
%     Function Name: anglematrix
%     Input:		 coordinates path and initial orientation of the robot
%     Output:		 Angular displacement valus according to sequence of nodes on the path from Robot's position to Destination 
%     Logic:		 Using coni method formula theta=arctan(z1-z2/z3-z2) an angle of rotation between two vecotors connecting                   
%                    complex pairs z1-z2 and z2-z3 is found out at every node so that rotation of robot at every node can be
%                    found out by taking z2 as given node and z1 and z3 its adjecent nodes on path.                   
%     Example Call:  anglematrix(Path_matrix,initial_angle_of_robot);
%	
alpha=angle((complex(path(2,1),path(2,2))-s)/(S-s))*180/pi; % calculation of initial orientation of robot w.r.t origin
Angle=angelmatrix(path,alpha); % making an angle matrix with alpha as initial orientation of robot
PATH=0; % PATH is a single row matrix that will contain angular and linear displacements robot has to traversed 
k=1; % initializing two index variables for next loop to merge angles(Angle matrix) and distances( matris D) into a
     % variable called PATH
m=1;
 for j=1:size(D,1)+size(Angle,1) % size of PATH will be sum of numer of linear and angular displacement for a given 'path' matrix
                                 % All the odd places are filled with  angular displacements while even with linear distances.
                                 
     if(rem(j,2)==1) % condition to begin with angular displacement as robot may not be oriented in direction of first nod to be reached
        PATH(1,j)=Angle(k,1); % storing angle values into odd position. +ve angle values means robot moves in clockwise(right) sense 
                              % as seen from top,while -ve is for anticlockwise(left) sense.                          
        k=k+1;
     end
     if(rem(j,2)==0) % storing distance value into even position
         PATH(1,j)=D(m,1)/10;
         m=m+1;
     end
 end
 % At this point PATH contains all the angular displacements ad linear
 % distaces it has to traversed in order to reach the destination
 
%
% Function Name:     transform_path_to_string
%     Input:		 path matrix
%     Output:		 n(number of path elements) cross four matrix that has been obtained such that for one motion(angular or linear) at a time
%                    each row of this matrix contains four columns. This matrix has number of rows equal to total number of
%                    displacements robot has to traversed.
%     Logic:		 For communication to be perfect one row for one motion is created 
%                    First element of each row has a fixed value indicating
%                    robot that data is sent from this hardware. Second cloumn contains magnitude of angle element only if
%                    its sign is -ve(left) other wise contains zero.Third cloumn contains magnitude of angle element only if
%                    its sign is +ve(right) other wise contains zero.Fourth column is to store linear distance(forward) to                    
%                    be traversed
%     Example Call:  transform_path_to_string(PATH);
%	
 [Y]=transform_path_to_string(PATH); % transforming PATH into a Y matrix so that data can be send properly with all possible motions
 
 %--------Bluetooth communication----%

b=Bluetooth('dhaval',1);        %Creating a bluetooth object%
fopen(b);                       %Connecting to HC-05 bluetooth module that is mounted on FIREBIRD V%
% 
%     Function Name: Sendstring
%     Input:		 bluetooth object, Transormed PATH matrix
%     Output:		 Data transfer to bluetooth b 
%     Logic:		 Each element of all rows are send asynchronously to connected bluetooth hardware by taking each row order
%                    wise
%     Example Call:  Straightline(y2,y1,x2,x1);
%
Sendstring(b,Y) % sending data to bluetooth hardware b


%----------------------------Function Definitions------------------------%

function [path,node,a,b]=ovrcome1ob(sx,sy,dx,dy,ObPolygon,obPolygon)
[a,b]=validnode(sx,sy,dx,dy,obPolygon,ObPolygon);
if a==0
    node =0;
    path =[sx sy;dx dy];
else
for i=1:size(a,1)
    plot(a(i,1),a(i,2),'g*')
    hold on
end
for i=1:size(b,1)
   plot(b(i,1),b(i,2),'r*')
    hold on
end
p=1;
c=zeros(1,1);
if size(a,1)==3 && size(b,1)==2
    path1=0;
    path2=0;
    for i=1:size(a,1)
        for j=1:size(b,1)
            if b(j,1)==a(i,1)
                c(p,1)=a(i,1);
                c(p,2)=a(i,2);
                c(p,3)=a(i,3);
                p=p+1;
            end
        end
    end
    path1=[sx sy;c(1,1) c(1,2);dx dy];
    d1=0;
    d1=dist(sx,sy,c(1,1),c(1,2)) + dist(c(1,1),c(1,2),dx,dy);
    for i=1:size(b,1)
        if b(i,1)==c(1,1)
            continue;
        else
            d(1,1)= b(i,1);
            d(1,2)= b(i,2);
        end
    end
    s=findnode(d(1,1),d(1,2),ObPolygon);
    for i=1:size(a,1)
        if a(i,1)==s(1,1) || a(i,1) == c(1,1)
            continue;
        else
             k(1,1)=a(i,1);
             k(1,2)=a(i,2);
        end
    end
    path2=[sx sy;d(1,1) d(1,2);k(1,1) k(1,2);dx dy];
    d2=dist(sx,sy,d(1,1),d(1,2))+dist(d(1,1),d(1,2),k(1,1),k(1,2))+dist(k(1,1),k(1,2),dx,dy);
    if d1>d2
        path=path2;
    else
        path=path1;
    end
end
p=1;
c=[0,0];
if size(a,1)==2 && size(b,1)==3
    path1=0;
    path2=0;
    for i=1:size(b,1)
        for j=1:size(a,1)
            if a(j,1)==b(i,1)
                c(p,1)=b(i,1);
                c(p,2)=b(i,2);
                c(p,3)=b(i,3);
                p=p+1;
            end
        end
    end
     path1=[sx sy;c(1,1) c(1,2);dx dy];
    d1=0;
    d1=dist(dx,dy,c(1,1),c(1,2)) + dist(c(1,1),c(1,2),sx,sy);
    for i=1:size(a,1)
        if a(i,1)==c(1,1)
            continue;
        else
            d(1,1)= a(i,1);
            d(1,2)= a(i,2);
        end
    end
     s=findnode(d(1,1),d(1,2),ObPolygon);
    for i=1:size(b,1)
        if b(i,1)==s(1,1) || b(i,1) == c(1,1)
            continue;
        else
             k(1,1)=b(i,1);
             k(1,2)=b(i,2);
        end
    end
    path2=[sx sy;k(1,1) k(1,2);d(1,1) d(1,2);dx dy];
    d2=dist(dx,dy,d(1,1),d(1,2))+dist(d(1,1),d(1,2),k(1,1),k(1,2))+dist(k(1,1),k(1,2),sx,sy);
    if d1>d2
        path=path2;
    else
        path=path1;
    end
end
if size(a,1)==2 && size (b,1)==2
    d1=zeros(1,1);
    for i=1:2;
        d1(i,1)=dist(a(1,1),a(1,2),b(i,1),b(i,2));
        d1(i,2)=a(1,3);
        d1(i,3)=b(i,3);
    end
   [D,I]=min(d1(:,1));
   D1(1,1)=dist(sx,sy,ObPolygon(a(1,3),1),ObPolygon(a(1,3),2))+D+dist(ObPolygon(d1(I,3),1),ObPolygon(d1(I,3),2),dx,dy);
   D1(1,2)=a(1,3);
   D1(1,3)=b(I,3);
   d1=zeros(1,1);
    for i=1:2
        d1(i,1)=dist(a(2,1),a(2,2),b(i,1),b(i,2));
        d1(i,2)=a(2,3);
        d1(i,3)=b(i,3);
    end
    [D,I]=min(d1(:,1));
   D1(2,1)=dist(sx,sy,ObPolygon(a(2,3),1),ObPolygon(a(2,3),2)+D+dist(ObPolygon(d1(I,3),1),ObPolygon(d1(I,3),2),dx,dy));
   D1(2,2)=a(2,3);
   D1(2,3)=b(I,3);
   [D,I]=min(D1(:,1));
   path=[sx sy;ObPolygon(D1(I,3),1) ObPolygon(D1(I,3),2);ObPolygon(D1(I,2),1) ObPolygon(D1(I,2),2);dx dy];
end
if size(a,1)==3 && size(b,1)==3
    d=[0 0];
    p=1;
    for i=1:3
        if a(i,1)==b(1,1)
            d(p,1)=dist(sx,sy,a(i,1),a(i,2))+dist(a(i,1),a(i,2),dx,dy);
            d(p,2)=a(i,3);
            d(p,3)=b(1,3);
            p=p+1;
        end
    end
    p=1;
    for i=1:3
        if a(i,1)==b(2,1)
            d(p,1)=dist(sx,sy,a(i,1),a(i,2))+dist(a(i,1),a(i,2),dx,dy);
            d(p,2)=a(i,3);
            d(p,3)=b(2,3);
            p=p+1;
        end
    end
    p=1;
    for i=1:3
        if a(i,1)==b(3,1)
            d(p,1)=dist(sx,sy,a(i,1),a(i,2))+dist(a(i,1),a(i,2),dx,dy);
            d(p,2)=a(i,3);
            d(p,3)=b(3,3);
        end
    end
    [D,I]=min(d(:,1));
    path=[sx sy;ObPolygon(d(I,3),1) ObPolygon(d(I,3),2);dx dy];
end
%disp(path);
node(1,:)=[path(size(path,1)-1,1) path(size(path,1)-1,2)];
if size(path,1)==4
    node(2,:)=[path(size(path,1)-2,1) path(size(path,1)-2,2)];
end
line(path(:,1),path(:,2),'color','y')
%disp(node);
end
end


function D=distance(path,ss,cameraParams)
[imagePoints, boardSize] = detectCheckerboardPoints('C:\Users\Dhaval\Desktop\MATLAB camera\proj\K.jpg');

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).

squareSize = 90; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%[imagePoints, boardSize] = detectCheckerboardPoints(ss);

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams); 
for i=1:size(path,1)-1
   
    imagePoints1=[path(i,1),path(i,2);path(i+1,1),path(i+1,2)];
    worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
    D(i,1)=dist(worldPoints1(1,1),worldPoints1(1,2),worldPoints1(2,1),worldPoints1(2,2)); 
end
end

function [B]= getrect(Orientation,Centroid,majoraxisLength,minoraxisLength)
O=Orientation;
major=majoraxisLength+50;
minor=minoraxisLength+50;
theta=(O*180)/pi;
major=major*1.15;
minor=minor*1.15;
if theta>0
    P=[Centroid(1,1)+(major*cos(O)/2),Centroid(1,2)-(major*sin(O)/2);...
       Centroid(1,1)-(major*cos(O)/2),Centroid(1,2)+(major*sin(O)/2);...
       Centroid(1,1)+(minor*abs(cos(O+pi/2))/2),Centroid(1,2)+(minor*sin(O+pi/2)/2);...
       Centroid(1,1)-(minor*abs(cos(O+pi/2))/2),Centroid(1,2)-(minor*sin(O+pi/2)/2)];
   B=[P(1,1)+(minor*abs(cos(O+pi/2))/2),P(1,2)+(minor*sin(O+pi/2)/2);...
      P(1,1)-(minor*abs(cos(O+pi/2))/2),P(1,2)-(minor*sin(O+pi/2)/2);...
      P(2,1)+(minor*abs(cos(O+pi/2))/2),P(2,2)+(minor*sin(O+pi/2)/2);...
      P(2,1)-(minor*abs(cos(O+pi/2))/2),P(2,2)-(minor*sin(O+pi/2)/2)];
end
if theta<=0
    
    P=[Centroid(1,1)+(major*cos(O)/2),Centroid(1,2)+(major*abs(sin(O))/2);...
       Centroid(1,1)-(major*cos(O)/2),Centroid(1,2)-(major*abs(sin(O))/2);...
       Centroid(1,1)+(minor*abs(cos(O+pi/2))/2),Centroid(1,2)-(minor*sin(O+pi/2)/2);...
       Centroid(1,1)-(minor*abs(cos(O+pi/2))/2),Centroid(1,2)+(minor*sin(O+pi/2)/2)];
   B=[P(1,1)+(minor*abs(cos(O+pi/2))/2),P(1,2)-(minor*sin(O+pi/2)/2);...
    P(1,1)-(minor*abs(cos(O+pi/2))/2),P(1,2)+(minor*sin(O+pi/2)/2);...
    P(2,1)+(minor*abs(cos(O+pi/2))/2),P(2,2)-(minor*sin(O+pi/2)/2);...
    P(2,1)-(minor*abs(cos(O+pi/2))/2),P(2,2)+(minor*sin(O+pi/2)/2)];
end
for i=1:4
    hold on
    plot(B(i,1),B(i,2),'*');
end
end
     
    
    
function [K]=orderofhit(X,Y,ox,oy)%ob5Polygon)
for i=1:size(ox,2)
In(i,:)=inpolygon(X,Y,ox(1:4,i),oy(1:4,i));
end
p=1;
o=0;
for i=1:size(In,1)
    for j=1:size(In,2)
        if In(i,j)==1
            o(1,p)=j;
            o(2,p)=i;
            p=p+1;
            break;
        end;
    end
end
k=o;
if k==zeros(size(o))
    K=k;
    return
end
A=k(1,:);
[A,I]=sort(A);
for i=1:size(A,2)
    K(1,i)=k(2,I(1,i));
end
% in1=inpolygon(X,Y,ob1Polygon(:,1),ob1Polygon(:,2));
% in2=inpolygon(X,Y,ob2Polygon(:,1),ob2Polygon(:,2));
% in3=inpolygon(X,Y,ob3Polygon(:,1),ob3Polygon(:,2));
% in4=inpolygon(X,Y,ob4Polygon(:,1),ob4Polygon(:,2));
% % in5=inpolygon(X,Y,ob5Polygon(:,1),ob5Polygon(:,2));
% o=0;
% p=1;
% for i=1:size(in1,2)
%     if in1(1,i)==1
%         o(1,p)=i;
%         o(2,p)=ob1Polygon(1,3);
%         p=p+1;
%        break
%     end
% end
% for i=1:size(in2,2)
%     if in2(1,i)==1
%         o(1,p)=i;
%         o(2,p)=ob2Polygon(1,3);
%         p=p+1;
%        break
%     end
% end
% for i=1:size(in3,2)
%     if in3(1,i)==1
%         o(1,p)=i;
%         o(2,p)=ob3Polygon(1,3);
%         p=p+1;
%        break
%     end
% end
% for i=1:size(in4,2)
%     if in4(1,i)==1
%         o(1,p)=i;
%         o(2,p)=ob4Polygon(1,3);
%         p=p+1;
%        break
%     end
% end
% % for i=1:size(in5,2)
% %     if in5(1,i)==1
% %         o(1,p)=i;
% %         o(2,p)=ob5Polygon(1,3);
% %         p=p+1;
% %        break
% %     end
% % end

end


function Sendstring(b,Y)
for i=1:size(Y,1)
    for j=1:4
        disp(Y(i,j));
        if j==4
            p=Y(i,j)*0.05+0.5;
        else
            p=Y(i,j)*(0.023)+0.5;
        end
        fwrite(b,int16(Y(i,j)),'async')
        pause(p);
    end
    
end
end
function [a,b]=validnode(sx,sy,dx,dy,obPolygon,ObPolygon) %for n number of polygon it needs to be changes
a=zeros(1,1);
p=1;
[Xx,Yy]=Straightline(sy,dy,sx,dx);
flag1=ishit(Xx,Yy,obPolygon);
if flag1==0
    a=0;
    b=0;
else
%-finding possible nodes that doest hit object from destination
for i=1:size(ObPolygon,1)
    [X,Y]=Straightline(ObPolygon(i,2),dy,ObPolygon(i,1),dx);
    flag=ishit(X,Y,obPolygon);
    if flag==0
        a(p,1)=ObPolygon(i,1);
        a(p,2)=ObPolygon(i,2);
        a(p,3)=i;
        p=p+1;
    else 
        continue;
    end
end
%--finding possible nodes that doesnt hit object from source---
q=1;
b=zeros(1,1);
for i=1:size(ObPolygon,1)
    [X,Y]=Straightline(ObPolygon(i,2),sy,ObPolygon(i,1),sx);
    flag=ishit(X,Y,obPolygon);
    if flag==0
        b(q,1)=ObPolygon(i,1);
        b(q,2)=ObPolygon(i,2);
        b(q,3)=i;
        q=q+1;
    else 
        continue;
    end
end
% disp(a);
% disp(b);
for i=1:size(a,1)
plot(a(i,1),a(i,2),'b*');
    hold on;
    for i=1:size(b,1)
plot(b(i,1),b(i,2),'r*');
    hold on;
    end
end
end
end

function [y]=transform_path_to_string(X)
Y=0;
for i=1:size(X,2)
    Y(size(Y,1)+1,1)=10;
    if(rem(i,2)==1)
            if(X(1,i)>0)
                    Y(size(Y,1),3)=X(1,i);
            else
                    Y(size(Y,1),2)=(-1)*X(1,i);
            end
    end
    if(rem(i,2)==0)
        if(X(1,i)>255)
            n=floor(X(1,i)/255);
            r=X(1,i)-n*255;
            Y(size(Y,1)+1,1)=10; %20 ni jagya 10 hatu by default
            for j=1:n;
                Y(size(Y,1),4)=255;
                Y(size(Y,1)+1,1)=10; % same here as above
            end
             Y(size(Y,1),4)=r;
            Y(size(Y,1),1)=10; % Y(size(Y,1)+1,1)=10;
           
        else
        Y(size(Y,1),4)=X(1,i);
        end
    end
end
j=1;
for i=1:size(Y,1)
    if((Y(i,:)==[10 0 0 0]) | (Y(i,:)==[0 0 0 0]))
        continue;
    else
        y(j,:)=Y(i,:);
        j=j+1;
    end
end
end
function [X,Y] = Straightline( y1,y2,x1,x2)
Dx=x2-x1;
Dy=y2-y1;
if abs(Dx)>abs(Dy)
    ds=floor(abs(Dx));
else 
    ds=floor(abs(Dy));
end
X(1,1)=x1;
Y(1,1)=y1;
for i=1:floor(ds)
    X(1,i+1)=X(1,i) + (Dx/ds);
    Y(1,i+1)=Y(1,i) + (Dy/ds);
end
end

function showdetectedobjects(scenerealImage,ob1Polygon,ob2Polygon,ob3Polygon,ob4Polygon)%,ob5Polygon)
figure;
imshow(scenerealImage);
hold on;
line(ob4Polygon(:, 1),ob4Polygon(:, 2), 'Color', 'r');
%line(ob5Polygon(:, 1),ob5Polygon(:, 2), 'Color', 'r');
line(ob1Polygon(:, 1),ob1Polygon(:, 2), 'Color', 'r');
line(ob3Polygon(:, 1),ob3Polygon(:, 2), 'Color', 'r');
line(ob2Polygon(:, 1),ob2Polygon(:, 2), 'Color', 'r');
end
function[b]=shiftnode(nodefoundx,nodefoundy,nodex,nodey)
x2=nodex;
y2=nodey;
x1=nodefoundx;
y1=nodefoundy;
Dx=x2-x1;
Dy=y2-y1;
if abs(Dx)>abs(Dy)
    ds=floor(abs(Dx));
else 
    ds=floor(abs(Dy));
end
X(1,1)=x1;
Y(1,1)=y1;
for i=1:floor(ds)
    X(1,i+1)=X(1,i) + (Dx/ds);
    Y(1,i+1)=Y(1,i) + (Dy/ds);
end
X(1,size(X,2))=x2;
Y(1,size(Y,2))=y2;
X(1,1)=x1;
Y(1,1)=y1;
for i=1:size(X,2)
if X(1,i)==nodex
index=i;
end
end
X1=X(1,index)+(Dx/ds);
for i=1:12;
    X1=X1+(Dx/ds);
end
X2=X(1,index)-(Dx/ds);
for i=1:12;
    X2=X2+(Dx/ds);
end
Y1=Y(1,index)+(Dy/ds);
for i=1:12;
    Y1=Y1+(Dy/ds);
end
Y2=Y(1,index)-(Dy/ds);
for i=1:12;
    Y2=Y2+(Dy/ds);
end
d1(1,1)=dist(X1,Y1,nodefoundx,nodefoundy);
d1(2,1)=dist(X2,Y2,nodefoundx,nodefoundy);
d1(1,2)=X1;
d1(1,3)=Y1;
d1(2,2)=X2;
d1(2,3)=Y2;
[D,index]=max(d1(:,1));
b(1,1)=d1(index,2);
b(1,2)=d1(index,3);
end%  need to be changed
function lyne(x1,x2,y1,y2)
m=(y1-y2)/(x1-x2);
x=x1:4:x2;
y=y1+m*x-x1*m;
for I=1:size(x,2)
    hold on
    plot(x(1,I),y(1,I),'g*')
    hold on
end

end
function [f]=ishit(x,y,polygon)
in=inpolygon(x,y,polygon(:,1),polygon(:,2));
f=0;
for i=1:size(in,2)
    if in(1,i)==1
        f=1;
       break
     else f=0;
    end
end
end
function [a]=findnode(nodex,nodey,obPolygon)
p=1;
dst=0;
for i=1:4
    if obPolygon(i,1)==nodex
        continue;
    else
        dst(p,1)=dist(nodex,nodey,obPolygon(i,1),obPolygon(i,2));
        dst(p,2)=i;
        p=p+1;
    end
end
[dmax,I]=max(dst(:,1));
index=dst(I,2);
a(1,1)=obPolygon(index,1);
a(1,2)=obPolygon(index,2);
end


function [a,b]=feasiblenode(validnodex,validnodey,sx,sy,polygon)
d=0;
j=1;
for i=1:size(polygon,1)-1
    if polygon(i,1)==validnodex
        continue;
    else
    d(j,1)=sqrt((validnodex-polygon(i,1))^2+(validnodey-polygon(i,2))^2);
    d(j,2)=i;
    j=j+1;
    end
end
maxd=max(d(:,1));
p=1;
D=0;
for i=1:size(d)
    if d(i,1)==maxd
        continue
    else
        D(p,1)=d(i,1);
        D(p,2)=d(i,2);
        p=p+1;
    end
end

for k=1:size(D,1)
    theta(k,1)=angle2vectors(sx,sy,validnodex,validnodey,polygon(D(k,2),1),polygon(D(k,2),2));
    theta(k,2)=D(k,2);
end
[thetamin,h]=min(theta(:,1));
index=theta(h,2);
a=polygon(index,1);
b=polygon(index,2);
plot(a,b,'r*')
hold on
plot(validnodex,validnodey,'g*')
end

function [x,y]=editnode(nodex,nodey,obPolygon)
[a]=findnode(nodex,nodey,obPolygon);
nodefoundx=a(1,1);
nodefoundy=a(1,2);
N=shiftnode(nodefoundx,nodefoundy,nodex,nodey);
x=N(1,1);
y=N(1,2);
plot(x,y,'g*');
end
function [Dist]=dist(x1,y1,x2,y2)
Dist=sqrt((x1-x2)^2+(y1-y2)^2);
end
function [a]=angle2vectors(sx,sy,dx,dy,nx,ny)
sn=[nx-sx,ny-sy];
sd=[dx-nx,dy-ny];
a=acosd((dot(sn,sd))/((sqrt(dot(sn,sn)))*(sqrt(dot(sd,sd)))));
end



                
                    
          