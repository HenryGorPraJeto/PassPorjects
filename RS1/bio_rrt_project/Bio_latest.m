clear all;
close all;

resX=0.1;
resY=0.1;


try
    rosnode list
catch exp   % Error from rosnode list
    rosinit  % only if error: rosinit
end

%Read map
startingpose=[];
goalpose=[];

laser = rossubscriber('/robot_0/base_scan');
pause(1);

robotpose = rossubscriber('/robot_0/odom');
pause(1);

robotpose2=rossubscriber('/robot_1/odom');
goalpose=robotpose2.LatestMessage;
pause(1);

obchecksub=rossubscriber('/obstaclecheck');




% The below obtains a map image
map = rossubscriber('/map_image/full');
pause(1.0);
map_msg=[];
map1_msg=[];
while(size(map_msg,1)==0)
   map_msg = map.LatestMessage;
   map1_msg=map.LatestMessage.Data;
   pause(0.2);
end

% Read the map image
map1=map1_msg;
map = readImage(map_msg);
figure1=imshow(map);
axis on;
hold on

osx=size(map,1)/2;
osy=size(map,2)/2;




% Define starting pose
startingpoint=robotpose.LatestMessage;
point0=[startingpoint.Pose.Pose.Position.X startingpoint.Pose.Pose.Position.Y];

startingpose=[osx osy];
plot(startingpose(1),startingpose(2),'ro');


startingposition=startingpose; 
plot(startingposition(1),startingposition(2),'r+');
pause(2);


%Define goal pose
endingpoint=robotpose2.LatestMessage;
point2=[endingpoint.Pose.Pose.Position.X endingpoint.Pose.Pose.Position.Y];
point3=[point2(1)-point0(1) point2(2)-point0(2)];% convert global coordinate of ending pose to local coordinate 
endingpose=global2image(point3(1),point3(2),0.1,0.1,osx,osy); % convert local pose to image pose
plot(endingpose(1),endingpose(2),'k+');


endingposition=endingpose 
plot(endingposition(1),endingposition(2),'k+');
pause(2);

if point3(1)>500 || point3(2)>500
    display ('goal pose outside map');
    return
end


goalposeFScheck=feasiblePoint(point3,map);
if goalposeFScheck==true
    display('goal pose not in free space');
    return
end

%ini TREE 1
T.v(1).x =startingposition(1);         
T.v(1).y = startingposition(2); 
T.v(1).xPrev = startingposition(1);    
T.v(1).yPrev = startingposition(2);
T.v(1).dist=0;          
T.v(1).indPrev = 0;
newnode11=startingpose;

%ini TREE 2
T2.v2(1).x2 =endingposition(1);         
T2.v2(1).y2 = endingposition(2); 
T2.v2(1).xPrev2 = endingposition(1);    
T2.v2(1).yPrev2 = endingposition(2);
T2.v2(1).dist2=0;          
T2.v2(1).indPrev2 = 0;
newnode22=endingpose;


stepping=30;
iteration = 10000;

count=1;
newnodeindex=count;
count2=1;
newnodeindex2=count2;

connected=false;

while connected==false


 xrandom=randi([0 osx*2]);
 yrandom=randi([0 osy*2]);
 
 %   step 1 generate random configuration 
 dist =zeros(length(T),1);

     for i =1: count
        dist(i) = (T.v(i).x- xrandom)^2 + (T.v(i).y - yrandom)^2;
        [value, location] = min(dist);
        x_near = [T.v(location).x,T.v(location).y];
     end
  
     deltaxsq=(xrandom-x_near(1))^2;
     deltaysq=(yrandom-x_near(2))^2; 
     c=sqrt(deltaxsq+deltaysq);

     unitx=(xrandom-x_near(1))/c;
     unity=(yrandom-x_near(2))/c;
     newnode=[x_near(1)+stepping * unitx, x_near(2)+ stepping * unity];  
     new_FScheck=feasiblePoint(newnode,map);
     
     if new_FScheck==false
         
         continue
     end
     
     

     
     % step 2 generate new node along the line (nearesr node and randomconfiguration)
     % using unit vector 
     
     if collisionChecking(newnode,x_near,map)==true
         newnode11=newnode;
             count=count+1;
             newnodeindex =count;
             T.v(newnodeindex).x = newnode11(1);         
             T.v(newnodeindex).y = newnode11(2); 
             T.v(newnodeindex).xPrev = x_near(1);     
             T.v(newnodeindex).yPrev = x_near(2);
             T.v(newnodeindex).dist=sqrt((newnode11(1)-x_near(1))^2 + (newnode11(2)-x_near(2))^2);          
             T.v(newnodeindex).indPrev = T.v(newnodeindex-1).indPrev + 1;
             plot(newnode11(1),newnode11(2),'ro');
             pause(0.5);
             plot([x_near(1),newnode11(1)],[x_near(2),newnode11(2)],'-r');       
     end
     
     
     disx_n2T2=zeros(length(T2),1);
     for j2=1:count2
         disx_n2T2(j2)=(  sqrt((newnode11(1)-T2.v2(j2).x2)^2+(newnode11(2)-T2.v2(j2).y2)^2)  );
         
         if disx_n2T2(j2)<=stepping && collisionChecking(newnode11,[T2.v2(j2).x2,T2.v2(j2).y2],map)==true
             pause(0.5);
             plot([T2.v2(j2).x2,newnode11(1)],[T2.v2(j2).y2,newnode11(2)],'-r');
             
             %push back t2's nearest node to T1 easier to get path 
             
             T.v(newnodeindex+1).x = T2.v2(j2).x2;         
             T.v(newnodeindex+1).y = T2.v2(j2).y2; 
             T.v(newnodeindex+1).xPrev = newnode11(1);     
             T.v(newnodeindex+1).yPrev = newnode11(2);
             T.v(newnodeindex+1).dist=sqrt((newnode11(1)-T2.v2(j2).x2)^2 + (newnode11(2)-T2.v2(j2).y2)^2);          
             T.v(newnodeindex+1).indPrev = T.v(newnodeindex+1-1).indPrev + 1;
             newnodeindex=newnodeindex+1;
             
             
             T2.v2(newnodeindex2+1).x2 = T2.v2(j2).x2;         
             T2.v2(newnodeindex2+1).y2 = T2.v2(j2).y2; 
             T2.v2(newnodeindex2+1).xPrev2 = T2.v2(j2).xPrev2;     
             T2.v2(newnodeindex2+1).yPrev2 = T2.v2(j2).yPrev2;
             T2.v2(newnodeindex2+1).dist2=T2.v2(j2).dist2          
             T2.v2(newnodeindex2+1).indPrev2 = T2.v2(j2+1).indPrev2;
             newnodeindex2=newnodeindex2+1;

             % push back finish 
             
             
             
             display('+++++close enough ++++++');
             pause(0.5);
             connected=true;
             pause(0.5);
         end
     if connected==true
         break
     end
 
     end
     
     if connected==true
         break
     end
     
     
% Tree 2 
     
     
%       x2random=randi([0 osx*2]);
%       y2random=randi([0 osy*2]); 
      
%        xrandom=randi([0 osx*2]);
%        yrandom=randi([0 osy*2]);
      x2random=xrandom;
      y2random=yrandom;

      dist2=zeros(length(T2),1);

     for j = 1: count2
        dist2(j) = (T2.v2(j).x2- x2random)^2 + (T2.v2(j).y2 - y2random)^2;
        [value2, location2] = min(dist2);
        x2_near = [T2.v2(location2).x2,T2.v2(location2).y2];
     end
     
     deltaxsq2=(x2random-x2_near(1))^2;
     deltaysq2=(y2random-x2_near(2))^2;
     c2=sqrt(deltaxsq2+deltaysq2);
     
     unitx2=(x2random-x2_near(1))/c2;
     unity2=(y2random-x2_near(2))/c2;
     newnode2=[x2_near(1)+stepping * unitx2, x2_near(2)+ stepping * unity2];
     new_FScheck2=feasiblePoint(newnode2,map);
     
     if new_FScheck2==false
         
         continue
     end
     
     
          if collisionChecking(newnode2,x2_near,map)==true 
              newnode22=newnode2;
             count2=count2+1;
             newnodeindex2 =count2;
             T2.v2(newnodeindex2).x2 = newnode22(1);         
             T2.v2(newnodeindex2).y2 = newnode22(2); 
             T2.v2(newnodeindex2).xPrev2 = x2_near(1);     
             T2.v2(newnodeindex2).yPrev2 = x2_near(2);
             T2.v2(newnodeindex2).dist2=sqrt((newnode22(1)-x2_near(1))^2 + (newnode22(2)-x2_near(2))^2);          
             T2.v2(newnodeindex2).indPrev2 = T2.v2(newnodeindex2-1).indPrev2 + 1;
             plot(newnode22(1),newnode22(2),'bo');
             pause(0.5);
             plot([x2_near(1),newnode22(1)],[x2_near(2),newnode22(2)],'-b');       
          end
          
          
               disx_n2T=zeros(length(T),1);
     for i2=1:count
         disx_n2T(i2)=(  sqrt((newnode22(1)-T.v(i2).x)^2+(newnode22(2)-T.v(i2).y)^2)  );
         
         if disx_n2T(i2)<=stepping && collisionChecking(newnode22,[T.v(i2).x,T.v(i2).y],map)==true
             
             pause(0.5);
             plot([T.v(i2).x,newnode22(1)],[T.v(i2).y,newnode22(2)],'-b');
             %push back information 
             
             T2.v2(newnodeindex2+1).x2 = T.v(i2).x;         
             T2.v2(newnodeindex2+1).y2 = T.v(i2).y; 
             T2.v2(newnodeindex2+1).xPrev2 = newnode22(1);     
             T2.v2(newnodeindex2+1).yPrev2 = newnode22(2);
             T2.v2(newnodeindex2+1).dist2=sqrt((newnode22(1)-T.v(i2).x)^2 + (newnode22(2)-T.v(i2).y)^2);          
             T2.v2(newnodeindex2+1).indPrev2 = T2.v2(newnodeindex2+1-1).indPrev2 + 1;
             newnodeindex2=newnodeindex2+1;
             
             
             T.v(newnodeindex+1).x = T.v(i2).x;         
             T.v(newnodeindex+1).y = T.v(i2).y; 
             T.v(newnodeindex+1).xPrev = T.v(i2).xPrev;     
             T.v(newnodeindex+1).yPrev = T.v(i2).yPrev;
             T.v(newnodeindex+1).dist=T.v(i2).dist;     
             T.v(newnodeindex+1).indPrev = T.v(newnodeindex-1).indPrev + 1;
             newnodeindex=newnodeindex+1;
             
             
             
             
             
             %push back finish 
             
             
             
             
             
             
             pause(0.5);
             display('+++++close enough ++++++');
             pause(0.5);
             connected=true;
         end
         
     if connected==true
         break
     end
 
     end

end


path={};
foundpath = false;
nodenum=newnodeindex;

nodeonpath=[T.v(newnodeindex).x,T.v(newnodeindex).y];
path{end+1}=nodeonpath;

childrennode=[T.v(newnodeindex).x,T.v(newnodeindex).y];
parentnode=[];

while foundpath==false
   
for q=1:newnodeindex
    
    if T.v(q).x==childrennode(1) && T.v(q).y==childrennode(2)
        
        parentnode=[T.v(q).xPrev,T.v(q).yPrev];
        path{end+1}=parentnode;
%         display (parentnode(1));
%         display(parentnode(2));
%         display(T.v(i).indPrev);
        
        plot([childrennode(1),parentnode(1)],[childrennode(2),parentnode(2)],'-g');
        pause(1);
        
        childrennode=parentnode;
        
        if  childrennode==startingposition
            foundpath=true;
            break
        end
        
    end
    
end

end

pathT1=flip(path);




path2={};
foundpath2 = false;
nodenum2=newnodeindex2;

nodeonpath2=[T2.v2(newnodeindex2).x2,T2.v2(newnodeindex2).y2];
path2{end+1}=nodeonpath2;

childrennode2=[T2.v2(newnodeindex2).x2,T2.v2(newnodeindex2).y2];
parentnode2=[];




while foundpath2==false
   
for q2=1:newnodeindex2
    
    if T2.v2(q2).x2==childrennode2(1) && T2.v2(q2).y2==childrennode2(2)
        
        parentnode2=[T2.v2(q2).xPrev2,T2.v2(q2).yPrev2];
        path2{end+1}=parentnode2;
%         display (parentnode(1));
%         display(parentnode(2));
%         display(T.v(i).indPrev);
        
        plot([childrennode2(1),parentnode2(1)],[childrennode2(2),parentnode2(2)],'-g');
        pause(1);
        
        childrennode2=parentnode2;
        
        if  childrennode2==endingposition
            foundpath2=true;
            break
        end
        
    end
    
    
end

end

pathT2=path2;

pathWhole=pathT1;

[bbb , sizeofPathT2]=size(pathT2);


for i3=1:sizeofPathT2
    
    unname=pathT2(i3);
    point_T=cell2mat(unname);
    
    pathWhole{end+1}=[point_T(1),point_T(2)];
    
    
end


[lol pathindex]=size(pathWhole);
xarrayrever=[];
yarrayrever=[];

for i=1:pathindex
    
    checkpoint=pathWhole(i);
    c2m=cell2mat(checkpoint);

    localcor=image2global(c2m(1),c2m(2),resX,resY,osx,osy);
    globalcor=[localcor(1)+point0(1), localcor(2)+point0(2)];
    

    xarrayrever(i)=globalcor(1);
    yarrayrever(i)=globalcor(2);
    
    
end

xarraypub=rospublisher('xarray','std_msgs/Float64MultiArray');
pause(2);
xarraymsg=rosmessage(xarraypub);
xarraymsg.Data=xarrayrever;
send(xarraypub,xarraymsg);
pause(2);

yarraypub=rospublisher('yarray','std_msgs/Float64MultiArray');
pause(2);
yarraymsg=rosmessage(yarraypub);
yarraymsg.Data=yarrayrever;
send(yarraypub,yarraymsg);
pause(2);


display('RRT loop finish');









