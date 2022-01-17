
% osx=154.5;
% osy=136.5;
% resX=0.1;
% resY=0.1;

resX=0.1;
resY=0.1;

% The below will only rosinit if needed
% https://au.mathworks.com/matlabcentral/answers/370444-how-do-i-check-the-state-of-ros-master
try
    rosnode list
catch exp   % Error from rosnode list
    rosinit  % only if error: rosinit
end








% rosinit

pathinformationx = rossubscriber('/xarray');
pathinformationy = rossubscriber('/yarray');



pathx=pathinformationx.LatestMessage.Data;
pathy=pathinformationy.LatestMessage.Data;
[elementsX,r]=size(pathx);
[elementsY,rr]=size(pathy);

if elementsX ~= elementsY
    display('error in paasing information on ROS')
end


robotpose2=rossubscriber('/robot_1/odom');
goalposition=robotpose2.LatestMessage;

laser = rossubscriber('/robot_0/base_scan');
pause(1);
robotpose = rossubscriber('/robot_0/odom');
pause(1);

obchecksub=rossubscriber('/obstaclecheck');
startingposition=robotpose.LatestMessage;


map2 = rossubscriber('/map_image/full');
pause(1.0);
map2_msg=[];
while(size(map2_msg,1)==0)
   map2_msg = map2.LatestMessage;
   pause(0.2);
end

% Read the map image
map2 = readImage(map2_msg);
figure2=imshow(map2);
axis on;
hold on



osx=size(map2,1)/2;
osy=size(map2,2)/2;


turningpub = rospublisher('robot_0/cmd_vel', 'geometry_msgs/Twist')
turningmsg=rosmessage(turningpub);


startingpoint=robotpose.LatestMessage;
point0=[startingpoint.Pose.Pose.Position.X startingpoint.Pose.Pose.Position.Y];

 
pathonglobal={};
pathonimage={};
plotcount=2;

for iii=1:elementsX
    
    
pathonglobal{end+1}=[pathx(iii),pathy(iii)];

globalpoint=[pathx(iii),pathy(iii)];

localcoor=[globalpoint(1)-point0(1),globalpoint(2)-point0(2)];


pointonimage=global2image(localcoor(1),localcoor(2),resX,resY,osx,osy);

pathonimage{end+1}=pointonimage;

end

plotdone=false;
firstcount=1;
secontcount=2;



while plotdone==false
    
    firstpoint=pathonimage{firstcount};
    secondpoint=pathonimage{secontcount};
    
     pause(0.1);
    plot([firstpoint(1),secondpoint(1)],[firstpoint(2),secondpoint(2)],'-r');
    plot([firstpoint(1)+10,secondpoint(1)+10],[firstpoint(2)+10,secondpoint(2)+10],'-b');
    plot([firstpoint(1)-10,secondpoint(1)-10],[firstpoint(2)-10,secondpoint(2)-10],'-b');
    
    plot(firstpoint(1),firstpoint(2),'ro');
    pause(0.1);

    firstcount=firstcount+1;
    secontcount=secontcount+1;

    if firstcount==elementsX
        plotdone=true;
    end

end





xGoal=robotpose2.LatestMessage.Pose.Pose.Position.X;
yGoal=robotpose2.LatestMessage.Pose.Pose.Position.Y;
path{end+1}=[xGoal,yGoal];
    


facinggoal=false;
reachgoal=false;
obstaclecheck=false;


cout=1;

while reachgoal==false 
    
 
%     display(obchecksub.LatestMessage);
    obstaclecheck=obchecksub.LatestMessage.Data;
    if obstaclecheck==true;
        display ('detected obstacle');
        break
    end
    
    
    
    nextGoal=pathonglobal{cout};
    
    xpost=robotpose.LatestMessage.Pose.Pose.Position.X;
    ypost=robotpose.LatestMessage.Pose.Pose.Position.Y;
    
    poseonlocal=[xpost-point0(1),ypost-point0(2)];
    
    
    poseonmap=global2image(poseonlocal(1),poseonlocal(2),resX,resY,osx,osy);
    plot(poseonmap(1),poseonmap(2),'ro');
    hold on;
    drawnow;

    

    while facinggoal==false 

    %%%%%%%%%%%%%%%
x1=robotpose.LatestMessage.Pose.Pose.Position.X;
y1=robotpose.LatestMessage.Pose.Pose.Position.Y;

x_next=nextGoal(1);
y_next=nextGoal(2);

yt=y_next-y1;
xt=x_next-x1;
angle=atan2(yt,xt);


[pitch, roll, yaw1]=quat2angle([robotpose.LatestMessage.Pose.Pose.Orientation.X,robotpose.LatestMessage.Pose.Pose.Orientation.Y,robotpose.LatestMessage.Pose.Pose.Orientation.Z,robotpose.LatestMessage.Pose.Pose.Orientation.W]);

angledeg=radtodeg(angle);
yaw1deg=radtodeg(yaw1);

if angledeg<0
    angledeg=360+angledeg;
end 
if yaw1deg<0
    yaw1deg=360+yaw1deg
end

      
      turndir=angle-yaw1;
      if turndir>=0
        turningmsg.Linear.X=0;
        turningmsg.Angular.Z=0.1;
        send(turningpub,turningmsg);
      end
      
      if turndir<=0
        turningmsg.Linear.X=0;
        turningmsg.Angular.Z=-0.1;
        send(turningpub,turningmsg);
      end

    
    if abs(yaw1-angle)<=0.01
        
        turningmsg.Angular.Z=0;
        send(turningpub,turningmsg);
        facinggoal=true;
        
        
        display('finish turning');
    end
    end
    
    if facinggoal==true
    turningmsg.Angular.Z=0;
    turningmsg.Linear.X=0.1;
    send(turningpub,turningmsg);
    display('moving forward nextGoal');
    end
   
    
    robotx=robotpose.LatestMessage.Pose.Pose.Position.X;
    roboty=robotpose.LatestMessage.Pose.Pose.Position.Y;
    
         if abs(robotx-x_next)<=0.1 && abs(roboty-y_next)<=0.1
    
    facinggoal=false;
    turningmsg.Angular.Z=0;
    turningmsg.Linear.X=0;
    send(turningpub,turningmsg);
        display('adjusting ');
        
        pause(0.5);
        
    end

    
    if abs(robotx-x_next)<=0.01 && abs(roboty-y_next)<=0.01
    
        facinggoal=false;
    turningmsg.Angular.Z=0;
    turningmsg.Linear.X=0;
    send(turningpub,turningmsg);
        display('+++++++++++');
        cout=cout+1;
        pause(0.5);
        
    end
   

    if abs( robotx-xGoal)<=0.1 && abs(roboty-yGoal)<=0.1 
    turningmsg.Angular.Z=0;
    turningmsg.Linear.X=0;
    send(turningpub,turningmsg);
    reachgoal=true;
    break
    
    end
        
    if reachgoal==true
        break
    end
 
    
end



display ('navigation finish');







    