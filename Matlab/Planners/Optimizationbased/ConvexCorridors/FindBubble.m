%% Find Bubble for all obstacles with circle
function [x,y,r] = FindBubble(xr,obj)
% the point xr:2*1, obstacle (nobj*2)*4
[m,~] = size(obj);
nobj = 0.5*m;
% Q=zeros(dim*nstep,dim*nstep);
% L=zeros(dim*nstep,1);
r=0.2; 
deltr=r; 
x=xr(1);
y=xr(2);
flag = 0;
for i=1:20 
    for n=1:nobj 
        obstacle=zeros(2,5);
        obstacle(1,1:4)=obj(2*n-1,:);
        obstacle(2,1:4)=obj(2*n,:);
        obstacle(1,5)=obstacle(1,1);
        obstacle(2,5)=obstacle(2,1);
        for j = 1:4 % The four vertices of obstacles
            if norm(obstacle(:,j)-xr) < r || judge(xr',obstacle(:,j)',obstacle(:,j+1)',r)==1
                flag = 1;
                break
            end
        end
        if flag ==1
            break
        end
    end
  if flag == 1
        r=r-deltr;
        break
  else
      r=deltr*i;
  end 
end
% center:£¨x,y£©,radius:r
% To facilitate the solution of quadratic problem: x = cplexqcp(H,f,Aineq,bineq,Aeq,beq,L,Q,R),½«bubble×ªÎªL'*x + x'*Q*x <= R
% Q((k-1)*dim+1:k*dim,(k-1)*dim+1:k*dim) = eye(dim);
% L((k-1)*dim+1,1) = -2*x;
% L(k*dim,1) = -2*y;
% R=r^2-x^2-y^2;
% rectangle('Position',[x-l,y-w,2*l,2*w]) % Draw rectangle
% rectangle('Position', [x-r, y-r, 2*r, 2*r], 'Curvature', [1 1]) %Draw circle
% hold on;
end


function flag=judge(P,Q1, Q2,r)  
flag=0;
% P - point coordinates; coordinates of two points on line Q1, Q2, coordinates are row vectors
x1=P(1); y1=P(2);
d = norm(cross([Q2 0]-[Q1 0],[P 0]-[Q1 0]))/norm([Q2 0]-[Q1 0]);
% Projection point of the point on the line
l=[Q1(2)-Q2(2),Q2(1)-Q1(1)];
s=l*Q1';
if l(1)==0  % same as vertical coordinate
    if x1<=max(Q2(1),Q1(2)) && x1>=min(Q2(1),Q1(2))
        if d<=r
            flag=1;
        end
    end
elseif l(2)==0 % same as horizontal coordinate
    if y1<=max(Q2(2),Q1(2)) && y1>=min(Q2(2),Q1(2))
        if d<=r
            flag=1;
        end
    end
else
    k= -l(1)/l(2);
    b= s/l(2);
    x2 = (k*y1+x1-k*b)/(1+k*k);
    y2 = k*x2+b;
    O = [x2, y2];
    OA = O - Q1;
    OB = O - Q2;
    cosOAOB = OA*OB'/(norm(OA)*norm(OB));
    % the projection point is on the line segment
    if cosOAOB ==-1 && d<r
        flag=1;
    end
end
end
