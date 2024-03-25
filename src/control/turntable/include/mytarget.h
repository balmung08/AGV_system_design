

class MyTarget
{
private:

   
public:
   vector<geometry_msgs::PointStamped> pos, vel;
   bool visible; 

   void Clear(void)
   {
       visible=false;
       pos.clear();  vel.clear();
   }
   
   void AddPos(geometry_msgs::Point p)
   {
       geometry_msgs::PointStamped mp;
       mp.point=p;
       mp.header.stamp=ros::Time::now();

       visible=true;
       pos.push_back(mp);
       if(pos.size()>1)
       {
           auto p2=pos[pos.size()-1],  p1=pos[pos.size()-2];
           float dt=(p2.header.stamp-p1.header.stamp).toSec();
           mp.point.x=(p2.point.x-p1.point.x)/dt;
           mp.point.y=(p2.point.y-p1.point.y)/dt;
           mp.point.z=(p2.point.z-p1.point.z)/dt;
           mp.header.stamp=ros::Time::now();
           vel.push_back(mp);
       }

       if(pos.size()>200)  pos.erase(pos.begin());
       if(vel.size()>200)  vel.erase(vel.begin());
   }

   geometry_msgs::Point GetPos(int fnum)
   {
       geometry_msgs::Point p;
       p.x=p.y=p.z=0;
       if(pos.size()>=fnum)
       {
           for(auto it=pos.end()-fnum; it!=pos.end(); ++it)
           {
               p.x+=it->point.x;
               p.y+=it->point.y;
               p.z+=it->point.z;
           }
           p.x/=fnum;  p.y/=fnum;  p.z/=fnum;
       }
       return p;
   }

   geometry_msgs::Point GetVel(int fnum)
   {
       geometry_msgs::Point p;
       p.x=p.y=p.z=0;
       if(vel.size()>=fnum)
       {
           for(auto it=vel.end()-fnum; it!=vel.end(); ++it)
           {
               p.x+=it->point.x;
               p.y+=it->point.y;
               p.z+=it->point.z;
           }
           p.x/=fnum;  p.y/=fnum;  p.z/=fnum;
       }
       return p;
   }
};
