import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.PositionSensor;
import java.io.File; 
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.FileNotFoundException; 
import java.io.IOException;
import java.util.Scanner; 
import java.util.Arrays;
import java.lang.Math;
import java.util.*;

public class twoopt {

  static Motor left_wheel;
  static Motor right_wheel;
  static Compass compass;
  static GPS  gps;
  static PositionSensor left_en;
  static DistanceSensor distance_sensors[] = new DistanceSensor[4];//For the 4 distance sensors in front of the robot
  static DistanceSensor side_distance_sensors[] = new DistanceSensor[2];//For the 2 distancesensor on the side of the robot
  static double target[] = new double[2];// Array that contains the target
  static double home[] = new double[2]; // Robot home location
  static String state = "turning";
  static int visit_count = 1; /// The number of targets left to visit
  static double start_time = 0; 
  static boolean end_run = false; // Variable to check if the simulation is complete
  static double current_en_value = 0; // Variables to store current value of position sensor
  static double total_en_value = 0; // Variables to store total value that the robot travled forward
  static double[][] nodes;
  static double[][] two_opt_path;
  static Supervisor robot;
  
  
  public static void main(String[] args) {
    int num_of_nodes = 0;
    // create the Robot instance.
    robot = new Supervisor();
    Node mark = robot.getFromDef("mark"); // The blue spot
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    left_wheel = robot.getMotor("left wheel motor");
    right_wheel = robot.getMotor("right wheel motor");
    left_wheel.setVelocity(0);
    left_wheel.setPosition(Double.POSITIVE_INFINITY);
    right_wheel.setVelocity(0);
    right_wheel.setPosition(Double.POSITIVE_INFINITY);
    
    left_en = robot.getPositionSensor("left wheel sensor");
    left_en.enable(timeStep);
    
    String[] distance_sensor_names = new String[]{"ps1","ps0","ps7","ps6"};
    String[] side_distance_sensor_names = new String[]{"ps2","ps5"};
    for(int i=0; i<4;i++)
    {
      distance_sensors[i] = robot.getDistanceSensor(distance_sensor_names[i]);
      distance_sensors[i].enable(timeStep);
    }
    for(int i=0; i<2;i++)
    {
      side_distance_sensors[i] = robot.getDistanceSensor(side_distance_sensor_names[i]);
      side_distance_sensors[i].enable(timeStep);
    }
    gps = robot.getGPS("gps");
    gps.enable(timeStep);
     
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
     
    robot.step(1000);// delay of 1 second before the robot starts
    
    /////// Reading the file to get the number of points 
    try 
    {
      File myObj = new File("../Targets.txt");
      Scanner myReader = new Scanner(myObj);
      String data = myReader.nextLine();
      ///System.out.println(data);
      num_of_nodes = Integer.parseInt(data); /// getting number of targets 
      
      myReader.close();
    } 
    catch (FileNotFoundException e) 
    {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    /////////////////////////////////////////////
    visit_count = 1; // Number of targets to visit
    
    nodes = new double[num_of_nodes+2][3];/// The array containing all targets in the second dimention [x,y,(0 or 1)] 0,1 is to tell if this target has been visited or not
    two_opt_path = new double[num_of_nodes+2][3];/////////Path array
    home[0] = gps.getValues()[0];
    home[1] = gps.getValues()[1];
    
    nodes[0][0] = home[0];//// Adding home position 
    nodes[0][1] = home[1];
    nodes[0][2] = 0;
    ////Getting all targets from text file 
    try 
    {
      File myObj = new File("../Targets.txt");
      Scanner myReader = new Scanner(myObj);
      int count = 1;
      while (myReader.hasNextLine()) {
        String data = myReader.nextLine();
        //System.out.println(data);
        if (data.length() > 2)
        {
          String[] points = data.split(",");
          double x = Double.parseDouble(points[0]);
          double y = Double.parseDouble(points[1]);

          nodes[count][0] = x;
          nodes[count][1] = y;
          nodes[count][2] = 0;
          //System.out.println(count);
          count +=1;
        }
      }
      myReader.close();
    } 
    catch (FileNotFoundException e) 
    {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    
    Field mark_f = mark.getField("translation");
    
    nodes[nodes.length-1][0] = home[0];/////Setting home location again since we have to return to it
    nodes[nodes.length-1][1] = home[1];
    nodes[nodes.length-1][2] = 0;
    //////////////////////////
    start_time = robot.getTime(); /// Recording start time
    two_opt();///calling 2 opt algo
    target[0] = two_opt_path[visit_count][0];/// Setting target
    target[1] = two_opt_path[visit_count][1];
    double[] pos = {target[0],target[1],0};
    
    mark_f.setSFVec3f(pos);
    visit_count ++;
    
  
  
    while (robot.step(timeStep) != -1) 
    {
      
      
      if (end_run) /// Breaks the main while loop
      { 
        ///Output file writing 
        try 
        {
          FileWriter myWriter = new FileWriter("output.txt");
          myWriter.write("Time :- "+(robot.getTime()-start_time)+" seconds Distance :- " + (total_en_value*0.02) + " meters");
          myWriter.close();
          System.out.println("Successfully wrote to the file.");
        } 
        catch (IOException e) 
        {
          System.out.println("An error occurred.");
          e.printStackTrace();
        }
        //////////////////////
        break;
      }
      switch(state) 
      {
      case "stop":
        if (visit_count == two_opt_path.length) /// If home location has been visited
        {
          move_robot(0,0);
          end_run = true; /// Simulation complete
        }
        else///setting next target
        {
          target[0] = two_opt_path[visit_count][0];
          target[1] = two_opt_path[visit_count][1];
          state = "turning";
          visit_count ++ ;
          /////Code to change the blue square location
          double[] pos2 = {target[0],target[1],0};
          mark_f.setSFVec3f(pos2);
          //////////////////////////
        }
        break;
      case "forward":///If the state of the robot is to go forward
        if (at_target())//checking if robot is  at the target
        {
          move_robot(0,0);//If so stop and change state to stop
          state = "stop";
        }
        else
        {
          if(obstacal_found())//checking if there is an obstacle infront
          {
            state = "avoiding"; //if soo switching state to avoiding
            total_en_value += (left_en.getValue() - current_en_value);
            current_en_value = left_en.getValue();
          }
          else
          {
              //following code checks if the robot is on its proper path 
              double robot_angle = robot_direction() ;
              double x = gps.getValues()[0];
              double y = gps.getValues()[1];
              double angle = (Math.atan2((target[1]-y),(target[0]-x))/Math.PI)*180;
              if(((target[1]-y)>0)&&((target[0]-x)>0))
            {
              angle =  angle;
            }
            else if(((target[1]-y)<0)&&((target[0]-x)>0))
            {
              angle = 360+ angle;
            }
            else if (((target[1]-y)>0)&&((target[0]-x)<0))
            {
              angle =  angle;
            }
            else
            {
              angle =  360 + angle;
            }
              if(Math.abs(robot_angle - angle)>5)//If the robots orientation and the orientation the robot should travel is greater than 5 degrees then a corrent of path is needed
              {
                move_robot(0,0);
                state = "turning";//switching state to turning
                total_en_value += (left_en.getValue() - current_en_value);
                current_en_value = left_en.getValue();
              }
              else
              {
                move_robot(5,5);//moving the robot straight
              }
             }
           }
          break;
        case "turning"://if the  state is turning
          double robot_angle = robot_direction() ;//getting robots orientation
          
          double x = gps.getValues()[0];//getting robots x coord
          double y = gps.getValues()[1];//getting robots y coord
          if (at_target() == false)//if at target
          {
            ///the below code gets the angle between the robots centroid and the target location which is the direction the robot should travel
            double angle = (Math.atan2((target[1]-y),(target[0]-x))/Math.PI)*180;
              ///following conditions are taken by testing done
              //System.out.println(angle);
            
            if(((target[1]-y)>0)&&((target[0]-x)>0))
            {
              angle =  angle;
            }
            else if(((target[1]-y)<0)&&((target[0]-x)>0))
            {
              angle = 360+ angle;
            }
            else if (((target[1]-y)>0)&&((target[0]-x)<0))
            {
              angle =  angle;
            }
            else
            {
              angle =  360 + angle;
            }
            /////////////////////////////////////////////////////////////
            
            while(Math.abs(robot_angle - angle)>1)///if the direction and the robot orientation is greater than 1 degree it needs a correction
            {
              if (robot_angle >angle)// If the robots angle is greater than angle 
              {
                move_robot(1,-1);//robot turns clockwise till the target angle is reached
              }
              else if(robot_angle <angle)
              {
                move_robot(-1,1);//anticlockwise  
              }
              robot_angle = robot_direction() ;
              
              robot.step(1);
              
            }
            state = "forward";//Since the robot is looking at the correct direction it switches to going forward state
          
          }
          else
          {
            state = "stop";
            move_robot(0,0);
          }
          break;
        case "avoiding":///Avoiding state
        if(obstacal_found()) //if there is a obstacle infront of the robot
          {
            avoid_obs();//turns to avoid it
            robot.step(100);
            move_robot(5,5);//slightly goes forward
            robot.step(100);
          }
          else if (side_obstacal_found())//a objetc is detected on the side
          {
             move_robot(5,5);//go forward becaude the robot wont be hitting it but has to pass it
             robot.step(1500);
          }
          else
          {
            
            move_robot(5,5);//going forward a bit 
            robot.step(100);
            state = "turning";//Switching state ot turning
          }
          
          break;
      }
      
    };

    // Enter here exit cleanup code.
  }
  static void move_robot(double ls, double rs)//Function to move robot
    {
      //setting speeds to wheels
      left_wheel.setVelocity(ls);
      right_wheel.setVelocity(rs);
    }
    
   static boolean obstacal_found()//checking if a obstacle is found on the front of the robot
    {
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)///if distance sensor value is less than 1000 there is an object
         {
           return true;
         };
         
       }
       return false;
    }
    static boolean side_obstacal_found()//checking if a obstacle is found on the side of the robot
    {
      for(int i=0; i<2;i++)
       {
         if (side_distance_sensors[i].getValue() <1000)
         {
           return true;
         };
         
       }
       return false;
    }
    static void avoid_obs()//function to turn robot when there is an object infront
    {
      double base = 3;//robots speed
      double total = 0;//variable to save the total value of weights
      double count = 0;// number of distance sensors that detected something
      double ls = 0;//left speed
      double rs = 0;//right speed
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)
         {
           total += i;//adding distance sensor weight
           count ++;
         };
         
       }
       if (count>0)//if any distnce sensors caught a object
       {
         //algorithm to decied how much to turn 
         ls = base + (total/count - 2)*3;//will be expaloned in the video
         rs = base - (total/count - 2)*3;
         move_robot(ls,rs);
       }
       else
       {
         move_robot(base,base);
       }
    }
    
    static double robot_direction() //getting robots orientation
    {
      double[] values = compass.getValues();//getting robots compass reading
      
      double rad = Math.atan(values[0]/  values[1]);//getting robots angle
      //converting the received angle to the range of 0-360
      double bearing =0 ;
      if(values[0]>0 && values[1]>0)
      {
        bearing = (rad/ Math.PI) * 180.0;
      }
      else if((values[0]>0 && values[1]<0) || (values[0]<0 && values[1]<0))
      {
        bearing = ((rad + Math.PI)/ Math.PI ) * 180.0;
      }
      else
      {
        bearing = ((rad + Math.PI*2)/ Math.PI ) * 180.0;
      }
     
      bearing =  Math.round(bearing);
      if (bearing == 360)
      {
        bearing  = 0;
      }
      return bearing;
   }
   
   static boolean at_target()//checking if at target
   {
     double x = gps.getValues()[0];
     double y = gps.getValues()[1];
     if(Math.pow((Math.pow((target[0]-x),2)+Math.pow((target[1]-y),2)),0.5)<0.05)//Using pythagoras and getting distance between robot and target and if its less than 5cm telling the robot to stop
     {
       return true;
     }
     return false;
   }
   
   static double path_distance()
   {
     double distance= 0;
     for (int i = 1 ; i<nodes.length; i++)
     {
       distance += Math.pow((Math.pow((nodes[i][0] - nodes[i-1][0]),2)+Math.pow((nodes[i][1] - nodes[i-1][1]),2)),0.5);
     }
     return distance;
   }
   
   static void two_opt_swap(int i, int k)////Swap function
   {
     
     double[][] b = new double[k-i][3];
     int count = 0;
     for(i=i;i<k;i++)///copying swap part
     {
       b[count][0] = nodes[i][0];
       b[count][1] = nodes[i][1];
       count ++;
     }
     
     
     for(int start = 0; start<b.length;start++ )
     {
       nodes[k-start-1][0] = b[start][0];
       nodes[k-start-1][1] = b[start][1];
     }
   }
   
   static void two_opt()
   {
     int count = 0;
     double best_distance = path_distance();
     for(int i = 0; i<nodes.length;i++)////Making a deep copy of the nodes array
     {
       two_opt_path[i][0]  = nodes[i][0]; 
       two_opt_path[i][1]  = nodes[i][1];
       two_opt_path[i][2]  = nodes[i][2];
     }

     while (robot.step(32) != -1) 
    {
      if (count>1000)////Counting the times no new  path has been found
      {
        break;
      }
      for(int i = 1; i < nodes.length -1 ; i++)
      {
        for(int k = i+2; k < nodes.length -1 ; k++)
        {
          two_opt_swap(i, k);
          double new_distance = path_distance();
          if(new_distance < best_distance)///better path found
          {
            best_distance = new_distance;
            for(int p = 0; p<nodes.length;p++)////Making a deep copy of the nodes array
            {
             two_opt_path[p][0]  = nodes[p][0]; 
             two_opt_path[p][1]  = nodes[p][1];
             two_opt_path[p][2]  = nodes[p][2];
             }
            count = 0;
          }
          else
          {
            count ++;
          }
        }
      }
    }
   }
   
}
