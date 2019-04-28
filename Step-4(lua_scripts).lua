-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()
	k=1
	kr=1
	tt=1
	qw=1
	cb=0
    wr=qw+1
    -- Add required handles here
    edrone = sim.getObjectHandle('Drone_Pos_Emulation')
   	g1 = sim.getObjectHandle('goal_1')
   	g2 = sim.getObjectHandle('goal_2')
   	g3 = sim.getObjectHandle('goal_3')
  	g4= sim.getObjectHandle('goal_4')
  	g5 = sim.getObjectHandle('goal_5')
   	g6 = sim.getObjectHandle('goal_6')
    g7=sim.getObjectHandle('goal_7')
    -- Creating Food tree handles
    
    pos_1= sim.getObjectHandle('Position_hoop1')
    or_1= sim.getObjectHandle('Orientation_hoop1')
    
    pos_2= sim.getObjectHandle('Position_hoop2')
    or_2= sim.getObjectHandle('Orientation_hoop2')
    
    pos_3= sim.getObjectHandle('Position_hoop3')
    or_3= sim.getObjectHandle('Orientation_hoop3')

    obs1= sim.getObjectHandle('obstacle_1')
    obs2= sim.getObjectHandle('obstacle_2')

    whycon_1={}
    aruco_1={}
    arr={}
    ht={}
    pos={pos_1,pos_2,pos_3,obs1,obs2}
    ori={or_1,or_2,or_3}

  
    
    --Creating Non food tree handles
    no_of_obstacles = 2
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle'..tostring(i)))
    end

    -- Creating and setting space and algorithm to create the path
    t=simOMPL.createTask('t') 
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,edrone,{-1.71,-1.8,0},{1.7,1.35,2},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    
    compute_path_flag =  false
    simROS.subscribe('/need_path','std_msgs/Float64','callback')

    scale_factor = {-0.1348,0.1322,0.0548} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required = 25-- Add no of path points you want from one point to another




    -- Subscribing to the required topics 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end

-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding

function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message

function packdata(path)
    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x = path[i]/scale_factor[1]
        pose.position.y = path[i+1]/scale_factor[2]
        pose.position.z = 35 - (path[i+2]/scale_factor[3])
        sender.poses[math.floor(i/7) + 1] = pose


        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    return sender
end

--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path

    r,path=simOMPL.compute(t,1,-1,no_of_path_points_required) -- Provide the correct arguments here.. Make sure you check the no of path points it actually computes
    --print (r,#path)
    if(r == true)then
        visualizePath(path)
        message = packdata(path)  
        --print (message)
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end


function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion
    x1 = msg.markers[1].pose.pose.orientation.x
    y1 = msg.markers[1].pose.pose.orientation.y
    z1 = msg.markers[1].pose.pose.orientation.z
    w1 = msg.markers[1].pose.pose.orientation.w
    print (w1)
    aruco_3 = {w1,-z1,-y1,-x1}--cashew
    aruco_1 = {y1,x1,w1,-z1}--sal tree
    aruco_2={-x1,y1,-z1,w1}----mango tree
    arr={aruco_1,aruco_2,aruco_3}
    --n21=sim.setObjectQuaternion(or_1,-1,aruco_1)
    
end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    x2 = msg.poses[1].position.x
    y2 = msg.poses[1].position.y
    z2 = msg.poses[1].position.z
    --z2=32-z2
    ht={38.5,34,34,28,29}
    
    if k>-1 then
    	whycon_1 = {-0.1348*x2,0.1322*y2,0.0548*(ht[kr]-z2)}
    end
    --n1=sim.setObjectPosition(pos_2,-1,whycon_1)
    	--n1=sim.setObjectPosition(edrone,-1,whycon_1)
    if k==-1 then
    	z2=32-z2
    	whycon_1 = {-0.1348*x2,0.1322*y2,0.0548*z2}
		n1=sim.setObjectPosition(edrone,-1,whycon_1)
   	end
    
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    d=msg.data
    if(d==25)then
        n1=sim.setObjectPosition(pos_1,-1,whycon_1)
        n2=sim.setObjectQuaternion(or_1,-1,arr[kr])
        kr=kr+1
        --print(kr)
    end
    if(d==30)then
        n1=sim.setObjectPosition(pos_2,-1,whycon_1)
        n2=sim.setObjectQuaternion(or_2,-1,arr[kr])
        --print(kr)
        kr=kr+1
    end
    if(d==35)then
        n1=sim.setObjectPosition(pos_3,-1,whycon_1)
        n2=sim.setObjectQuaternion(or_3,-1,arr[kr])
        print(kr)
        kr=kr+1
    end
    if(d==45)then
    	if cb==0 then
        	n1=sim.setObjectPosition(obs1,-1,whycon_1)
            print(ht[kr])
            print(kr)
        end
        if cb==1 then
        	n1=sim.setObjectPosition(obs2,-1,whycon_1)
            print(ht[kr])
            print(kr)
        	k=-1
        end
        kr=kr+1
        cb=1
    end
    
    --else
    	--if k>-1 then
    	--	n1=sim.setObjectPosition(pos[k],-1,whycon_1) 
        --	if (tt>-1) then
    	  -- 		n2=sim.setObjectQuaternion(ori[tt],-1,arr[tt])
           	--	tt=tt+1
        	--end
        	--print(arr[tt])
    		--print(k,tt)
    	
    		--k=k+1
    		--if (tt>3)then
    		--	tt=-1
    		--end
    		--if k>5 then
    		--	k=-1
    		--end
    	--end
    --end
end

function sysCall_actuation()
     
    l={g7,g1,g2,g5,g6,g2,g1,g4,g3,g1,g2,g1,g7}---list containing order of the goals between which path is to be made
    if compute_path_flag == true then
        -- Getting startpose

        start_pose = getpose(l[qw],-1)
        -- Getting the goalpose
        
        goal_pose = getpose(l[wr],-1)

        qw=qw+1
        wr=wr+1

        -- Setting start state
        simOMPL.setStartState(t,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t)
        if(status == true) then -- path computed
            compute_path_flag = false
        end
        
    end
end
function callback(msg)
    compute_path_flag=true
    
end
