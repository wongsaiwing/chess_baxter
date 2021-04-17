#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from pick_and_place_moveit import PickAndPlaceMoveIt

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    PicknPlace = PickAndPlaceMoveIt('left', 0.15)
    
    # Referred to:https://helloworldkb.com/11440605/AttributeError%EF%BC%9A%E2%80%9C%E5%B8%83%E5%B0%94%E2%80%9D%E5%AF%B9%E8%B1%A1%E6%B2%A1%E6%9C%89%E5%B1%9E%E6%80%A7%22items%E2%80%9D
    quater_orient = Quaternion(x=-0.024959, y=0.99964, z=0.0073791, w=0.0048645)
    
    PicknPlace.move_to_start(Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=quater_orient))
    
    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')
    
    # setup of chess pieces
    # chessboard state
    board_setup = ['', '', '', '', '', '', 'P*P**P*P', 'RNBQKBNR']
    
    piece_positionmap = dict()
    piece_names = []
    
    # hard coded position at the side of the chess board
    piece_spawn_loc = deepcopy(board_pose)
    piece_spawn_loc.position.x = 0.6
    piece_spawn_loc.position.y = 0.6
    piece_spawn_loc.position.z = 0.8

    for row, each in enumerate(board_setup):
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018

            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference

            if piece in pieces_xml:
                # get chess model and spawn the piece
                spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                spawn_sdf("%s%d" % (piece, col), pieces_xml[piece], "/", piece_spawn_loc, "world")
            
            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))

                # pick chess piece from hard coded position on table
                PicknPlace.pick(Pose( #-0.14 = height
                    position=Point(x=0.6, y=0.6, z=-0.14), 
                    orientation=quater_orient))
                # place the chess piece at the defined position
                PicknPlace.place(Pose( #-0.14 = height
                    position=Point(x=pose.position.x, y=pose.position.y, z = -0.14), 
                orientation=quater_orient))

    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
