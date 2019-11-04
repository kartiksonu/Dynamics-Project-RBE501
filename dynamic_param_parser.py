import numpy as np
import rbdl
import yaml

## Loading the file
def yaml_loader(filepath):
    """ Loads a file path """
    with open(filepath, 'r') as file_descriptor:
        data = yaml.load(file_descriptor)
    return data

## Dumping it
def yaml_dumper(filepath):
    """ Dumps data to a yaml file"""
    with open (filepath, 'w') as file_descriptor:
        yaml.dump(data, file_descriptor)

# File path
# file_path = "/home/sonu/KUKA_7Dof/blender-kuka7dof.yaml"
file_path = "/home/sonu/KUKA_7Dof/blender-kuka7dof.yaml"
data = yaml_loader(file_path)

# Bodies and Joints count
Bodies = []
Joints = []

# Function for couting number of bodies in robot
def Bodies_count(data):
    bodies = data.get('bodies')

    for ele in bodies:
        Bodies.append(ele)

    num_of_bodies = len(bodies)

    # return num_of_bodies, Bodies
    return num_of_bodies, Bodies

# Function for couting number of joints in robot
def Joint_count(data):
    joints = data.get('joints')
    for ele in joints:
        Joints.append(ele)

    num_of_joints = len(joints)

    return num_of_joints, Joints

# Joint and Bodies count testing
n_j,j = Joint_count(data)
n_b,b = Bodies_count(data)

# Function to read mass values
def get_mass_array(data,Bodies):
    mass_arr=[1]
    for body in Bodies:
        mass_arr.append(data[body]['mass'])
    mass_arr.pop(1)
    return np.array(mass_arr).reshape(len(mass_arr),-1)

# Function to read inertia values
def get_inertia_values(data, Bodies):

    inn_val_arr = []
    for bdy in Bodies:
        inn_val_temp=[]
        inn_val_temp.append(data[bdy]['inertia']['ix'])
        inn_val_temp.append(data[bdy]['inertia']['iy'])
        inn_val_temp.append(data[bdy]['inertia']['iz'])
        inn_val_arr.append(inn_val_temp)

    return np.array(inn_val_arr)

# Function to get the position of centre of mass values
def get_inertial_offset(data, Bodies):

    inn_off_arr = []
    for bdy in Bodies:
        inn_off_temp=[]
        inn_off_temp.append(data[bdy]['inertial offset']['position']['x'])
        inn_off_temp.append(data[bdy]['inertial offset']['position']['y'])
        inn_off_temp.append(data[bdy]['inertial offset']['position']['z'])
        inn_off_arr.append(inn_off_temp)

    return np.array(inn_off_arr)

# Function to get type of joint
def get_joint_type(data, Joints):
    J_type = []
    for Joint in Joints:
        J_temp = []
        J_temp.append(data[Joint]['type'])
        J_type.append(J_temp)

    return np.array(J_type)

# Function to get the distance vector between two adjacent bodies
def get_parent_pivot(data, Joints):
	pivot_type = [[0,0,0]]

	for Joint in Joints:
		pivot_temp_type=[]
		pivot_temp_type.append(data[Joint]['parent pivot']['x'])
		pivot_temp_type.append(data[Joint]['parent pivot']['y'])
		pivot_temp_type.append(data[Joint]['parent pivot']['z'])
		pivot_type.append(pivot_temp_type)
	return np.array(pivot_type)

# print("parent pivot value ", get_parent_pivot(data,Joints))

def Inverse_dynamics_calc_func(q_val, qdot_val, qddot_val):

    # Create a new model
    model = rbdl.Model()
    print "No. of bodies =", len(q_val)
    print "Q_values are" , q_val
    # print "Q_values are" , q_val[6]

    
    # Creation of mass array from yaml file
    mass = get_mass_array(data, Bodies)

    # Number of bodies present in the robot
    no_of_bodies, random_var = Bodies_count(data)

    # Creation of joint type array from yaml file
    joint_name_temp = np.array([["JointTypeFixed"]])
    joint_type_name = "JointTypeRevoluteZ"
    for count in range(0, int(no_of_bodies) - 1):
        joint_name_temp = np.append(joint_name_temp, [[joint_type_name]], axis=0)
    joint_name = joint_name_temp
    # joint_name = [["JointTypeFixed"], ["JointTypeRevoluteZ"], ["JointTypeFixed"], ["JointTypeFixed"], ["JointTypeFixed"], ["JointTypeFixed"], ["JointTypeFixed"]]
    
    # The centre of mass array from yaml file
    com_val = get_inertial_offset(data, Bodies)

    # The inertia array from yaml file
    inertia_val = get_inertia_values(data, Bodies)

    # The distance vector array between two adjacent bodies from yaml file
    r_val = get_parent_pivot(data, Joints)


    q     = q_val
    qdot  = qddot_val
    qddot = qddot_val
    tau   = np.zeros(no_of_bodies-1)

    #This for loop iteratively computes the torque values of the entire system
    for i in range(0, no_of_bodies-1):

        # Creating of the transformation matrix between two adjacent bodies
        trans = rbdl.SpatialTransform()
        trans.E = np.eye(3)
        trans.r = r_val[i]

        # Using principal inertia values from yaml file
        I_x = inertia_val[i][0]
        I_y = inertia_val[i][1]
        I_z = inertia_val[i][2]
        # count = count + 1
        # Creation of inertia Matrix
        inertia_matrix = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])

        # Creating each body of the robot
        body = rbdl.Body.fromMassComInertia(mass[i], com_val[i], inertia_matrix)

        # Specifying joint Type
        joint_type = rbdl.Joint.fromJointType(joint_name[i][0])
        # print(joint_type)

        # Assigning the q, q dot, q dot dot values to the input values of inverse dynamics

        # Adding body to the model to create the complete robot
        append_body = model.AppendBody(trans, joint_type, body)


    # RBDL inverse dynamics function
    rbdl.InverseDynamics(model, q, qdot, qddot, tau)
    # print("length of tau is ", len(tau))

    return tau

