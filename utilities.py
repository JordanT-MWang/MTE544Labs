from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
# Part 3: Convert quaternion to yaw angle
def euler_from_quaternion(quat):
    x, y, z, w = quat

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    yaw = atan2(siny_cosp, cosy_cosp)

    return yaw

# Part 4: Calculate distance to target
def calculate_linear_error(current_pose, goal_pose):

    x, y = current_pose[0], current_pose[1]
    goal_x, goal_y = goal_pose[0], goal_pose[1]

    # Difference between target and current position
    dx = goal_x - x
    dy = goal_y - y

    # Euclidean distance
    error_linear = sqrt(dx**2 + dy**2)

    return error_linear


# Part 4: Calculate angle needed to face target
def calculate_angular_error(current_pose, goal_pose):

    x, y, theta = current_pose[0:3]
    goal_x, goal_y = goal_pose[0:2]

    # Find the direction toward the target
    desired_theta = atan2(goal_y - y, goal_x - x)

    # Difference between desired and current heading
    error_angular = desired_theta - theta

    # Keep error between -pi and pi
    while error_angular > M_PI:
        error_angular -= 2 * M_PI

    while error_angular < -M_PI:
        error_angular += 2 * M_PI

    return error_angular

