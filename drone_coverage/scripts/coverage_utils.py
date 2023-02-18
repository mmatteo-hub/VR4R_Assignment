import rospy

class CoverageUtils:
    
    @staticmethod
    def get_drones_names():
        # The drones names are passed as an encoded array
        drones_names_str = rospy.get_param("/drones_names")
        drones_names = CoverageUtils._drones_names_from_str(drones_names_str)
        return drones_names
    

    @staticmethod
    def _drones_names_from_str(string):
        # Removing eventual spaces at the beginning and end
        string = string.strip()
        # Check if there are the parenthesis
        if string[0] != "[" or string[-1] != "]" :
            return None
        # Removing parenthesis
        string = string[1:-1]
        # Splitting with comman
        return list(map(lambda name : name.strip(), string.split(",")))