import socket
import json
import time

class Elt:
    """
    a class for elite control
    """

    def __init__(self, ip="192.168.1.200"):
        """initialize elite control object
        ip: connect ip
        """
        self.ip = ip
        self.conSuc, self.sock = self.connectETController(self.ip)
        if self.conSuc:
            print("elite connect success")
        else:
            print("elite connect fail")

    def connectETController(self, ip, port=8055):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((ip, port))
            return (True, sock)
        except Exception as e:
            sock.close()
            return (False,)

    def disconnectETController(self):
        if (self.sock):
            self.sock.close()
            self.sock = None
        else:
            self.sock = None

    def sendCMD(self, cmd, params=None, id=1):
        if (not params):
            params = []
        else:
            params = json.dumps(params)
        sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params,id) + "\n"
        try:
            self.sock.sendall(bytes(sendStr, "utf-8"))
            ret = self.sock.recv(1024)
            jdata = json.loads(str(ret, "utf-8"))
            if ("result" in jdata.keys()):
                return (True, json.loads(jdata["result"]), jdata["id"])
            elif ("error" in jdata.keys()):
                return (False, jdata["error"], jdata["id"])
            else:
                return (False, None, None)
        except Exception as e:
            return (False, None, None)

    """
    1. ServoService
    """
    def getServoStatus(self):
        """get robot servo status
        return: bool result, True(active)/False(inactive)
        """
        suc, result, id = self.sendCMD("getServoStatus")
        return result

    def setServoStatus(self, status):
        """set robot servo status
        param: int status, 1(open)/0(close)
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("set_servo_status", {"status":status})
        time.sleep(1)
        return result

    def syncMotorStatus(self):
        """sync motor status
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("syncMotorStatus")
        time.sleep(0.5)
        return result

    def clearAlarm(self):
        """clear alarm
        return bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("clearAlarm")
        return result

    def getMotorStatus(self):
        """get motor sync status
        return: bool result, True(sync)/False(unsync)
        """
        suc, result, id = self.sendCMD("getMotorStatus")
        return result

    """
    2. ParamService
    """
    def getRobotState(self):
        """get robot state
        return int result, 0(stop)/1(pause)/2(emergency stop)/3(run)/4(error)/5(collision)
        """
        suc, result, id = self.sendCMD("getRobotState")
        return result

    def getRobotMode(self):
        """get robot mode
        return: int result, 0(teach)/1(play)/2(remote)
        """
        suc, result, id = self.sendCMD("getRobotMode")
        return result

    def getRobotPos(self):
        """get robot current position
        return: list result, robot current position
        """
        suc, result, id = self.sendCMD("getRobotPos")
        return result

    def getRobotPose(self, coorddinate_num=-1, tool_num=-1):
        """get robot current position and orientation
        param: int coorddinate_num, -1(base frame), [0, 7](corresponding user frame)
        param: int tool_num, -1(current tool number), [0, 7](corresponding tool number)
        return: list result, robot current position and orientation
        """
        suc, result, id = self.sendCMD("getRobotPose", 
        {"coorddinate_num":coorddinate_num, "tool_num":tool_num})
        return result

    def getMotorSpeed(self):
        """get robot motor speed
        return: list result, robot motor speed
        """
        suc, result, id = self.sendCMD("getMotorSpeed")
        return result

    def getCurrentCoord(self):
        """get robot current coordinate system
        return: int result, 0(joint)/1(Cartesian)/2(tool)/3(user)/4(cylinder)
        """
        suc, result, id = self.sendCMD("getCurrentCoord")
        return result

    # getCycleMode()
    # getCurrentJobLine()
    # getCurrentEncode()
    # getToolNumber()
    # setToolNumber()
    # getUserNumber()
    # setUserNumber()

    def getRobotTorques(self):
        """get robot current torques
        return: list result, robot current torques
        """
        suc, result, id = self.sendCMD("getRobotTorques")
        return result

    # getPathPointIndex()
    # getAnalogInput()
    # setAnalogOutput()

    def setCurrentCoord(self, coord_mode):
        """set robot current coordinate system
        param: int coord_mode, 0(joint)/1(Cartesian)/2(tool)/3(user)/4(cylinder)
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("setCurrentCoord", {"coord_mode":coord_mode})
        time.sleep(0.5)
        return result

    def dragTeachSwitch(self, switch):
        """open/close drag switch
        param: int switch, 0(close)/1(open)
        return: True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("drag_teach_switch", {"switch":switch})
        return result

    def cmdSetPayload(self, tool_num, m, point):
        """set robot payload and payload's center of gravity
        param: int tool_num, range[0, 7]
        param: int m, mass of payload(Kg), range[0, 7.2] for EC66
        param: list point, center of gravity of x, y and z(mm), range[-5000, 5000]
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("cmd_set_payload", 
        {"tool_num":tool_num, "m":m, "point":point})
        return result

    def cmdSetTCP(self, point, tool_num):
        """set robot TCP(tool center point)
        param: list point, position and orientation of TCP 
        param: int tool_num, tool number, range[0, 7]
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("cmd_set_tcp", {"point":point, "tool_num":tool_num})
        return result

    def getCollisionState(self):
        """get robot collision state
        return: int result, 1(collision)/0(no collision)
        """
        suc, result, id = self.sendCMD("getCollisionState")
        return result

    # getUserFrame()
    # setCycleMode()
    # setUserFrame()

    def getTCPPos(self, tool_num):
        """get tool frame position and orientation
        param: int tool_num, tool number, range[0, 7]
        return: list result, tool frame position(x, y, z) and orientation(rx, ry, rz)
                the unit of rx, ry and rz is degree
        """
        suc, result, id = self.sendCMD("getTcpPos", {"tool_num":tool_num})
        return result

    def getPayload(self, tool_num):
        """get the mass of tool payload
        param: int tool_num, tool number, range[0, 7]
        return: double result, the mass of tool payload
        """
        suc, result, id = self.sendCMD("getPayload", {"tool_num":tool_num})
        return result

    def getToolCentreMass(self, tool_num):
        """get the canter mass of tool
        param: int tool_num, tool number, range[0, 7]
        return: list result, the canter mass of tool
        """
        suc, result, id = self.sendCMD("getCentreMass", {"tool_num":tool_num})
        return result

    # getRobotType()
    # getDH()
    # setCollisionEnable()
    # setCollisionSensitivity()
    # setSafetyParams()
    # getSpeed()
    # resetCollisionState()
    # getAutoRunToolNumber()
    # setAutoRunToolNumber()

    def getBaseFlangePose(self):
        """get the position and orientation of flange in Cartesian coordinate system
        return: list result, the position and orientation of flange in Cartesian coordinate system
        """
        suc, result, id = self.sendCMD("get_base_flange_pose")
        return result

    # get_user_flange_pose()
    # setSysVarP()
    # save_var_data()
    # getRobotSubtype()
    # getRobotSafetyParamsEnabled()
    # getRobotSafeyPower()
    # getRobotSafetyMomentum()
    # getRobotSafetyToolForce()
    # getRobotSafetyElbowForce()
    # getRobotSpeedPercentage()
    # getRobotDragStartupMaxSpeed()
    # getRobotTorqueErrorMaxPercents()
    # setFlangeButton()
    # checkFlangeButton()

    """
    3. MovementService
    """
    def moveByJoint(self, targetPos, speed, acc, dec, 
                    cond_type=0, cond_num=7, cond_value=1):
        """move by joint
        
        """
        suc, result, id = self.sendCMD("moveByJoint",{"targetPos":targetPos,
        "speed":speed, "acc":acc, "dec":dec, "cond_type":cond_type, "cond_num":cond_num,
        "cond_value":cond_value})
        return result

    def moveByLine(self, targetPos, speed_type, speed, acc, dec, 
                    cond_type=0, cond_num=7, cond_value=1):
        """move by line
        
        """
        suc, result, id = self.sendCMD("moveByLine",{"targetPos":targetPos,
        "speed_type":speed_type, "speed":speed, "acc":acc, "dec":dec,
        "cond_type":cond_type, "cond_num":cond_num, "cond_value":cond_value})
        return result

    def moveByArc(self, midPos, targetPos, speed_type, speed, 
                cond_type=0, cond_num=7, cond_value=1):
        """move by arc
        
        """
        suc, result, id = self.sendCMD("moveByArc",{"midPos":midPos, 
        "targetPos":targetPos, "speed_type":speed_type, "speed":speed, 
        "cond_type":cond_type, "cond_num":cond_num, "cond_value":cond_value})
        return result

    # moveByRotate()
    # addPathPoint()
    # clearPathPoint()
    # moveByPath()
    # jog()

    def stop(self):
        """stop robot
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("stop")
        return result

    def run(self):
        """run robot
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("run")
        return result

    def pause(self):
        """pause robot
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("pause")
        return result

    # checkJbiExist()
    # runJbi()
    # getJbiState()
    # setSpeed()
    # moveBySpeedj()
    # moveBySpeedl()

    """
    4. KinematicsService
    """
    def inverseKinematic(self, targetPose):
        """robot inverse kinematics
        param: list targetPose, target position and orientation
        return: list result, corresponding joint angles
        """
        suc, result, id = self.sendCMD("inverseKinematic", {"targetPose":targetPose})
        return result

    def positiveKinematic(self, targetPose):
        """robot positive kinematics
        param: list targetPose, target joint angles
        return: list result, corresponding position and orientation
        """
        suc, result, id = self.sendCMD("positiveKinematic", {"targetPose":targetPose})
        return result

    # convertPoseFromCartToUser()
    # convertPoseFromUserToCart()
    # inverseKinematic()
    # poseMul()
    # poseInv()

    """
    5. IOService
    """

    """
    6. VarService
    """

    """
    7. TransparentTransmissionService
    """
    def transparentTransmissionInit(self, lookahead, t, smoothness):
        """initialize robot transparent transmission
        param: double lookahead, lookahead time(ms), range[10, 1000]
        param: double t, sample time(ms), range[2, 100]
        param: double smoothness, gain, range[0, 1]
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("transparent_transmission_init", 
        {"lookahead":lookahead, "t":t, "smoothness":smoothness})
        return result

    def ttSetCurrentServoJoint(self, targetPos):
        """set current transparent transmission servo target joint angles
        param: list targetPos, target joint angles
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("tt_set_current_servo_joint", {"targetPos":targetPos})
        return result

    def ttPutServoJointToBuf(self, targetPos):
        """put transparent transmission servo target joint angles to buffer
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("tt_put_servo_joint_to_buf", {"targetPos":targetPos})
        return result

    def ttClearServoJointBuf(self, clear=0):
        """clear transparent transmission buffer
        param: int clear, 0
        return: bool result, True(success)/False(fail)
        """
        suc, result, id = self.sendCMD("tt_clear_servo_joint_buf", {"clear":clear})
        time.sleep(0.5)
        return result

    def getTransparentTransmissionState(self):
        """get current robot transparent transmission state
        return: bool result, current tt state
        """
        suc, result, id = self.sendCMD("get_transparent_transmission_state")
        return result

    """
    8. SystemService
    """
    def getSoftVersion(self):
        """get controller software version
        return: result, controller software version
        """
        suc, result, id = self.sendCMD("getSoftVersion")
        return result

    def getJointVersion(self, axis):
        """get joint servo version
        param: int axis, range[0, 7] corresponding to axis 1-8
        return: joint servo version
        """
        suc, result, id = self.sendCMD("getJointVersion", {"axis":axis})
        return result

if __name__ == "__main__":
    robot_ip = "192.168.1.200"
    my_elt = Elt(robot_ip)