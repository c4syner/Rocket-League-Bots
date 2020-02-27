import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState

from util.orientation import Orientation
from util.vec import Vec3
import time



class PID:
    """PID controller."""

    def __init__(self, Kp, Ki, Kd, origin_time=None):
        if origin_time is None:
            origin_time = time.time()

        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_time = origin_time
        self.previous_error = 0.0

    def Update(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return 0
        de = error - self.previous_error

        self.Cp = error
        self.Ci += error * dt
        self.Cd = de / dt

        self.previous_time = current_time
        self.previous_error = error

        return (
                (self.Kp * self.Cp)  # proportional term
                + (self.Ki * self.Ci)  # integral term
                + (self.Kd * self.Cd)  # derivative term
        )


class MyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.pid = PID(10, 0, 0.5)
        self.error = 0
        self.jump = 0
        self.throttle = 0
        self.turn = 0
        self.drift = 0
        self.boost = 0
        self.pitch = 0
        self.timeToDrive = 0
        self.beenUp = 0

        self.orangeGoal = Vec3(-32.90999984741211, 5758.33984375, 35.209999084472656)
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.dribble(packet)
        self.controller_state.throttle = self.throttle
        self.controller_state.steer = self.turn
        self.controller_state.handbrake = self.drift
        self.controller_state.boost = self.boost
        self.controller_state.jump = self.jump
        self.controller_state.pitch = self.pitch
        return self.controller_state
    def getLocationForeign(self, packet): #Gets location of your car, only works if there is one other ar
        main_car = packet.game_cars[1]
        car_location = Vec3(main_car.physics.location)
        return car_location
    def getBallLoc(self, packet, secs, test=0):#Arg: Seconds into future from current time #Max of 6
        ball_prediction = self.get_ball_prediction_struct()
        currentTime = packet.game_info.seconds_elapsed
        goalTime = currentTime + secs
        timeList = []
        locations = []
        if ball_prediction is not None:
            for i in range(0, ball_prediction.num_slices):
                prediction_slice = ball_prediction.slices[i]
                closestTime = prediction_slice.game_seconds
                locations.append(prediction_slice.physics.location)
                timeError = math.fabs(goalTime - closestTime)
                timeList.append([timeError, i])
            if(test == 1 and self.index == 0):
                self.renderer.begin_rendering()
                self.renderer.draw_polyline_3d(locations, self.renderer.cyan())
                self.renderer.end_rendering()

            t = min(timeList, key=lambda x:abs(x[0]-0))
            bestSlice = ball_prediction.slices[t[1]]
            location = bestSlice.physics.location
        if(test == 0):
            return location

        else:
            return location
    def findBallGround(self): #returns when ball hits ground next (game seconds)
        ball_prediction = self.get_ball_prediction_struct()

        if ball_prediction is not None:
            for i in range(0, ball_prediction.num_slices):
                prediction_slice = ball_prediction.slices[i]
                location = prediction_slice.physics.location
                if(location.z < 94): #on ground
                    return prediction_slice.game_seconds


    def dribble(self, packet):

        timeChange = self.findBallGround() - packet.game_info.seconds_elapsed
        fballLoc = self.getBallLoc(packet, timeChange, 1)


        #original Code:
        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.velocity)
        main_car = packet.game_cars[self.index]
        car_velocity = Vec3(main_car.physics.velocity)
        car_location = Vec3(main_car.physics.location)
        ballHeight = ball_location.z

        if(ballHeight < 152 and ballHeight > 147):
            self.beenUp += 1
        else:
            self.beenUp = 0
        if(ballHeight < 152 and ballHeight > 147 and self.beenUp >= 5): #officially dribbling ball
            pass
        else:
            ball_location = Vec3(packet.game_ball.physics.location)

        carSpeed = 1410
        carSpeedBoost = 2300
        carMagnitude = self.findMagnitude(car_velocity.x, car_velocity.y)
        fcarToBall = self.Distance3D(car_location, fballLoc)
        if(ball_location.z < 94):
            requiredSpeed = 2300
        else:
            requiredSpeed = fcarToBall/timeChange

        #end new

        mirrorBall = Vec3(ball_location.x, ball_location.y, 0)
        mirrorCar = Vec3(car_location.x, car_location.y, 0)

        car_to_ball = car_location - ball_location
        # Find the direction of our car using the Orientation class

        car_orientation = Orientation(main_car.physics.rotation)
        car_direction = car_orientation.forward
        if(self.index == 0 ):
            print(car_direction)
        distFromBall = self.Distance3D(car_location, ball_location)
        mirrorBall = Vec3(ball_location.x, ball_location.y, 0)
        mirrorCar = Vec3(car_location.x, car_location.y, 0)

        leg = self.Distance3D(mirrorBall, ball_location)
        legCar = self.Distance3D(mirrorBall, mirrorCar)
        distFromGround = self.Distance3D(car_location, mirrorCar)
        hypotenuse = self.Distance3D(mirrorCar, ball_location)
        theta = math.asin(leg / hypotenuse)

        self.turn = self.find_correction(car_direction, car_to_ball)
        self.drift = self.determineDrift(self.find_correction_normal(car_direction, car_to_ball))
        if(self.manageBoost(self.find_correction_normal(car_direction, car_to_ball)) == 1): #Allowed to boost
            self.defaultThrottleDribble(requiredSpeed, carMagnitude)
        else:
            self.boost = 0
            self.throttle = 1

    def defaultThrottleDribble(self, target_speed, carMagnitude): #accidentally made a god tier dribble bot:
        # accelerates the car to a desired speed using throttle and boost
        car_speed = carMagnitude
        t = (target_speed) - car_speed
        self.throttle = self.map(t, 0,1410,-.01,1)
        if(self.throttle < -1):
            self.throttle = -1
        elif(self.throttle > 1):
            self.throttle = 1
        self.boost = True if t > 150 and car_speed < 2275 else False
    def simpleballChase(self, packet):
        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.velocity)
        main_car = packet.game_cars[self.index]

        car_location = Vec3(main_car.physics.location)

        car_to_ball = car_location - ball_location
        # Find the direction of our car using the Orientation class

        car_orientation = Orientation(main_car.physics.rotation)
        car_direction = car_orientation.forward

        distFromBall = self.Distance3D(car_location, ball_location)
        mirrorBall = Vec3(ball_location.x, ball_location.y, 0)
        mirrorCar = Vec3(car_location.x, car_location.y, 0)
        leg = self.Distance3D(mirrorBall, ball_location)
        legCar = self.Distance3D(mirrorBall, mirrorCar)
        distFromGround = self.Distance3D(car_location, mirrorCar)
        hypotenuse = self.Distance3D(mirrorCar, ball_location)
        theta = math.asin(leg / hypotenuse)

        if(legCar < 400):
            self.boost = 0
            self.throttle = self.map(legCar, 400, 0, 1, 0)
        else:
            self.turn = self.find_correction(car_direction, car_to_ball)
            self.drift = self.determineDrift(self.find_correction_normal(car_direction, car_to_ball))
            self.boost = self.manageBoost(self.find_correction_normal(car_direction, car_to_ball))
            self.throttle = 1

    def doAerial(self, car_location, ball_location, car_orientation):
        jump = 0
        pitch = 0
        distFromBall = self.Distance3D(car_location, ball_location)
        mirrorBall = Vec3(ball_location.x, ball_location.y, 0)
        mirrorCar = Vec3(car_location.x, car_location.y, 0)
        leg = self.Distance3D(mirrorBall, ball_location)
        legCar = self.Distance3D(mirrorBall, mirrorCar)
        distFromGround = self.Distance3D(car_location, mirrorCar)
        hypotenuse = self.Distance3D(mirrorCar, ball_location)
        theta = math.asin(leg / hypotenuse)
        car_angle = -car_orientation.pitch
        if (self.index == 0):
            self.renderer.begin_rendering()
            self.renderer.draw_line_3d(mirrorBall, ball_location, self.renderer.cyan())  # Draw Leg
            self.renderer.draw_line_3d(mirrorCar, mirrorBall, self.renderer.lime())  # Draw Leg between car and ball
            self.renderer.draw_line_3d(mirrorCar, ball_location, self.renderer.red())  # Draw hypotenuse
            self.renderer.end_rendering()
        # All data acquired start determining values for flight
        if (leg < 100):
            jump = 0
            pitch = 0
            return [0, 0]
        elif(distFromBall > 2000):
            jump = 0
            pitch = 0
            return [0,0]
        elif(distFromGround > 18):
            jump = 0
            pitch = 0
            return [0,0]
        elif(self.readyToJump == 0):
            jump = 0
            pitch = 0
            return [0,0]
        else:


            jump = 1
            error = theta-car_angle
            if(error > 0):
                pitch = 1
            elif(error < 0 ):
                pitch = -1
            if(self.readyToJump == 0):
                jump = 0
        if(self.index == 0):
            print(distFromGround)
        return [jump, pitch]

    def Distance3D(self, p1, p2):
        xAddend = math.pow((p2.x - p1.x), 2)
        yAddend = math.pow((p2.y - p1.y), 2)
        zAddend = math.pow((p2.z - p1.z), 2)
        return math.sqrt(xAddend + yAddend + zAddend)
    def findMagnitude(self, x,y):
        return math.sqrt(math.pow(x, 2)+math.pow(y, 2))
    def determineJump(self, car_location, ball_location):

        if (self.lastCall == 1):
            self.lastCall = 0
            return 0
        ballHeight = ball_location.z
        carToBall = self.Distance3D(car_location, ball_location)

        if (self.index == 0):
            # print(carToBall)
            a = 1
        if ((carToBall < 600) and ((ballHeight > 100) and (ballHeight < 120))):
            self.lastCall = 1
            return 1
        else:
            return 0

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def determineDrift(self, correct):
        if (correct > .3):
            return 1
        elif (correct < -.3):
            return 1
        else:
            return 0

    def manageBoost(self, correct):
        if ((correct < .2) and (correct > -.2)):
            return 1
        else:
            return 0
    def find_correction_normal(self, current: Vec3, ideal: Vec3) -> float:
        # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.
        target = math.pi * 2
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(current.y, -current.x)
        ideal_in_radians = math.atan2(ideal.y, -ideal.x)
        diff = (ideal_in_radians - current_in_radians)
        if (diff >= 0):
            diff = diff - math.pi
        else:
            diff = diff + math.pi
        diff = diff + (math.pi * 2)
        # At this ponit diff is between 0 and 12
        error = target - diff
        normal = self.map(error, -math.pi, math.pi, -1, 1)
        self.error = error
        return normal

    def find_correction(self, current: Vec3, ideal: Vec3) -> float:
        # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.
        target = math.pi * 2
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(current.y, -current.x)
        ideal_in_radians = math.atan2(ideal.y, -ideal.x)
        diff = (ideal_in_radians - current_in_radians)
        if (diff >= 0):
            diff = diff - math.pi
        else:
            diff = diff + math.pi
        diff = diff + (math.pi * 2)
        # At this ponit diff is between 0 and 12
        error = target - diff
        correct = self.pid.Update(error)
        # correction value acquired now map: v2: no longer need to map instead clip
        if (correct < -1):
            correct = -1
        elif (correct > 1):
            correct = 1
        # Make sure that diff is between -pi and +pi.
        return correct
