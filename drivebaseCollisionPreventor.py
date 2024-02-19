from math import sin, cos, asin

class Point():
    def __init__(self, x: float, y :float) -> None:
        self.x = x
        self.y = y

    def add(self, point):
        self.x += point.x
        self.y += point.y

class RoboState():
    def __init__(self, pivotRads, wristRads, telescopeMeters) -> None:
        self.pivotRads = pivotRads
        self.wristRads = wristRads
        self.telescopeMeters = telescopeMeters

class Rectangle():
    def __init__(self, x: float, y: float, width: float, height: float) -> None:
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.left = x
        self.right = x + width
        self.bottom = y
        self.top = y + height

class RobotGeometry():
    def __init__(self) -> None:
        '''
        Represents the current status of the robot's geometry.
        Imagining the robot from the side in an orientation
        where the umbrella extends out of the right side of
        the drive base, the bottom left corner of the drive 
        base is (0,0).
        '''
        self.stemLength: float = 0.0 # Dynamic | Includes the telescope (always)
        self.pivotRadians: float = 0.0 # Dynamic
        self.wristLength: float = 0.0 # Dynamic
        self.wristRadians: float = 0.0 # Dynamic
        self.driveBaseRectangle: Rectangle = Rectangle(0.0, 0.0, 0.0, 0.0) # Constant
        self.pivotAxelPoint: Point = Point(0.0, 0.0) # Constant

def computeIntersection(pt1: Point, pt2: Point, y: float) -> float:
    m = (pt2.y - pt1.y) / (pt2.x - pt1.x)
    b = (pt2.y - (m * pt2.x)) / m
    return (y - (m * b)) / m

# Should be updated every cycle (represents the current geometry of the robot RIGHT NOW)
currentRobotGeometry: RobotGeometry = RobotGeometry()

def isValidState(state: RoboState) -> bool:

    # Point defining where the wrist axel is from (0,0)
    wristAxelPoint: Point = Point(
        (state.telescopeMeters * cos(state.pivotRads)),
        state.telescopeMeters * sin(state.pivotRads)
    )
    wristAxelPoint.add(currentRobotGeometry.pivotAxelPoint)

    wristVector: Point = Point(
        currentRobotGeometry.stemLength * cos(state.wristRads - state.pivotRads),
        currentRobotGeometry.stemLength * -1.0 * sin(state.wristRads - state.pivotRads)
    )

    # Point defining where the wrist bottom right corner is from (0,0)
    wristEndPoint: Point = wristAxelPoint.add(wristVector)

    # The x coordinates of where on the top and bottom of the drive base's 
    # y coordinates intesects with the function of the line of the angle 
    # of the wrist with the wrist length from the wrist axel.
    wristDriveBaseTopIntersect: float = computeIntersection(
        wristAxelPoint, 
        wristEndPoint, 
        currentRobotGeometry.driveBaseRectangle.top)
    wristDriveBaseBottomIntersect: float = computeIntersection(
        wristAxelPoint,
        wristEndPoint,
        currentRobotGeometry.driveBaseRectangle.bottom
    )

    # The x coordinates of where on the top and bottom of the drive base's 
    # y coordinates intesects with the function of the line of the angle 
    # of the pivot with the stem length.
    telescopeDriveBaseTopIntersect: float = computeIntersection(
        currentRobotGeometry.pivotAxelPoint,
        wristAxelPoint,
        currentRobotGeometry.driveBaseRectangle.top
    )
    telescopeDriveBaseBottomIntersect: float = computeIntersection(
        currentRobotGeometry.pivotAxelPoint,
        wristAxelPoint,
        currentRobotGeometry.driveBaseRectangle.bottom
    )

    # Boolean logic for recognizing when the intersection points on the drive
    # base are within the interval of the drive base left and right side.
    # Also boolean logic to determine when the actual mechanism is below the
    # drive base's top coordinate.
    isTopWristInRect: bool = wristDriveBaseTopIntersect <= currentRobotGeometry.driveBaseRectangle.right and wristDriveBaseTopIntersect >= currentRobotGeometry.driveBaseRectangle.left
    isBottomWristInRect: bool = wristDriveBaseBottomIntersect <= currentRobotGeometry.driveBaseRectangle.right and wristDriveBaseBottomIntersect >= currentRobotGeometry.driveBaseRectangle.left
    isWristEndPointBelowRectTop: bool = wristEndPoint <= currentRobotGeometry.driveBaseRectangle.top

    isTopStemInRect: bool = telescopeDriveBaseTopIntersect <= currentRobotGeometry.driveBaseRectangle.right and telescopeDriveBaseTopIntersect >= currentRobotGeometry.driveBaseRectangle.left
    isBottomStemInRect: bool = telescopeDriveBaseBottomIntersect <= currentRobotGeometry.driveBaseRectangle.right and telescopeDriveBaseBottomIntersect >= currentRobotGeometry.driveBaseRectangle.left
    isWristAxelPointBelowRectTop: bool = wristAxelPoint <= currentRobotGeometry.driveBaseRectangle.top

    isUmbrellaValid: bool = isWristEndPointBelowRectTop and (isTopWristInRect or isBottomWristInRect)
    isStemValid: bool = isWristAxelPointBelowRectTop and (isTopStemInRect or isBottomStemInRect)

    if (isUmbrellaValid == False or isStemValid == False):
        return False
    
    return True

def stepTowardsTargetState(targetState: RoboState) -> RoboState:
    currentState: RoboState = RoboState(
        currentRobotGeometry.pivotRadians,
        currentRobotGeometry.wristRadians,
        currentRobotGeometry.stemLength
    )

    # Determine which mechanism can and can't move towards the target state at the moment
    pivotMovementValid: bool = isValidState(RoboState(targetState.pivotRads, currentState.wristRads, currentState.telescopeMeters))
    wristMovementValid: bool = isValidState(RoboState(currentState.pivotRads, targetState.wristRads, currentState.telescopeMeters))
    telescopeMovementValid: bool = isValidState(RoboState(currentState.pivotRads, currentState.wristRads, targetState.telescopeMeters))

    # Generate new state that stunts mechanism that can't move from moving at the moment
    midStatePivotRads = targetState.pivotRads if pivotMovementValid else currentState.pivotRads
    midStateWristRads = targetState.wristRads if wristMovementValid else currentState.wristRads
    midStateTelescopeMeters = targetState.telescopeMeters if telescopeMovementValid else currentState.telescopeMeters

    if (pivotMovementValid == False):
        invalidState: RoboState = RoboState(targetState.pivotRads, currentState.wristRads, currentState.telescopeMeters) 

        # Point defining where the wrist axel is from (0,0)
        wristAxelPoint: Point = Point(
            (invalidState.telescopeMeters * cos(invalidState.pivotRads)),
            invalidState.telescopeMeters * sin(invalidState.pivotRads)
        )
        wristAxelPoint.add(currentRobotGeometry.pivotAxelPoint)

        wristVector: Point = Point(
            currentRobotGeometry.stemLength * cos(invalidState.wristRads - invalidState.pivotRads),
            currentRobotGeometry.stemLength * -1.0 * sin(invalidState.wristRads - invalidState.pivotRads)
        )

        # Point defining where the wrist bottom right corner is from (0,0)
        wristEndPoint: Point = wristAxelPoint.add(wristVector)

        # The distance from the drive base top y coordinate that the umbrella
        # is going past in the invalid state
        intersectionDifference: float = abs(wristAxelPoint.y - currentRobotGeometry.driveBaseRectangle.top)

        # How much higher than the drive base the umbrella needs to be to
        # not be intersecting with the drive base
        minimumDistFromDriveBaseTop: float = (currentRobotGeometry.stemLength * sin(invalidState.pivotRads)) + intersectionDifference

        # The new pivot angle that acheives the minimum distance from the drive
        # base top the umbrella needs to be in order to not intersect the drive base
        midStatePivotRads = asin(minimumDistFromDriveBaseTop / currentRobotGeometry.stemLength)

    midState: RoboState = RoboState(midStatePivotRads, midStateWristRads, midStateTelescopeMeters)
    return midState