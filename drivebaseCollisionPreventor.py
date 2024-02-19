from math import sin, cos, asin


class Translation2d():
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def add(self, point: 'Translation2d'):
        self.x += point.x
        self.y += point.y

class Rectangle2d():
    def __init__(self, x: float, y: float, width: float, height: float) -> None:
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.left = x
        self.right = x + width
        self.bottom = y
        self.top = y + height


class StemPosition():
    def __init__(self, pivotRads, wristRads, telescopeMeters) -> None:
        self.pivotRads = pivotRads
        self.wristRads = wristRads
        self.telescopeMeters = telescopeMeters


class RobotGeometry():
    def __init__(self) -> None:
        '''
        Represents the current status of the robot's geometry.
        Imagining the robot from the side in an orientation
        where the umbrella extends out of the right side of
        the drive base, the bottom left corner of the drive 
        base is (0,0).
        '''
        self.umbrellaLength: float = 0.0  # Constant | From wrist axel
        # Constant | The farthest side of the umbrella bounding box
        self.umbrellaHeight: float = 0.0
        self.driveBaseRectangle: Rectangle2d = Rectangle2d(
            0.0, 0.0, 0.0, 0.0)  # Constant
        self.pivotAxelPoint: Translation2d = Translation2d(
            0.0, 0.0)  # Constant


def computeIntersection(pt1: Translation2d, pt2: Translation2d, y: float) -> float:
    m = (pt2.y - pt1.y) / (pt2.x - pt1.x)
    b = (pt2.y - (m * pt2.x)) / m
    return (y - (m * b)) / m


# Should be updated every cycle (represents the current geometry of the robot RIGHT NOW)
ROBOT_GEOMETRY: RobotGeometry = RobotGeometry()


def isValidState(state: StemPosition) -> bool:

    # Point defining where the wrist axel is from (0,0)
    wristAxelPoint: Translation2d = Translation2d(
        (state.telescopeMeters * cos(state.pivotRads)),
        state.telescopeMeters * sin(state.pivotRads)
    )
    # Pivot axel point is constant that doesnt change
    wristAxelPoint.add(ROBOT_GEOMETRY.pivotAxelPoint)

    wristVector: Translation2d = Translation2d(
        state.telescopeMeters * cos(state.wristRads - state.pivotRads),
        state.telescopeMeters * -1.0 * sin(state.wristRads - state.pivotRads)
    )

    # Point defining where the umbrella bottom right corner is from (0,0)
    wristEndPoint: Translation2d = wristAxelPoint.add(wristVector)

    # Point deifning where the umbrella top right corner is from (0,0) | UNTESTED
    wristTopRightVector: Translation2d = Translation2d(
        ROBOT_GEOMETRY.umbrellaHeight * cos(state.wristRads - state.pivotRads),
        ROBOT_GEOMETRY.umbrellaHeight * sin(state.wristRads - state.pivotRads)
    )
    wristTopRightVector.add(wristEndPoint)

    # Checks if the umbrella is impacting the floor
    if (wristEndPoint.y < 0 or wristTopRightVector.y < 0):
        return False

    # The x coordinates of where on the top and bottom of the drive base's
    # y coordinates intesects with the function of the line of the angle
    # of the wrist with the wrist length from the wrist axel.
    wristDriveBaseTopIntersect: float = computeIntersection(
        wristAxelPoint,
        wristEndPoint,
        ROBOT_GEOMETRY.driveBaseRectangle.top)
    wristDriveBaseBottomIntersect: float = computeIntersection(
        wristAxelPoint,
        wristEndPoint,
        ROBOT_GEOMETRY.driveBaseRectangle.bottom
    )

    # The x coordinates of where on the top and bottom of the drive base's
    # y coordinates intesects with the function of the line of the angle
    # of the pivot with the stem length.
    telescopeDriveBaseTopIntersect: float = computeIntersection(
        ROBOT_GEOMETRY.pivotAxelPoint,
        wristAxelPoint,
        ROBOT_GEOMETRY.driveBaseRectangle.top
    )
    telescopeDriveBaseBottomIntersect: float = computeIntersection(
        ROBOT_GEOMETRY.pivotAxelPoint,
        wristAxelPoint,
        ROBOT_GEOMETRY.driveBaseRectangle.bottom
    )

    # Boolean logic for recognizing when the intersection points on the drive
    # base are within the interval of the drive base left and right side.
    # Also boolean logic to determine when the actual mechanism is below the
    # drive base's top coordinate.
    isTopWristInRect: bool = wristDriveBaseTopIntersect <= ROBOT_GEOMETRY.driveBaseRectangle.right and wristDriveBaseTopIntersect >= ROBOT_GEOMETRY.driveBaseRectangle.left
    isBottomWristInRect: bool = wristDriveBaseBottomIntersect <= ROBOT_GEOMETRY.driveBaseRectangle.right and wristDriveBaseBottomIntersect >= ROBOT_GEOMETRY.driveBaseRectangle.left
    isWristEndPointBelowRectTop: bool = wristEndPoint <= ROBOT_GEOMETRY.driveBaseRectangle.top

    isTopStemInRect: bool = telescopeDriveBaseTopIntersect <= ROBOT_GEOMETRY.driveBaseRectangle.right and telescopeDriveBaseTopIntersect >= ROBOT_GEOMETRY.driveBaseRectangle.left
    isBottomStemInRect: bool = telescopeDriveBaseBottomIntersect <= ROBOT_GEOMETRY.driveBaseRectangle.right and telescopeDriveBaseBottomIntersect >= ROBOT_GEOMETRY.driveBaseRectangle.left
    isWristAxelPointBelowRectTop: bool = wristAxelPoint <= ROBOT_GEOMETRY.driveBaseRectangle.top

    isUmbrellaValid: bool = isWristEndPointBelowRectTop and (
        isTopWristInRect or isBottomWristInRect)
    isStemValid: bool = isWristAxelPointBelowRectTop and (
        isTopStemInRect or isBottomStemInRect)

    if (isUmbrellaValid == False or isStemValid == False):
        return False

    return True


def stepTowardsTargetState(currentState: StemPosition, targetState: StemPosition) -> StemPosition:
    # Determine which mechanism can and can't move towards the target state at the moment
    pivotMovementValid: bool = isValidState(StemPosition(
        targetState.pivotRads, currentState.wristRads, currentState.telescopeMeters))
    wristMovementValid: bool = isValidState(StemPosition(
        currentState.pivotRads, targetState.wristRads, currentState.telescopeMeters))
    telescopeMovementValid: bool = isValidState(StemPosition(
        currentState.pivotRads,
        currentState.wristRads,
        targetState.telescopeMeters
    ))

    # Generate new state that stunts mechanism that can't move from moving at the moment
    midStatePivotRads = targetState.pivotRads if pivotMovementValid else currentState.pivotRads
    midStateWristRads = targetState.wristRads if wristMovementValid else currentState.wristRads
    midStateTelescopeMeters = targetState.telescopeMeters if telescopeMovementValid else currentState.telescopeMeters

    if (pivotMovementValid == False):
        invalidState: StemPosition = StemPosition(
            targetState.pivotRads, currentState.wristRads, currentState.telescopeMeters)

        # Point defining where the wrist axel is from (0,0)
        wristAxelPoint: Translation2d = Translation2d(
            (invalidState.telescopeMeters * cos(invalidState.pivotRads)),
            invalidState.telescopeMeters * sin(invalidState.pivotRads)
        )
        wristAxelPoint.add(ROBOT_GEOMETRY.pivotAxelPoint)

        wristVector: Translation2d = Translation2d(
            currentState.telescopeMeters *
            cos(invalidState.wristRads - invalidState.pivotRads),
            currentState.telescopeMeters * -1.0 *
            sin(invalidState.wristRads - invalidState.pivotRads)
        )

        # Point defining where the wrist bottom right corner is from (0,0)
        wristEndPoint: Translation2d = wristAxelPoint.add(wristVector)

        # The distance from the drive base top y coordinate that the umbrella
        # is going past in the invalid state
        intersectionDifference: float = abs(
            wristEndPoint.y - ROBOT_GEOMETRY.driveBaseRectangle.top)

        # How much higher than the drive base the umbrella needs to be to
        # not be intersecting with the drive base
        minimumDistFromDriveBaseTop: float = (
            currentState.telescopeMeters * sin(invalidState.pivotRads)) + intersectionDifference

        # The new pivot angle that acheives the minimum distance from the drive
        # base top the umbrella needs to be in order to not intersect the drive base
        midStatePivotRads = asin(
            minimumDistFromDriveBaseTop / currentState.telescopeMeters)

    return StemPosition(midStatePivotRads, midStateWristRads, midStateTelescopeMeters)