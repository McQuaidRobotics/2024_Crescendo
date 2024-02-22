using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Reflection.Metadata;
using System.Xml.Linq;

namespace StemSolvers
{
    internal class StateTransitionHandler
    {
        private const float DEGREES_TO_RADIANS = (float) Math.PI / 180.0f;
        private const float RADIANS_TO_DEGREES = 180.0f / (float) Math.PI;

        private float stemLength;
        private float pivotRadians;
        private float wristLength;
        private float wristRadians;
        private Rectangle driveBaseRectangle;
        private RoboState targetState;
        private Robot robot;
        private bool buttonState;

        private enum InvalidationReasons
        {
            NONE,
            DRIVE_BASE,
            WALLS,
            DRIVE_BASE_AND_WALLS
        }

        public StateTransitionHandler(Robot robot)
        {
            this.robot = robot;
            this.targetState = new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels());
            updateRobotMechDimensions();

            this.buttonState = Keyboard.GetState().IsKeyDown(Keys.Up);
        }

        public void updateRobotMechDimensions()
        {
            this.stemLength = robot.getTelescopePixels();
            this.pivotRadians = robot.getPivotDegrees() * DEGREES_TO_RADIANS;
            this.wristLength = robot.getUmbrellaLength();
            this.wristRadians = robot.getWristDegrees() * DEGREES_TO_RADIANS;
            this.driveBaseRectangle = robot.getDriveBaseRect();
        }

        public void transitionTo(RoboState state)
        {
            if (isValidState(state) == InvalidationReasons.NONE)
                targetState = state;
        }

        private float getLineIntercept(Vector2 pt1, Vector2 pt2, float y)
        {
            float m = (pt2.Y - pt1.Y) / (pt2.X - pt1.X);
            float b = (pt2.Y - (m * pt2.X)) / m;
            return (y - (m * b)) / m;
        }

        private InvalidationReasons isValidState(RoboState state)
        {
            MechanismPoints mechPts = new MechanismPoints(state, robot);

            float wristDriveBaseTopIntercept = getLineIntercept(mechPts.umbrellaBottomLeftPoint, new Vector2(mechPts.umbrellaBottomRightPoint.X + 0.0001f, mechPts.umbrellaBottomRightPoint.Y + 0.0001f), driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            float wristDriveBaseBottomIntercept = getLineIntercept(mechPts.umbrellaBottomLeftPoint, new Vector2(mechPts.umbrellaBottomRightPoint.X + 0.0001f, mechPts.umbrellaBottomRightPoint.Y + 0.0001f), driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            float telescopeDriveBaseTopIntercept = getLineIntercept(new Vector2(robot.getTelescopeRect().X + 0.0001f, robot.getTelescopeRect().Y + 0.0001f), mechPts.wristAxelPoint, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            float telescopeDriveBaseBottomIntercept = getLineIntercept(new Vector2(robot.getTelescopeRect().X + 0.0001f, robot.getTelescopeRect().Y + 0.0001f), mechPts.wristAxelPoint, driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            //FLIP BOTTOM AND TOP BECAUSE OF GAME WORLD

            bool topWristIntInBase = (wristDriveBaseTopIntercept <= driveBaseRectangle.Right - (driveBaseRectangle.Width / 2) && wristDriveBaseTopIntercept >= driveBaseRectangle.Left - (driveBaseRectangle.Width / 2));
            bool bottomWristIntInBase = (wristDriveBaseBottomIntercept <= driveBaseRectangle.Right - (driveBaseRectangle.Width / 2) && wristDriveBaseBottomIntercept >= driveBaseRectangle.Left - (driveBaseRectangle.Width / 2));
            bool isWristRightPointsBelowBase = (mechPts.umbrellaBottomRightPoint.Y <= driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2)) || mechPts.umbrellaTopRightPoint.Y <= driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2);

            bool topTeleIntInBase = (telescopeDriveBaseTopIntercept <= driveBaseRectangle.Right - (driveBaseRectangle.Width / 2) && telescopeDriveBaseTopIntercept >= driveBaseRectangle.Left - (driveBaseRectangle.Width / 2));
            bool bottomTeleIntInBase = (telescopeDriveBaseBottomIntercept <= driveBaseRectangle.Right - (driveBaseRectangle.Width / 2) && telescopeDriveBaseBottomIntercept >= driveBaseRectangle.Left - (driveBaseRectangle.Width / 2));
            bool isTeleEndPointBelowBase = mechPts.wristAxelPoint.Y <= driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2);

            if (((isWristRightPointsBelowBase && (topWristIntInBase || bottomWristIntInBase)) || 
                (isTeleEndPointBelowBase && (topTeleIntInBase || bottomTeleIntInBase))) &&
                (!robot.getAllowedBounds().Contains(mechPts.wristAxelPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaBottomLeftPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaBottomRightPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaTopLeftPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaTopRightPoint)))
                    return InvalidationReasons.DRIVE_BASE_AND_WALLS;
            if ((isWristRightPointsBelowBase && (topWristIntInBase || bottomWristIntInBase)) || 
                (isTeleEndPointBelowBase && (topTeleIntInBase || bottomTeleIntInBase)))
                    return InvalidationReasons.DRIVE_BASE;
            if (!robot.getAllowedBounds().Contains(mechPts.wristAxelPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaBottomLeftPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaBottomRightPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaTopLeftPoint) ||
                !robot.getAllowedBounds().Contains(mechPts.umbrellaTopRightPoint))
                    return InvalidationReasons.WALLS;

            return InvalidationReasons.NONE;
        }

        private RoboState stepTowardsTargetState()
        {
            RoboState currentState = new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels());

            InvalidationReasons pivotMovementValid = isValidState(new RoboState(targetState.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels()));
            InvalidationReasons wristMovementValid = isValidState(new RoboState(robot.getPivotDegrees(), targetState.getWristDegrees(), robot.getTelescopePixels()));
            InvalidationReasons telescopeMovementValid = isValidState(new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), targetState.getTelescopePixels()));

            float midStatePivotDegrees = pivotMovementValid != InvalidationReasons.NONE ? currentState.getPivotDegrees() : targetState.getPivotDegrees();
            float midStateWristDegrees = wristMovementValid != InvalidationReasons.NONE ? currentState.getWristDegrees() : targetState.getWristDegrees();
            float midStateTelescopePixels = telescopeMovementValid != InvalidationReasons.NONE ? currentState.getTelescopePixels() : targetState.getTelescopePixels();

            if (pivotMovementValid != InvalidationReasons.NONE)
            {
                RoboState invalidState = new RoboState(targetState.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels());
                
                MechanismPoints mechPts = new MechanismPoints(invalidState, robot);
                Rectangle umbrellaRect = mechPts.getCastedRect();

                if (pivotMovementValid == InvalidationReasons.DRIVE_BASE || pivotMovementValid == InvalidationReasons.DRIVE_BASE_AND_WALLS)
                {
                    float lowestOfUmbrellaPoints = Math.Min(mechPts.umbrellaBottomRightPoint.Y, mechPts.umbrellaTopRightPoint.Y); // should be math.min but is opposite bc of game world
                    float difference = Math.Abs(lowestOfUmbrellaPoints - (robot.getDriveBaseRect().Bottom - (robot.getDriveBaseRect().Height / 2))); // shout be rect top but flipped ciz game world
                    float opposite = (robot.getTelescopePixels() * (float)Math.Sin(invalidState.getPivotDegrees() * DEGREES_TO_RADIANS)) + difference;
                    midStatePivotDegrees = (float)(Math.Asin(opposite / robot.getTelescopePixels())) * RADIANS_TO_DEGREES;
                }
                else if (pivotMovementValid == InvalidationReasons.WALLS)
                {
                    //ALL TOPS AND BOTTOMS ARE REVERSED CUZ GAME WORLD

                    float leftDiff = umbrellaRect.Left < robot.getAllowedBounds().Left ? Math.Abs(umbrellaRect.Left - robot.getAllowedBounds().Left) : 0;
                    float rightDiff = umbrellaRect.Right > robot.getAllowedBounds().Right ? Math.Abs(umbrellaRect.Right - robot.getAllowedBounds().Right) : 0;
                    float topDiff = umbrellaRect.Bottom > robot.getAllowedBounds().Bottom ? Math.Abs(umbrellaRect.Bottom - robot.getAllowedBounds().Bottom) : 0; // should be greater than, but opposite cuz game world
                    float bottomDiff = umbrellaRect.Top < robot.getAllowedBounds().Top ? Math.Abs(umbrellaRect.Top - robot.getAllowedBounds().Top) : 0; // should be less than, but opposite cuz game world

                    Vector2 midWristAxelPoint = mechPts.wristAxelPoint;
                    midWristAxelPoint.X += leftDiff;
                    midWristAxelPoint.X -= rightDiff;
                    midWristAxelPoint.Y -= topDiff; // should subtract to move down when in real world, but is opposite cuz game world
                    midWristAxelPoint.Y += bottomDiff; // should add to move up when in real world, but is opposite cuz game world

                    Vector2 pivotAxelToWristAxelSlope = new Vector2(midWristAxelPoint.X - robot.getTelescopeRect().X, midWristAxelPoint.Y - robot.getTelescopeRect().Y); // invert x2 and x1 since game world is opposite | slope formula you get it its 4am AHHHHH
                    midStateTelescopePixels = (float) Math.Sqrt(Math.Pow(pivotAxelToWristAxelSlope.X, 2) + Math.Pow(pivotAxelToWristAxelSlope.Y, 2));
                    midStatePivotDegrees = (float)(Math.Asin(pivotAxelToWristAxelSlope.Y / midStateTelescopePixels)) * RADIANS_TO_DEGREES;
                }
            }

            RoboState midState = new RoboState(midStatePivotDegrees, midStateWristDegrees, midStateTelescopePixels);

            Debug.WriteLine("midState: { Pivot Deg: " + midState.getPivotDegrees() + ", Wrist Deg: " + midState.getWristDegrees() + ", TelescopePixels: " + midState.getTelescopePixels() + " }\n");
            return midState;
        }

        public void update()
        {
            updateRobotMechDimensions();
            //if (Keyboard.GetState().IsKeyDown(Keys.Up) && Keyboard.GetState().IsKeyDown(Keys.Up) != buttonState)
                robot.moveToState(stepTowardsTargetState());
            //buttonState = Keyboard.GetState().IsKeyDown(Keys.Up);
        }

        public void debugDraw(SpriteBatch spriteBatch)
        {
            MechanismPoints mechPts = new MechanismPoints(new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels()), robot);

            float wristDriveBaseTopIntercept = getLineIntercept(mechPts.umbrellaBottomLeftPoint, mechPts.umbrellaBottomRightPoint, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            float wristDriveBaseBottomIntercept = getLineIntercept(mechPts.umbrellaBottomLeftPoint, mechPts.umbrellaBottomRightPoint, driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2)); 

            float telescopeDriveBaseTopIntersect = getLineIntercept(new Vector2(robot.getTelescopeRect().X, robot.getTelescopeRect().Y), mechPts.wristAxelPoint, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            float telescopeDriveBaseBottomIntersect = getLineIntercept(new Vector2(robot.getTelescopeRect().X, robot.getTelescopeRect().Y), mechPts.wristAxelPoint, driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            //------------------------------------------------------------------------------------------------------------------//

            Texture2D texture = new Texture2D(spriteBatch.GraphicsDevice, 1, 1, false, SurfaceFormat.Color);
            texture.SetData(new[] { Color.White });

            Vector2 topWristPos = new Vector2(wristDriveBaseTopIntercept, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            Vector2 bottomWristPos = new Vector2(wristDriveBaseBottomIntercept,driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            Vector2 topTelePos = new Vector2(telescopeDriveBaseTopIntersect, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            Vector2 bottomTelePos = new Vector2(telescopeDriveBaseBottomIntersect,driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            spriteBatch.Draw(texture, new Rectangle((int)mechPts.wristAxelPoint.X, (int)mechPts.wristAxelPoint.Y, 5, 5), Color.OrangeRed);
            spriteBatch.Draw(texture, new Rectangle((int)mechPts.umbrellaBottomRightPoint.X, (int)mechPts.umbrellaBottomRightPoint.Y, 5, 5), Color.OrangeRed);
            spriteBatch.Draw(texture, new Rectangle((int)mechPts.umbrellaTopRightPoint.X, (int)mechPts.umbrellaTopRightPoint.Y, 5, 5), Color.OrangeRed);
            spriteBatch.Draw(texture, new Rectangle((int)mechPts.umbrellaTopLeftPoint.X, (int)mechPts.umbrellaTopLeftPoint.Y, 5, 5), Color.OrangeRed);
            spriteBatch.Draw(texture, new Rectangle((int)mechPts.umbrellaBottomLeftPoint.X, (int)mechPts.umbrellaBottomLeftPoint.Y, 5, 5), Color.OrangeRed);
            //spriteBatch.Draw(texture, mechPts.getCastedRect(), Color.OrangeRed);

            spriteBatch.Draw(texture, new Rectangle((int)topWristPos.X - 2, (int)topWristPos.Y - 2, 5, 5), Color.Black);
            spriteBatch.Draw(texture, new Rectangle((int)bottomWristPos.X - 2, (int)bottomWristPos.Y - 5, 5, 5), Color.Black);

            spriteBatch.Draw(texture, new Rectangle((int) topTelePos.X - 2, (int) topTelePos.Y - 2, 5, 5), Color.Black);
            spriteBatch.Draw(texture, new Rectangle((int) bottomTelePos.X - 2, (int) bottomTelePos.Y - 5, 5, 5), Color.Black);
        }

        public void debugInfo(SpriteBatch spriteBatch)
        {
            MechanismPoints mechPts = new MechanismPoints(new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels()), robot);

            float wristDriveBaseTopIntercept = getLineIntercept(mechPts.wristAxelPoint, mechPts.umbrellaBottomRightPoint, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            float wristDriveBaseBottomIntercept = getLineIntercept(mechPts.wristAxelPoint, mechPts.umbrellaBottomRightPoint, driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2)); 

            Vector2 topWristPos = new Vector2(wristDriveBaseTopIntercept, driveBaseRectangle.Top - (driveBaseRectangle.Height / 2));
            Vector2 bottomWristPos = new Vector2(wristDriveBaseBottomIntercept,driveBaseRectangle.Bottom - (driveBaseRectangle.Height / 2));

            spriteBatch.DrawString(Game1.debugFont, "Current State Valid: " + isValidState(new RoboState(robot.getPivotDegrees(), robot.getWristDegrees(), robot.getTelescopePixels())).ToString(), new Vector2(10, 230), Color.LimeGreen);
            spriteBatch.DrawString(Game1.debugFont, "Top-Int: " + topWristPos.ToString(), new Vector2(10, 90), Color.LimeGreen);
            spriteBatch.DrawString(Game1.debugFont, "Bottom-Int: " + bottomWristPos.ToString(), new Vector2(10, 110), Color.LimeGreen);
            spriteBatch.DrawString(Game1.debugFont, "Target State Pivot Deg: " + targetState.getPivotDegrees().ToString(), new Vector2(10, 130), Color.LimeGreen);
            spriteBatch.DrawString(Game1.debugFont, "Target State Wrist Deg: " + targetState.getWristDegrees().ToString(), new Vector2(10, 150), Color.LimeGreen);
            spriteBatch.DrawString(Game1.debugFont, "Target State Telescope Pixels: " + targetState.getTelescopePixels().ToString(), new Vector2(10, 170), Color.LimeGreen);
        }
    }
}
