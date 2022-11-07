package RobotParts

import RobotParts.Pose
import RobotParts.RobotPosition.robotPose
import Utils.MathUtils
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.cos
import kotlin.math.sin

object KtPoseTest{
    fun updatePosPoseExp(baseDelta:Pose, finalAngle: Double) {
        val dtheta = baseDelta.angle
        val (sineTerm, cosTerm) = if (dtheta epsilonEquals 0.0) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1.0 - cos(dtheta)) / dtheta
        }

        val move = sineTerm * baseDelta.y - cosTerm * baseDelta.x
        val strafe = cosTerm * baseDelta.y + sineTerm * baseDelta.x

        val pointDelta = Point(
                strafe,
                move
        )

        val finalDelta = MathUtils.pointDelta(pointDelta, robotPose.angle)
        robotPose = Pose(robotPose.x+finalDelta.x, robotPose.y + finalDelta.y, finalAngle)
    }
}