local AnglePIDController = require("@self/AnglePIDController")
local PIDController = require("@self/PIDController")
local Vector3PIDController = require("@self/Vector3PIDController")

local PIDControllers = {
	new = PIDController.new,
	createAnglePID = AnglePIDController.new,
	createVector3PID = Vector3PIDController.new,
}

export type PIDController = PIDController.PIDController
export type AnglePIDController = AnglePIDController.AnglePIDController
export type Vector3PIDController = Vector3PIDController.Vector3PIDController

return PIDControllers
