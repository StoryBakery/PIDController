--!native

local PIDController = require("./PIDController")

export type Vector3PIDControllerParams = {
	Kp: number | Vector3?,
	Ki: number | Vector3?,
	Kd: number | Vector3?,
	Setpoint: Vector3?,
	InitialMeasurement: Vector3?,
	OutputMin: number | Vector3?,
	OutputMax: number | Vector3?,
	IntegralMin: number | Vector3?,
	IntegralMax: number | Vector3?,
	DerivativeMode: PIDController.DerivativeMode?,
}

local Vector3PIDController = {}
Vector3PIDController.__index = Vector3PIDController

export type Vector3PIDController = setmetatable<{
	Setpoint: Vector3,
	_x: any,
	_y: any,
	_z: any,
}, typeof(Vector3PIDController)>

local function axisFromVector(value: Vector3?, axis: "X" | "Y" | "Z"): number?
	if value == nil then
		return nil
	end
	if axis == "X" then
		return value.X
	elseif axis == "Y" then
		return value.Y
	end
	return value.Z
end

local function numberOrAxisValue(value: number | Vector3, axis: "X" | "Y" | "Z"): number
	local valueType = typeof(value)
	if valueType == "number" then
		return value :: number
	elseif axis == "X" then
		return (value :: Vector3).X
	elseif axis == "Y" then
		return (value :: Vector3).Y
	end
	return (value :: Vector3).Z
end

local function makeAxisParams(params: Vector3PIDControllerParams, axis: "X" | "Y" | "Z")
	return {
		Kp = params.Kp and numberOrAxisValue(params.Kp, axis) or nil,
		Ki = params.Ki and numberOrAxisValue(params.Ki, axis) or nil,
		Kd = params.Kd and numberOrAxisValue(params.Kd, axis) or nil,
		Setpoint = axisFromVector(params.Setpoint, axis),
		InitialMeasurement = axisFromVector(params.InitialMeasurement, axis),
		OutputMin = params.OutputMin and numberOrAxisValue(params.OutputMin, axis) or nil,
		OutputMax = params.OutputMax and numberOrAxisValue(params.OutputMax, axis) or nil,
		IntegralMin = params.IntegralMin and numberOrAxisValue(params.IntegralMin, axis) or nil,
		IntegralMax = params.IntegralMax and numberOrAxisValue(params.IntegralMax, axis) or nil,
		DerivativeMode = params.DerivativeMode,
	}
end

function Vector3PIDController.new(params: Vector3PIDControllerParams?): Vector3PIDController
	params = params or {}

	local xController = PIDController.new(makeAxisParams(params, "X"))
	local yController = PIDController.new(makeAxisParams(params, "Y"))
	local zController = PIDController.new(makeAxisParams(params, "Z"))

	local self = setmetatable({
		Setpoint = Vector3.new(xController.Setpoint, yController.Setpoint, zController.Setpoint),
		_x = xController,
		_y = yController,
		_z = zController,
	}, Vector3PIDController)

	return self
end

function Vector3PIDController:SetTunings(
	kp: number | Vector3,
	ki: number | Vector3,
	kd: number | Vector3
)
	self._x:SetTunings(
		numberOrAxisValue(kp, "X"),
		numberOrAxisValue(ki, "X"),
		numberOrAxisValue(kd, "X")
	)
	self._y:SetTunings(
		numberOrAxisValue(kp, "Y"),
		numberOrAxisValue(ki, "Y"),
		numberOrAxisValue(kd, "Y")
	)
	self._z:SetTunings(
		numberOrAxisValue(kp, "Z"),
		numberOrAxisValue(ki, "Z"),
		numberOrAxisValue(kd, "Z")
	)
end

function Vector3PIDController:SetKp(kp: number | Vector3)
	self._x:SetKp(numberOrAxisValue(kp, "X"))
	self._y:SetKp(numberOrAxisValue(kp, "Y"))
	self._z:SetKp(numberOrAxisValue(kp, "Z"))
end

function Vector3PIDController:SetKi(ki: number | Vector3)
	self._x:SetKi(numberOrAxisValue(ki, "X"))
	self._y:SetKi(numberOrAxisValue(ki, "Y"))
	self._z:SetKi(numberOrAxisValue(ki, "Z"))
end

function Vector3PIDController:SetKd(kd: number | Vector3)
	self._x:SetKd(numberOrAxisValue(kd, "X"))
	self._y:SetKd(numberOrAxisValue(kd, "Y"))
	self._z:SetKd(numberOrAxisValue(kd, "Z"))
end

function Vector3PIDController:SetDerivativeMode(mode: PIDController.DerivativeMode)
	self._x:SetDerivativeMode(mode)
	self._y:SetDerivativeMode(mode)
	self._z:SetDerivativeMode(mode)
end

function Vector3PIDController:SetSetpoint(setpoint: Vector3)
	self.Setpoint = setpoint
	self._x:SetSetpoint(setpoint.X)
	self._y:SetSetpoint(setpoint.Y)
	self._z:SetSetpoint(setpoint.Z)
end

function Vector3PIDController:SetOutputLimits(
	minValue: number | Vector3,
	maxValue: number | Vector3
)
	self._x:SetOutputLimits(numberOrAxisValue(minValue, "X"), numberOrAxisValue(maxValue, "X"))
	self._y:SetOutputLimits(numberOrAxisValue(minValue, "Y"), numberOrAxisValue(maxValue, "Y"))
	self._z:SetOutputLimits(numberOrAxisValue(minValue, "Z"), numberOrAxisValue(maxValue, "Z"))
end

function Vector3PIDController:SetIntegralLimits(
	minValue: number | Vector3,
	maxValue: number | Vector3
)
	self._x:SetIntegralLimits(numberOrAxisValue(minValue, "X"), numberOrAxisValue(maxValue, "X"))
	self._y:SetIntegralLimits(numberOrAxisValue(minValue, "Y"), numberOrAxisValue(maxValue, "Y"))
	self._z:SetIntegralLimits(numberOrAxisValue(minValue, "Z"), numberOrAxisValue(maxValue, "Z"))
end

function Vector3PIDController:ClearOutputLimits()
	self._x:ClearOutputLimits()
	self._y:ClearOutputLimits()
	self._z:ClearOutputLimits()
end

function Vector3PIDController:ClearIntegralLimits()
	self._x:ClearIntegralLimits()
	self._y:ClearIntegralLimits()
	self._z:ClearIntegralLimits()
end

function Vector3PIDController:Reset(measurement: Vector3?)
	if measurement then
		self._x:Reset(measurement.X)
		self._y:Reset(measurement.Y)
		self._z:Reset(measurement.Z)
	else
		self._x:Reset(nil)
		self._y:Reset(nil)
		self._z:Reset(nil)
	end
end

function Vector3PIDController:GetMeasurement(): Vector3?
	local xMeasurement = self._x:GetMeasurement()
	local yMeasurement = self._y:GetMeasurement()
	local zMeasurement = self._z:GetMeasurement()
	if xMeasurement == nil or yMeasurement == nil or zMeasurement == nil then
		return nil
	end

	return Vector3.new(xMeasurement, yMeasurement, zMeasurement)
end

function Vector3PIDController:Update(dt: number, measurement: Vector3): Vector3
	return Vector3.new(
		self._x:Update(dt, measurement.X),
		self._y:Update(dt, measurement.Y),
		self._z:Update(dt, measurement.Z)
	)
end

return Vector3PIDController
