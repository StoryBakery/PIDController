--!native

local PIDController = require("./PIDController")

export type Vector3PIDControllerParams = {
	Kp: vector?,
	Ki: vector?,
	Kd: vector?,
	Setpoint: vector?,
	InitialMeasurement: vector?,
	OutputMin: vector?,
	OutputMax: vector?,
	IntegralMin: vector?,
	IntegralMax: vector?,
	DerivativeMode: PIDController.DerivativeMode?,
}

local Vector3PIDController = {}
Vector3PIDController.__index = Vector3PIDController

export type Vector3PIDController = setmetatable<{
	Setpoint: vector,
	_x: PIDController.PIDController,
	_y: PIDController.PIDController,
	_z: PIDController.PIDController,
}, typeof(Vector3PIDController)>

local function axisFromVector(value: vector?, axis: "X" | "Y" | "Z"): number?
	if value == nil then
		return nil
	end
	if axis == "X" then
		return value.x
	elseif axis == "Y" then
		return value.y
	end
	return value.z
end

local function makeAxisParams(params: Vector3PIDControllerParams, axis: "X" | "Y" | "Z")
	return {
		Kp = axisFromVector(params.Kp, axis),
		Ki = axisFromVector(params.Ki, axis),
		Kd = axisFromVector(params.Kd, axis),
		Setpoint = axisFromVector(params.Setpoint, axis),
		InitialMeasurement = axisFromVector(params.InitialMeasurement, axis),
		OutputMin = axisFromVector(params.OutputMin, axis),
		OutputMax = axisFromVector(params.OutputMax, axis),
		IntegralMin = axisFromVector(params.IntegralMin, axis),
		IntegralMax = axisFromVector(params.IntegralMax, axis),
		DerivativeMode = params.DerivativeMode,
	}
end

function Vector3PIDController.new(params: Vector3PIDControllerParams?): Vector3PIDController
	params = params or {}

	local xController = PIDController.new(makeAxisParams(params, "X"))
	local yController = PIDController.new(makeAxisParams(params, "Y"))
	local zController = PIDController.new(makeAxisParams(params, "Z"))

	local self = setmetatable({
		Setpoint = vector.create(xController.Setpoint, yController.Setpoint, zController.Setpoint),
		_x = xController,
		_y = yController,
		_z = zController,
	}, Vector3PIDController)

	return self
end

function Vector3PIDController:SetTunings(kp: vector, ki: vector, kd: vector)
	self._x:SetTunings(kp.x, ki.x, kd.x)
	self._y:SetTunings(kp.y, ki.y, kd.y)
	self._z:SetTunings(kp.z, ki.z, kd.z)
end

function Vector3PIDController:SetKp(kp: vector)
	self._x:SetKp(kp.x)
	self._y:SetKp(kp.y)
	self._z:SetKp(kp.z)
end

function Vector3PIDController:SetKi(ki: vector)
	self._x:SetKi(ki.x)
	self._y:SetKi(ki.y)
	self._z:SetKi(ki.z)
end

function Vector3PIDController:SetKd(kd: vector)
	self._x:SetKd(kd.x)
	self._y:SetKd(kd.y)
	self._z:SetKd(kd.z)
end

function Vector3PIDController:SetDerivativeMode(mode: PIDController.DerivativeMode)
	self._x:SetDerivativeMode(mode)
	self._y:SetDerivativeMode(mode)
	self._z:SetDerivativeMode(mode)
end

function Vector3PIDController:SetSetpoint(setpoint: vector)
	self.Setpoint = setpoint
	self._x:SetSetpoint(setpoint.x)
	self._y:SetSetpoint(setpoint.y)
	self._z:SetSetpoint(setpoint.z)
end

function Vector3PIDController:SetOutputLimits(minValue: vector, maxValue: vector)
	self._x:SetOutputLimits(minValue.x, maxValue.x)
	self._y:SetOutputLimits(minValue.y, maxValue.y)
	self._z:SetOutputLimits(minValue.z, maxValue.z)
end

function Vector3PIDController:SetIntegralLimits(minValue: vector, maxValue: vector)
	self._x:SetIntegralLimits(minValue.x, maxValue.x)
	self._y:SetIntegralLimits(minValue.y, maxValue.y)
	self._z:SetIntegralLimits(minValue.z, maxValue.z)
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

function Vector3PIDController:Reset(measurement: vector?)
	if measurement then
		self._x:Reset(measurement.x)
		self._y:Reset(measurement.y)
		self._z:Reset(measurement.z)
	else
		self._x:Reset(nil)
		self._y:Reset(nil)
		self._z:Reset(nil)
	end
end

function Vector3PIDController:GetMeasurement(): vector?
	local xMeasurement = self._x:GetMeasurement()
	local yMeasurement = self._y:GetMeasurement()
	local zMeasurement = self._z:GetMeasurement()
	if xMeasurement == nil or yMeasurement == nil or zMeasurement == nil then
		return nil
	end

	return vector.create(xMeasurement, yMeasurement, zMeasurement)
end

function Vector3PIDController:Update(dt: number, measurement: vector): vector
	return vector.create(
		self._x:Update(dt, measurement.x),
		self._y:Update(dt, measurement.y),
		self._z:Update(dt, measurement.z)
	)
end

return Vector3PIDController
